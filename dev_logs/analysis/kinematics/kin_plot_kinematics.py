import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

C_MOCAP = '#1F77B4'      # Steel Blue (Actual Ground Truth)
C_CMD = '#D62728'        # Crimson Red (Commanded Setpoint)
C_BAT = '#9467BD'        # Muted Purple (Battery Voltage)

def draw_timeline_markers(ax, wp_events, arming_time, y_lims, is_absolute=False, achieved_angle=None, draw_labels=True, label_pos='top', draw_end_label=True):
    """Draws perfectly aligned, standard timeline event markers on the given axis (General Plot Rule 6)."""
    y_min, y_max = y_lims

    esp_time = wp_events.get('WP2')   # refined experiment start (actual forward movement)
    t_impact = wp_events.get('Column Impact')
    eep_time = wp_events.get('WP3')   # experiment end (sweep end achievement)
    t_passed = wp_events.get('Column Center Passed') or wp_events.get('Column Passed')

    # Draw Shared Impact Window Shading (General Plot Rule 7 / SSoT)
    if t_impact is not None:
        t_rel_impact = t_impact if is_absolute else (t_impact - arming_time)
        ax.axvspan(t_rel_impact - 0.05, t_rel_impact + 0.35, color='#D62728', alpha=0.10, zorder=3)

    events = []
    if esp_time is not None:
        events.append(('Exp. Start-point', esp_time, '#9467BD', ':', 1.8, '#444444'))
    if t_passed is not None:
        events.append(('Column Center Passed', t_passed, '#FF9900', '--', 1.8, '#995C00'))
    if t_impact is not None:
        angle_str = f" ({achieved_angle:.1f}°)" if achieved_angle is not None else ""
        events.append((f'Impact{angle_str}', t_impact, '#D62728', '-.', 1.2, '#7F0000'))
    if eep_time is not None:
        events.append(('Exp. End-point', eep_time, '#9467BD', ':', 1.8, '#444444'))

    for idx, (name, t_abs, color, style, width, text_color) in enumerate(events):
        t_val = t_abs if is_absolute else (t_abs - arming_time)
        ax.axvline(x=t_val, color=color, linestyle=style, linewidth=width, alpha=0.95, zorder=5)

        # Exp. Start-point and End-point labels are vertically centred on the line
        if 'Start-point' in name or 'End-point' in name:
            h_pos = y_min + (y_max - y_min) * 0.50
            v_align = 'center'
        # Prevents label text collision by alternating offsets and height positions
        elif label_pos == 'bottom':
            h_fraction = 0.12 + 0.20 * (idx % 3)
            v_align = 'bottom'
        else:
            h_fraction = 0.88 - 0.20 * (idx % 3)
            v_align = 'top'

        if 'Start-point' not in name and 'End-point' not in name:
            h_pos = y_min + (y_max - y_min) * h_fraction

        # Left-align the label for Impact to prevent obscuring post-impact curves (General Plot Rule 7)
        if 'Impact' in name:
            x_pos = t_val - 0.12
            h_align = 'right'
        else:
            x_pos = t_val + 0.12
            h_align = 'left'

        if draw_labels:
            # Suppress Exp. End-point label in combined plots (tighter timeline)
            if not draw_end_label and 'End-point' in name:
                continue
            ax.text(x_pos, h_pos, name, rotation=90, va=v_align, ha=h_align, fontsize=8, fontweight='bold',
                    color=text_color, bbox=dict(facecolor='white', alpha=0.90, edgecolor=color, pad=1.5), zorder=6)

def get_timeline_limits(wp_events, arming_time, df_t, is_absolute=False):
    """Calculates the strict [exp_start - 1.0s, exp_end + 1.0s] display limits (General Plot Rule 3 / SSoT)."""
    esp_time = wp_events.get('WP2')   # experiment start point (refined forward movement)
    eep_time = wp_events.get('WP3')   # experiment end point (sweep end achievement)

    if is_absolute:
        t_min = df_t.min() if not df_t.empty else 0.0
        t_max = df_t.max() if not df_t.empty else 20.0
        crop_min = max(t_min, esp_time - 1.0) if esp_time is not None else t_min
        crop_max = min(t_max, eep_time + 1.0) if eep_time is not None else t_max
    else:
        t_max_rel = (df_t.max() - arming_time) if not df_t.empty else 20.0
        crop_min = max(0.0, (esp_time - arming_time) - 1.0) if esp_time is not None else 0.0
        crop_max = (eep_time - arming_time) + 1.0 if eep_time is not None else t_max_rel

    return crop_min, crop_max


def get_combined_timeline_limits(wp_events, arming_time, is_absolute=False):
    """Combined side-by-side crop: [WP2 refined - 0.5s, Column Center Passed + 2.5s].

    Tighter window than the standard plot for combined Rotating|Fixed comparisons —
    starts closer to sweep onset and ends just after the drone clears the column.
    """
    start = (wp_events.get('WP2 refined') or wp_events.get('WP2', 0.0))
    end   = (wp_events.get('Column Center Passed')
             or wp_events.get('Column Passed')
             or wp_events.get('Column Impact')
             or start + 10.0)
    t_min = start - 0.5
    t_max = end + 2.5
    if is_absolute:
        t_min += arming_time
        t_max += arming_time
    return t_min, t_max

def plot_velocity_profile(df_mocap, wp_events, arming_time, takeoff_time, disarming_time, events_log, ax=None, label="", flight_name=None, achieved_angle=None, mocap_rate=240.0, condition=None, output_path=None, show_plot=True, df_raw=None, is_raw=False):
    """Plots a 3-subplot consolidated physical kinetic profile (Velocity, Tangential Accel, MoCap Rate)."""
    if df_mocap.empty:
        print("[WARN] Empty MoCap DataFrame, skipping velocity profile plot.")
        return None

    if condition is None and flight_name:
        if "rotating_cage" in flight_name.lower():
            condition = "Rotating Cage"
        elif "fixed_cage" in flight_name.lower():
            condition = "Fixed Cage"

    df_plot = df_mocap[df_mocap['t'] >= arming_time].copy()
    t_rel = df_plot['t'] - arming_time

    # Calculate dynamic timeline limits and physical figure width (proportional scale)
    t_min_crop, t_max_crop = get_timeline_limits(wp_events, arming_time, df_plot['t'], is_absolute=False)
    duration = t_max_crop - t_min_crop
    fig_width = max(6.0, duration * 1.5)

    # Set up subplots with gridspec
    if ax is None:
        fig, (ax_vel, ax_acc, ax_rate) = plt.subplots(3, 1, figsize=(fig_width, 10), sharex=True, 
                                                      gridspec_kw={'height_ratios': [2.5, 2.5, 1.2]})
    else:
        # Fallback if ax is passed (compatibility with legacy single panel)
        ax_vel = ax
        ax_acc = None
        ax_rate = None
        show_plot = False
        
    is_raw_inferred = is_raw or (label and "raw" in label.lower()) or (condition and "raw" in condition.lower())

    # ---------------- 1. Top Panel: continuous velocity profile ----------------
    ax_vel.plot(t_rel, df_plot['speed'], color=C_MOCAP, linewidth=2.2, label=f'MoCap SG Velocity ({label})')
    
    # Highlight surgically repaired dropout segments
    if 'is_doctored' in df_plot.columns and df_plot['is_doctored'].any():
        # Dilate mask by 1 point to ensure the doctored segment connects perfectly to the clean data lines
        doctored_mask = df_plot['is_doctored'].values
        dilated_mask = doctored_mask.copy()
        dilated_mask[1:] |= doctored_mask[:-1]
        dilated_mask[:-1] |= doctored_mask[1:]
        
        doctored_speed = np.where(dilated_mask, df_plot['speed'], np.nan)
        ax_vel.plot(t_rel, doctored_speed, color='#10B981', linewidth=2.5,
                    label='Reconstructed Dropout Segment (Linear Fit)', zorder=4)

    # Segment average velocities
    has_avg_legend = False
    for idx in range(1, len(events_log)):
        evt_prev = events_log[idx-1]
        evt_curr = events_log[idx]
        t_prev_str = evt_prev['Time Since Arming (s)'].replace('s', '').replace('+', '')
        t_curr_str = evt_curr['Time Since Arming (s)'].replace('s', '').replace('+', '')
        if t_prev_str == "N/A" or t_curr_str == "N/A":
            continue
        try:
            t_p = float(t_prev_str)
            t_c = float(t_curr_str)
        except ValueError:
            continue
        avg_v_str = evt_curr['Average Velocity'].replace(' m/s', '')
        if avg_v_str == "N/A":
            continue
        try:
            avg_v = float(avg_v_str)
        except ValueError:
            continue
        lbl = 'Segment Avg Velocity' if not has_avg_legend else ''
        ax_vel.hlines(y=avg_v, xmin=t_p, xmax=t_c, colors='#2CA02C', linestyles='--', linewidth=2.0, label=lbl)
        has_avg_legend = True

    ax_vel.set_ylabel('Velocity (m/s)')
    ax_vel.set_ylim(-0.05, 0.85)
    ax_vel.set_yticks(np.arange(0, 0.9, 0.2))
    ax_vel.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#EAEAEA')
    ax_vel.grid(True)

    cond_suffix = f" <{condition}>" if condition else ""
    ax_vel.set_title(f'Flight Kinetic Profile: Filtered Velocity & Event Timeline{cond_suffix}', pad=15, fontweight='bold')

    # ---------------- 2. Middle Panel: continuous tangential acceleration ----------------
    if ax_acc is not None:
        # Fill positive (acceleration) and negative (deceleration) regions
        ax_acc.fill_between(t_rel, df_plot['accel'], 0,
                            where=(df_plot['accel'] >= 0),
                            color='#2CA02C', alpha=0.18, label='_nolegend_')
        ax_acc.fill_between(t_rel, df_plot['accel'], 0,
                            where=(df_plot['accel'] < 0),
                            color='#D62728', alpha=0.18, label='_nolegend_')

        # Main signal line in Crimson Red (#D62728) as per SSoT section 4
        ax_acc.plot(t_rel, df_plot['accel'], color='#D62728', linewidth=1.8,
                    label='Tangential Acceleration (m/s²)')

        # Highlight surgically repaired dropout segments in acceleration
        if 'is_doctored' in df_plot.columns and df_plot['is_doctored'].any():
            doctored_mask = df_plot['is_doctored'].values
            dilated_mask = doctored_mask.copy()
            dilated_mask[1:] |= doctored_mask[:-1]
            dilated_mask[:-1] |= doctored_mask[1:]
            
            doctored_accel = np.where(dilated_mask, df_plot['accel'], np.nan)
            ax_acc.plot(t_rel, doctored_accel, color='#10B981', linewidth=2.2,
                        label='Reconstructed Dropout Segment (Linear Fit)', zorder=4)

        # Bold zero reference line separating acceleration and deceleration states
        ax_acc.axhline(0.0, color='black', linewidth=1.6, linestyle='-', zorder=5,
                       label='Zero Acceleration (Steady State)')

        # Symmetric Y-limits strictly bounded to [-8.0, 8.0] m/s², tick intervals at 2.0 m/s²
        ax_acc.set_ylim(-8.0, 8.0)
        ax_acc.set_yticks(np.arange(-8.0, 8.1, 2.0))
        ax_acc.set_ylabel('Tangential Accel (m/s²)\n← Decel | Accel →')
        ax_acc.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#EAEAEA')
        ax_acc.grid(True)

        # Add filter explanation box at the right bottom
        is_high_jitter = df_plot['is_high_jitter'].iloc[0] if 'is_high_jitter' in df_plot.columns else False
        if is_high_jitter:
            filter_text = (
                "Filter Info:\n"
                "• Savitzky-Golay (w=19, p=3)\n"
                "• 4Hz Butterworth Low-Pass\n"
                "• Surgical Gap Repair (>100ms)"
            )
        else:
            filter_text = (
                "Filter Info:\n"
                "• Savitzky-Golay (w=19, p=3)\n"
                "• Surgical Gap Repair (>33ms)"
            )
        props = dict(boxstyle='round', facecolor='white', alpha=0.85, edgecolor='#EAEAEA')
        ax_acc.text(0.98, 0.08, filter_text, transform=ax_acc.transAxes, fontsize=8,
                    verticalalignment='bottom', horizontalalignment='right', bbox=props, zorder=6)

    # ---------------- 3. Bottom Panel: dynamic MoCap /poses rate ----------------
    if ax_rate is not None:
        df_rate_source = df_raw[df_raw['t'] >= arming_time].copy() if df_raw is not None else df_plot
        dt = df_rate_source['t'].diff()
        rate = 1.0 / dt
        rate = rate.clip(0, 450)
        
        # Color coding: Green for healthy, Red for dropouts (< 30Hz)
        t_rel_rate = df_rate_source['t'] - arming_time
        
        if is_raw_inferred:
            colors = np.where(rate < 30.0, '#D62728', '#2CA02C')
            ax_rate.scatter(t_rel_rate, rate, c=colors, s=12, alpha=0.8, label='/poses Publish Rate')
            ax_rate.plot(t_rel_rate, rate, color='#888888', linewidth=0.5, alpha=0.3)
        else:
            # Clean plot: no scatter dots, just a smooth solid line
            ax_rate.plot(t_rel_rate, rate, color='#2CA02C', linewidth=1.5, alpha=0.8, label='/poses Publish Rate')
        
        # Draw threshold lines
        ax_rate.axhline(30.0, color='#D62728', linestyle='--', linewidth=1.2, label='Min Failsafe Rate (30 Hz)')
        ax_rate.axhline(mocap_rate, color='#2CA02C', linestyle=':', linewidth=1.2, label=f'Nominal Motive Rate ({mocap_rate:.1f} Hz)')
        
        ax_rate.set_ylabel('Mocap Rate (Hz)')
        ax_rate.set_xlabel('Time Since Arming (seconds)')
        ax_rate.set_ylim(-10, 480)
        ax_rate.grid(True)
        ax_rate.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#EAEAEA', fontsize=8)

        # Print transmission diagnostics
        avg_rate = rate.dropna().mean()
        min_rate = rate.dropna().min()
        print(f"📊 MoCap Stream Transmission Diagnostics ({label}):")
        print(f"   - Nominal Streaming Rate:             {avg_rate:.1f} Hz")
        print(f"   - Minimum Observed Rate (Worst Jitter): {min_rate:.1f} Hz")
        total_dropouts = (rate < 30.0).sum()
        print(f"   - Critical Tracking Degradation Events: {total_dropouts} frames below 30Hz failsafe limit")

    # ---------------- Timeline Limits & Markers ----------------
    # Shared X axis limit crop window
    ax_vel.set_xlim(t_min_crop, t_max_crop)
    
    # Strictly enforce 1.0s uniform ticks
    if ax_rate is not None:
        ax_rate.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    else:
        ax_vel.xaxis.set_major_locator(ticker.MultipleLocator(1.0))

    # Draw timeline markers. Draw labels only on top plot to prevent overlapping text labels.
    draw_timeline_markers(ax_vel, wp_events, arming_time, (-0.05, 0.85), is_absolute=False, achieved_angle=achieved_angle, draw_labels=True)
    if ax_acc is not None:
        draw_timeline_markers(ax_acc, wp_events, arming_time, (-8.0, 8.0), is_absolute=False, achieved_angle=achieved_angle, draw_labels=False)
    if ax_rate is not None:
        draw_timeline_markers(ax_rate, wp_events, arming_time, (-10, 480), is_absolute=False, achieved_angle=achieved_angle, draw_labels=False)

    if flight_name:
        target_ax = ax_rate if ax_rate is not None else ax_vel
        fig.text(0.98, 0.01, f"{flight_name}", transform=fig.transFigure,
                 ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA', boxstyle='round,pad=0.2'))

    plt.tight_layout()
    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        plt.savefig(output_path, dpi=300)
        print(f"[INFO] Kinetic profile plot saved successfully to: {output_path}")
    if show_plot:
        plt.show()
    plt.close()

def plot_battery_sag(df_bat, takeoff_time, wp_events, arming_time, label="", flight_name=None, achieved_angle=None, output_path=None, show_plot=True):
    """Plots a double Y-axis LiPo dynamic battery depletion and voltage sag profile."""
    if df_bat is None or df_bat.empty:
        print("[WARN] Empty battery DataFrame, skipping battery plot.")
        return None

    df_plot = df_bat[df_bat['t'] >= arming_time].copy()
    if df_plot.empty:
        df_plot = df_bat.copy()
        t_rel = df_plot['t']
    else:
        t_rel = df_plot['t'] - arming_time

    t_min_crop, t_max_crop = get_timeline_limits(wp_events, arming_time, df_plot['t'], is_absolute=False)
    duration = t_max_crop - t_min_crop
    fig_width = max(6.0, duration * 1.5)

    fig, ax1 = plt.subplots(figsize=(fig_width, 5))

    # Plot Voltage on left axis
    ax1.plot(t_rel, df_plot['voltage'], color=C_BAT, linewidth=2.0, label='Voltage Under Load')
    ax1.axhline(21.6, color='orange', linestyle=':', label='Low Battery Alert (3.6V/cell)')
    ax1.axhline(20.4, color='red', linestyle='--', label='Critical Voltage Threshold (3.4V/cell)')
    
    ax1.set_xlabel('Flight Time (seconds since Arming)')
    ax1.set_ylabel('Battery Voltage (V)', color=C_BAT)
    ax1.tick_params(axis='y', labelcolor=C_BAT)
    ax1.set_ylim(19.0, 26.0)
    
    # Twin axis for capacity percentage on right axis
    ax2 = ax1.twinx()
    ax2.plot(t_rel, df_plot['remaining'], color='#A6761D', linestyle='-.', alpha=0.7, label='Capacity Remaining')
    ax2.set_ylabel('Remaining Capacity (%)', color='#A6761D')
    ax2.tick_params(axis='y', labelcolor='#A6761D')
    ax2.set_ylim(0, 105)
    
    # Combine legends
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='lower left')
    
    ax1.set_title(f'6S LiPo Dynamic Battery Depletion & Motor Load Sag Profile <{label}>', fontweight='bold')
    ax1.grid(True)

    # Draw standard timeline markers on Battery plot (relative time)
    draw_timeline_markers(ax1, wp_events, arming_time, (19.0, 26.0), is_absolute=False, achieved_angle=achieved_angle)

    if flight_name:
        fig.text(0.98, 0.01, f"{flight_name}", transform=fig.transFigure,
                 ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA', boxstyle='round,pad=0.2'))

    # Set timeline limits (General Plot Rule 3: [T_start - 1s, T_end + 1s])
    t_min_crop, t_max_crop = get_timeline_limits(wp_events, arming_time, df_plot['t'], is_absolute=False)
    ax1.set_xlim(t_min_crop, t_max_crop)

    plt.tight_layout()
    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        plt.savefig(output_path, dpi=300)
        print(f"[INFO] Battery sag plot saved successfully to: {output_path}")
    if show_plot:
        plt.show()
    plt.close()

    # Calculate metrics (uses full df_bat for correct diagnostics)
    active_flight_time_min = (df_bat['t'].max() - takeoff_time) / 60.0 if takeoff_time is not None else df_bat['t'].max() / 60.0
    init_voltage = df_bat['voltage'].iloc[0] if not df_bat.empty else 0.0
    min_voltage = df_bat['voltage'].min() if not df_bat.empty else 0.0
    sag_v = init_voltage - min_voltage
    capacity_depleted = (df_bat['remaining'].iloc[0] - df_bat['remaining'].iloc[-1]) if not df_bat.empty else 0.0
    depletion_rate_per_min = capacity_depleted / active_flight_time_min if active_flight_time_min > 0 else 0.0
    
    print(f"🔋 Battery Chemical Consumption Analysis ({label}):")
    print(f"   - Initial Unloaded Voltage:           {init_voltage:.2f} V")
    print(f"   - Minimum Voltage Sag (Takeoff/Loads): {min_voltage:.2f} V  (Total Drop = -{sag_v:.2f} V)")
    print(f"   - Active Flight Duration Recorded:     {active_flight_time_min*60.0:.1f} seconds")
    print(f"   - Total Capacity Consumption:          {capacity_depleted:.1f}%")
    print(f"   - Consumption Rate (load-adjusted):     {depletion_rate_per_min:.1f}% capacity per minute of flight")
    if depletion_rate_per_min > 0:
        print(f"   - Safe Flight Ceiling (40% Cutoff):    {(60.0 / depletion_rate_per_min) * 2.4:.1f} seconds")

def plot_imu_dynamics(df_imu, wp_events, arming_time, takeoff_time, disarming_time, events_log, label="", flight_name=None, achieved_angle=None, output_path=None, show_plot=True):
    """
    Plots the physical IMU collision dynamics overlaid with the flight mission waypoints.
    *** FROZEN PLOT: DO NOT MODIFY ***
    (This plot is considered complete and serves as a template for future aggregated visualizations.)
    """
    if df_imu is None or df_imu.empty:
        print("[WARN] Empty IMU DataFrame, skipping IMU plot.")
        return

    df_plot = df_imu[df_imu['t'] >= arming_time].copy()
    if df_plot.empty:
        df_plot = df_imu.copy()
        t_rel = df_plot['t']
    else:
        t_rel = df_plot['t'] - arming_time

    t_min_crop, t_max_crop = get_timeline_limits(wp_events, arming_time, df_plot['t'], is_absolute=False)
    duration = t_max_crop - t_min_crop
    fig_width = max(6.0, duration * 1.5)

    fig, ax1 = plt.subplots(figsize=(fig_width, 5))
    
    # Plot Acceleration Deviation (Impact Magnitude)
    ax1.plot(t_rel, df_plot['a_deviation'], color='#D62728', linewidth=1.5, label='Linear Accel Deviation (m/s²)')
    
    # Threshold for High-G Event
    ax1.axhline(5.0, color='orange', linestyle='--', linewidth=1.2, label='Severe Impact Threshold (5.0 m/s²)')
    
    ax1.set_xlabel('Time (seconds since Arming)')
    ax1.set_ylabel('Acceleration Deviation from Gravity (m/s²)', color='#D62728')
    ax1.tick_params(axis='y', labelcolor='#D62728')
    
    # Strictly bounded Y-axis limits of [-1.0, 20.0] m/s², tick intervals at 2.0 m/s²
    ax1.set_ylim(-1.0, 20.0)
    import matplotlib.ticker as ticker
    ax1.yaxis.set_major_locator(ticker.MultipleLocator(2.0))
    
    # Plot Gyro Surge on right axis
    ax2 = ax1.twinx()
    ax2.plot(t_rel, df_plot['g_mag'], color='#1F77B4', linestyle='-.', alpha=0.7, label='Gyro Rotational Surge (rad/s)')
    ax2.set_ylabel('Gyro Magnitude (rad/s)', color='#1F77B4')
    ax2.tick_params(axis='y', labelcolor='#1F77B4')
    ax2.set_ylim(0, 10)
    ax2.set_yticks(np.arange(0, 11, 1))
    
    # Combine legends - keep upper right but shift slightly left to prevent interference with final marker line
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right', bbox_to_anchor=(0.85, 0.98), frameon=True, facecolor='white', edgecolor='#EAEAEA')
    
    cond_str = label
    if cond_str and not cond_str.startswith('<'):
        cond_str = f"<{cond_str}>"
    ax1.set_title(f'IMU Collision Dynamic {cond_str}', fontsize=12, fontweight='bold')
    ax1.grid(True)

    # Draw standard timeline markers (relative time)
    draw_timeline_markers(ax1, wp_events, arming_time, (-1.0, 20.0), is_absolute=False, achieved_angle=achieved_angle)

    if flight_name:
        fig.text(0.98, 0.01, f"{flight_name}", transform=fig.transFigure,
                 ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA', boxstyle='round,pad=0.2'))

    # Set timeline limits (General Plot Rule 3: [T_start - 1s, T_end + 1s])
    t_min_crop, t_max_crop = get_timeline_limits(wp_events, arming_time, df_plot['t'], is_absolute=False)
    ax1.set_xlim(t_min_crop, t_max_crop)
    
    # X-axis ticks strictly set to 1.0s intervals
    ax1.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    
    plt.tight_layout()
    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        plt.savefig(output_path, dpi=300)
        print(f"[INFO] IMU dynamics plot saved successfully to: {output_path}")
    if show_plot:
        plt.show()
    plt.close()

def plot_imu_xyz_components(df_imu, wp_events, arming_time, takeoff_time, disarming_time, events_log, label="", flight_name=None, achieved_angle=None, output_path=None, show_plot=True):
    """
    Plots the raw X, Y, Z components of the IMU acceleration and gyroscope.
    Axis labels are professionally annotated to define the physical meaning on the Pixhawk 6C body frame.
    *** FROZEN PLOT: DO NOT MODIFY ***
    (This plot is considered beautiful and complete. Event labels have been moved to the bottom.)
    """
    if df_imu is None or df_imu.empty:
        print("[WARN] Empty IMU DataFrame, skipping IMU XYZ plot.")
        return

    df_plot = df_imu[df_imu['t'] >= arming_time].copy()
    if df_plot.empty:
        df_plot = df_imu.copy()
        t_rel = df_plot['t']
    else:
        t_rel = df_plot['t'] - arming_time

    t_min_crop, t_max_crop = get_timeline_limits(wp_events, arming_time, df_plot['t'], is_absolute=False)
    duration = t_max_crop - t_min_crop
    fig_width = max(6.0, duration * 1.5)

    # Proportional height ratios based on their exact physical spans:
    # X subplot: [-20.0, 6.0] -> span of 26
    # Y subplot: [-6.0, 20.0] -> span of 26
    # Z subplot: [-20.0, 0.0] -> span of 20
    fig, axs = plt.subplots(3, 1, figsize=(fig_width, 10), sharex=True, gridspec_kw={'height_ratios': [26, 26, 20]})
    
    # Classic RGB standard: Red = X, Green = Y, Blue = Z
    # Mappings from SSoT:
    # X = Lateral / Roll (side-to-side drift)
    # Y = Longitudinal / Pitch (main transit vector)
    # Z = Vertical / Yaw / Heave (altitude/rotation)
    components = [
        ('X-Axis (Lateral / Roll)', 'ax', 'gx', '#d62728', '#fc8d59', 'Roll Rate\n(rad/s)', (-20.0, 6.0)),  
        ('Y-Axis (Longitudinal / Pitch)', 'ay', 'gy', '#2ca02c', '#a6d96a', 'Pitch Rate\n(rad/s)', (-6.0, 20.0)),  
        ('Z-Axis (Vertical / Yaw/Heave)', 'az', 'gz', '#1f77b4', '#9ecae1', 'Yaw Rate\n(rad/s)', (-20.0, 0.0))   
    ]

    for i, (name, a_col, g_col, c_acc, c_gyr, gyro_lbl, bounds) in enumerate(components):
        ax = axs[i]
        
        # Acceleration
        line1 = ax.plot(t_rel, df_plot[a_col], color=c_acc, linewidth=1.5, label=f'Accel {name} (m/s²)')
        ax.set_ylabel(f'Accel {name.split()[0]} (m/s²)', color=c_acc)
        ax.tick_params(axis='y', labelcolor=c_acc)
        
        y_min, y_max = bounds
        ax.set_ylim(y_min, y_max)
        # Uniform, evenly spaced 2.0 m/s^2 Left Y-axis ticks across all subplots
        ax.set_yticks(np.arange(y_min, y_max + 0.1, 2.0))

        # Gyro
        ax_gyr = ax.twinx()
        line2 = ax_gyr.plot(t_rel, df_plot[g_col], color=c_gyr, linestyle='-.', alpha=0.8, label=f'Gyro {name} (rad/s)')
        ax_gyr.set_ylabel(gyro_lbl, color=c_gyr)
        ax_gyr.tick_params(axis='y', labelcolor=c_gyr)
        
        # Gyro Y-limits — fixed per axis for visual consistency across flights
        if i == 0:      # X-Axis (Lateral / Roll)
            ax_gyr.set_ylim(-1.5, 1.5)
            ax_gyr.set_yticks(np.arange(-1.5, 1.6, 0.5))
        elif i == 1:    # Y-Axis (Longitudinal / Pitch)
            ax_gyr.set_ylim(-1.5, 1.5)
            ax_gyr.set_yticks(np.arange(-1.5, 1.6, 0.5))
        else:           # Z-Axis (Vertical / Yaw/Heave)
            ax_gyr.set_ylim(-1.0, 5.0)
            ax_gyr.set_yticks(np.arange(-1.0, 5.1, 1.0))

        # Draw standard timeline markers (relative time). Draw labels only on top plot (index 0) to prevent text collision.
        draw_timeline_markers(ax, wp_events, arming_time, (y_min, y_max), is_absolute=False, achieved_angle=achieved_angle, draw_labels=(i == 0), label_pos='bottom')
        
        ax.grid(True)
        
        # Combine legends for the subplot
        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax.legend(lines, labels, loc='upper right', fontsize=8)

    axs[-1].set_xlabel('Time (seconds since Arming)')
    
    # Set timeline limits (General Plot Rule 3: [T_start - 1s, T_end + 1s])
    axs[-1].set_xlim(t_min_crop, t_max_crop)
    
    # Strictly enforce 1.0s uniform ticks
    axs[-1].xaxis.set_major_locator(ticker.MultipleLocator(1.0))

    if flight_name:
        fig.text(0.98, 0.01, f"{flight_name}", transform=fig.transFigure,
                 ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA', boxstyle='round,pad=0.2'))

    fig.suptitle(f'Raw IMU X/Y/Z Components <{label}>\n[X = Lateral/Roll, Y = Longitudinal/Pitch, Z = Vertical/Yaw]', fontsize=14, fontweight='bold', y=0.98)
    
    plt.tight_layout()
    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        plt.savefig(output_path, dpi=300)
        print(f"[INFO] IMU XYZ components plot saved successfully to: {output_path}")
    if show_plot:
        plt.show()
    plt.close()


# ═══════════════════════════════════════════════════════════════════════════════
#  EKF Kinetic Profile (2-panel: EKF velocity + tangential accel, no MoCap rate)
# ═══════════════════════════════════════════════════════════════════════════════

def plot_ekf_kinetic_profile(ekf_t, ekf_speed, ekf_rate, df_mocap, wp_events,
                              arming_time, flight_name="", condition="",
                              achieved_angle=None, output_path=None,
                              show_plot=True):
    """2-panel EKF-based kinetic profile: velocity + tangential accel.

    Parameters
    ----------
    ekf_t : np.ndarray
        Time array (seconds, aligned to arming=0 via ``ekf_t - arming_time``).
    ekf_speed : np.ndarray
        Speed magnitude array (m/s), same length as *ekf_t*.
    ekf_rate : float
        EKF update rate (Hz) — shown in the completion printout.
    df_mocap : pd.DataFrame
        MoCap DataFrame (used only for timeline limit computation).
    wp_events : dict
        Waypoint events dict (from :func:`find_waypoint_events`).
    arming_time : float
        System arming timestamp for relative time conversion.
    flight_name, condition : str
        Plot metadata.
    achieved_angle : float or None
        Impact angle for marker annotation.
    output_path : str or None
        Save to file if set.
    show_plot : bool
        Display interactively if True (default).
    """
    if len(ekf_t) < 2 or len(ekf_speed) < 2:
        print("[WARN] EKF data too short, skipping EKF kinetic profile.")
        return

    # ── Tangential acceleration via finite difference ──
    t_rel = ekf_t - arming_time
    accel = np.gradient(ekf_speed, ekf_t)

    # ── Timeline limits ──
    t_min_crop, t_max_crop = get_timeline_limits(
        wp_events, arming_time, df_mocap["t"], is_absolute=False
    )
    duration = t_max_crop - t_min_crop
    fig_width = max(6.0, duration * 1.5)

    fig, (ax_vel, ax_acc) = plt.subplots(
        2, 1, figsize=(fig_width, 8), sharex=True,
        gridspec_kw={"height_ratios": [2.5, 2.5]},
    )

    cond_suffix = f" <{condition}>" if condition else ""
    fig.suptitle(
        f"Flight Kinetic Profile: EKF Velocity & Tangential Accel{cond_suffix}",
        fontsize=13, fontweight="bold", y=0.98,
    )

    # ── Top panel: EKF velocity (fixed 0-0.8 m/s, ticks every 0.2) ──
    ax_vel.plot(t_rel, ekf_speed, color=C_MOCAP, linewidth=2.2,
                label="EKF Velocity (vehicle_odometry)")
    ax_vel.set_ylabel("Velocity (m/s)")
    ax_vel.set_ylim(-0.05, 0.85)
    ax_vel.set_yticks(np.arange(0, 0.9, 0.2))
    ax_vel.legend(loc="upper right", frameon=True, facecolor="white",
                  edgecolor="#EAEAEA")
    ax_vel.grid(True)

    # ── Bottom panel: tangential acceleration (fixed ±8 m/s², ticks every 2) ──
    ax_acc.fill_between(t_rel, accel, 0,
                        where=(accel >= 0),
                        color="#2CA02C", alpha=0.18, label="_nolegend_")
    ax_acc.fill_between(t_rel, accel, 0,
                        where=(accel < 0),
                        color="#D62728", alpha=0.18, label="_nolegend_")
    ax_acc.plot(t_rel, accel, color="#D62728", linewidth=1.8,
                label="Tangential Acceleration (m/s²)")
    ax_acc.axhline(0.0, color="black", linewidth=1.6, linestyle="-", zorder=5,
                   label="Zero Acceleration (Steady State)")
    ax_acc.set_ylim(-8.0, 8.0)
    ax_acc.set_yticks(np.arange(-8.0, 8.1, 2.0))
    ax_acc.set_ylabel("Tangential Accel (m/s²)\n← Decel | Accel →")
    ax_acc.legend(loc="upper right", frameon=True, facecolor="white",
                  edgecolor="#EAEAEA")
    ax_acc.set_xlabel("Time Since Arming (seconds)")
    ax_acc.grid(True)

    # ── Timeline markers + 1.0s ticks ──
    ax_vel.set_xlim(t_min_crop, t_max_crop)
    ax_vel.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    draw_timeline_markers(ax_vel, wp_events, arming_time,
                          (-0.05, 0.85), is_absolute=False,
                          achieved_angle=achieved_angle, draw_labels=True)
    draw_timeline_markers(ax_acc, wp_events, arming_time,
                          (-8.0, 8.0), is_absolute=False,
                          achieved_angle=achieved_angle, draw_labels=False)

    # Data origin label (below x-axis, figure-bottom)
    if flight_name:
        fig.text(0.98, 0.01, f"{flight_name}",
                 transform=fig.transFigure,
                 ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA',
                           boxstyle='round,pad=0.2'))

    plt.tight_layout()
    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        plt.savefig(output_path, dpi=300)
        print(f"[INFO] EKF kinetic profile saved to: {output_path}")
    if show_plot:
        plt.show()
    plt.close()
    print(f"✅ EKF kinetic profile — EKF rate ≈ {ekf_rate:.0f} Hz")


# ═══════════════════════════════════════════════════════════════════════════════
#  Combined side-by-side plots  (Rotating Cage | Fixed Cage)
#  Timeline: [WP2 refined − 0.5s, Column Center Passed + 2.0s]
# ═══════════════════════════════════════════════════════════════════════════════

def plot_ekf_kinetic_combined(rot_data, fix_data, output_path=None):
    """Side-by-side EKF kinetic profile: Rotating Cage | Fixed Cage.

    Timeline uses get_combined_timeline_limits (tighter than standard).
    Each column: EKF velocity (top) + tangential acceleration (bottom).
    """
    from dev_logs.analysis.database.flight_loader import compute_ekf_velocity

    ekf_rot = compute_ekf_velocity(rot_data)
    ekf_fix = compute_ekf_velocity(fix_data)
    if ekf_rot is None or ekf_fix is None:
        print("[WARN] Cannot plot combined EKF kinetic — missing EKF data.")
        return

    rot_arm = rot_data["arming_time"]
    fix_arm = fix_data["arming_time"]

    # Relative time arrays
    t_rot_rel = ekf_rot["t"] - rot_arm
    t_fix_rel = ekf_fix["t"] - fix_arm

    # Tangential acceleration
    acc_rot = np.gradient(ekf_rot["speed"], ekf_rot["t"])
    acc_fix = np.gradient(ekf_fix["speed"], ekf_fix["t"])

    # Combined timeline limits
    t_min_r, t_max_r = get_combined_timeline_limits(rot_data["wp_events"], rot_arm)
    t_min_f, t_max_f = get_combined_timeline_limits(fix_data["wp_events"], fix_arm)

    fig, axes = plt.subplots(2, 2, figsize=(16, 10), sharex='col', sharey='row')

    # ── Row 0: Speed ──
    for col_idx, (t_rel, spd, label) in enumerate([
        (t_rot_rel, ekf_rot["speed"], "Rotating Cage"),
        (t_fix_rel, ekf_fix["speed"], "Fixed Cage"),
    ]):
        ax = axes[0, col_idx]
        ax.plot(t_rel, spd, color=C_MOCAP, linewidth=2.2, label="EKF Velocity")
        # Y-axis label only on left column
        if col_idx == 0:
            ax.set_ylabel("Velocity (m/s)")
        else:
            ax.set_ylabel('')
            ax.tick_params(axis='y', labelleft=False)
        ax.set_ylim(-0.05, 0.85)
        ax.set_yticks(np.arange(0, 0.9, 0.2))
        ax.set_title(f"{label}", fontsize=12, fontweight="bold")
        # Legend only on left column (upper left corner)
        if col_idx == 0:
            ax.legend(loc="upper left", frameon=True, facecolor="white",
                      edgecolor="#EAEAEA")
        ax.grid(True)

    # ── Row 1: Tangential accel ──
    for col_idx, (t_rel, acc, label) in enumerate([
        (t_rot_rel, acc_rot, "Rotating Cage"),
        (t_fix_rel, acc_fix, "Fixed Cage"),
    ]):
        ax = axes[1, col_idx]
        ax.fill_between(t_rel, acc, 0, where=(acc >= 0),
                        color="#2CA02C", alpha=0.18, label="_nolegend_")
        ax.fill_between(t_rel, acc, 0, where=(acc < 0),
                        color="#D62728", alpha=0.18, label="_nolegend_")
        ax.plot(t_rel, acc, color="#D62728", linewidth=1.8,
                label="Tangential Acceleration (m/s²)")
        ax.axhline(0.0, color="black", linewidth=1.6, linestyle="-", zorder=5,
                   label="Zero Acceleration (Steady State)")
        ax.set_ylim(-8.0, 8.0)
        ax.set_yticks(np.arange(-8.0, 8.1, 2.0))
        # Y-axis label only on left column
        if col_idx == 0:
            ax.set_ylabel("Tangential Accel (m/s²)\n← Decel | Accel →")
        else:
            ax.set_ylabel('')
            ax.tick_params(axis='y', labelleft=False)
        # Legend only on left column (upper left corner)
        if col_idx == 0:
            ax.legend(loc="upper left", frameon=True, facecolor="white",
                      edgecolor="#EAEAEA")
        ax.set_xlabel("Time Since Arming (seconds)")
        ax.grid(True)

    # ── Timeline limits + markers (no Exp. End-point label in combined view) ──
    axes[0, 0].set_xlim(t_min_r, t_max_r)
    axes[0, 1].set_xlim(t_min_f, t_max_f)
    for ax in axes[0]:
        ax.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    draw_timeline_markers(axes[0, 0], rot_data["wp_events"], rot_arm,
                          (-0.05, 0.85), is_absolute=False,
                          achieved_angle=rot_data.get("achieved_angle"),
                          draw_labels=True, draw_end_label=False)
    draw_timeline_markers(axes[1, 0], rot_data["wp_events"], rot_arm,
                          (-8.0, 8.0), is_absolute=False,
                          achieved_angle=rot_data.get("achieved_angle"),
                          draw_labels=False)
    draw_timeline_markers(axes[0, 1], fix_data["wp_events"], fix_arm,
                          (-0.05, 0.85), is_absolute=False,
                          achieved_angle=fix_data.get("achieved_angle"),
                          draw_labels=True, draw_end_label=False)
    draw_timeline_markers(axes[1, 1], fix_data["wp_events"], fix_arm,
                          (-8.0, 8.0), is_absolute=False,
                          achieved_angle=fix_data.get("achieved_angle"),
                          draw_labels=False)

    fig.suptitle("EKF Kinetic Profile — Rotating Cage vs Fixed Cage",
                 fontsize=14, fontweight="bold", y=0.98)

    # Per-column source labels: left at left-bottom, right at right-bottom
    rn = rot_data.get("flight_name", "")
    fn = fix_data.get("flight_name", "")
    if rn:
        fig.text(0.02, 0.015, rn, transform=fig.transFigure,
                 ha='left', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA',
                           boxstyle='round,pad=0.2'))
    if fn:
        fig.text(0.98, 0.015, fn, transform=fig.transFigure,
                 ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA',
                           boxstyle='round,pad=0.2'))

    plt.tight_layout(rect=[0, 0.035, 1, 0.96])
    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        fig.savefig(output_path, dpi=300)
        print(f"[INFO] Combined EKF kinetic plot saved to: {output_path}")
    plt.show()
    print("✅ Combined EKF kinetic — Rotating | Fixed")


def plot_imu_dynamics_combined(rot_data, fix_data, output_path=None):
    """Side-by-side IMU collision dynamics: Rotating Cage | Fixed Cage.

    Timeline uses get_combined_timeline_limits.
    Each panel: accel deviation (left Y) + gyro magnitude (right Y).
    """
    rot_arm = rot_data["arming_time"]
    fix_arm = fix_data["arming_time"]

    df_rot = rot_data["df_imu"]
    df_fix = fix_data["df_imu"]

    def _prep(df, arm):
        if df is None or df.empty:
            return None, None
        df_p = df[df['t'] >= arm].copy()
        if df_p.empty:
            df_p = df.copy()
            t_rel = df_p['t']
        else:
            t_rel = df_p['t'] - arm
        return df_p, t_rel

    df_rot_p, t_rot_r = _prep(df_rot, rot_arm)
    df_fix_p, t_fix_r = _prep(df_fix, fix_arm)

    t_min_r, t_max_r = get_combined_timeline_limits(rot_data["wp_events"], rot_arm)
    t_min_f, t_max_f = get_combined_timeline_limits(fix_data["wp_events"], fix_arm)

    fig, (ax1_l, ax1_r) = plt.subplots(1, 2, figsize=(14, 7), sharey=True)

    def _draw_panel(ax, df_p, t_rel, wp_events, arm, label, angle, t_min, t_max, show_legend, is_left=True):
        if df_p is None:
            ax.text(0.5, 0.5, "No IMU data", transform=ax.transAxes,
                    ha='center', va='center', fontsize=12, alpha=0.5)
            return

        ax.plot(t_rel, df_p['a_deviation'], color='#D62728', linewidth=1.5,
                label='Linear Accel Deviation (m/s²)')
        ax.axhline(5.0, color='orange', linestyle='--', linewidth=1.2,
                   label='Severe Impact Threshold (5.0 m/s²)')
        # Accel Y-label only on left plot's left axis
        if is_left:
            ax.set_ylabel('Accel Deviation from Gravity (m/s²)', color='#D62728')
            ax.tick_params(axis='y', labelcolor='#D62728')
        else:
            ax.set_ylabel('')
            ax.tick_params(axis='y', labelleft=False)
        ax.set_ylim(-1.0, 20.0)
        ax.yaxis.set_major_locator(ticker.MultipleLocator(2.0))

        ax2 = ax.twinx()
        ax2.plot(t_rel, df_p['g_mag'], color='#1F77B4', linestyle='-.', alpha=0.7,
                 label='Gyro Rotational Surge (rad/s)')
        # Gyro Y-label only on right plot's right axis
        if is_left:
            ax2.set_ylabel('')
            ax2.tick_params(axis='y', labelright=False)
        else:
            ax2.set_ylabel('Gyro Magnitude (rad/s)', color='#1F77B4')
            ax2.tick_params(axis='y', labelcolor='#1F77B4')
        ax2.set_ylim(0, 10)
        ax2.set_yticks(np.arange(0, 11, 1))

        # Legend only on left panel (upper left corner)
        if show_legend:
            lines1, labels1 = ax.get_legend_handles_labels()
            lines2, labels2 = ax2.get_legend_handles_labels()
            ax.legend(lines1 + lines2, labels1 + labels2, loc='upper left',
                      frameon=True, facecolor='white',
                      edgecolor='#EAEAEA')

        cond_str = f"<{label}>"
        ax.set_title(f'IMU Collision Dynamic {cond_str}', fontsize=12, fontweight='bold')
        ax.grid(True)

        draw_timeline_markers(ax, wp_events, arm, (-1.0, 20.0),
                              is_absolute=False, achieved_angle=angle,
                              draw_end_label=False)
        ax.set_xlim(t_min, t_max)
        ax.xaxis.set_major_locator(ticker.MultipleLocator(1.0))

    _draw_panel(ax1_l, df_rot_p, t_rot_r, rot_data["wp_events"], rot_arm,
                "Rotating Cage", rot_data.get("achieved_angle"),
                t_min_r, t_max_r, show_legend=True, is_left=True)
    _draw_panel(ax1_r, df_fix_p, t_fix_r, fix_data["wp_events"], fix_arm,
                "Fixed Cage", fix_data.get("achieved_angle"),
                t_min_f, t_max_f, show_legend=False, is_left=False)

    fig.suptitle("IMU Collision Dynamics — Rotating Cage vs Fixed Cage",
                 fontsize=14, fontweight="bold", y=0.98)

    # Per-column source labels: left at left-bottom, right at right-bottom
    rn = rot_data.get("flight_name", "")
    fn = fix_data.get("flight_name", "")
    if rn:
        fig.text(0.02, 0.015, rn, transform=fig.transFigure,
                 ha='left', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA',
                           boxstyle='round,pad=0.2'))
    if fn:
        fig.text(0.98, 0.015, fn, transform=fig.transFigure,
                 ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA',
                           boxstyle='round,pad=0.2'))

    plt.tight_layout(rect=[0, 0.035, 1, 0.96])
    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        fig.savefig(output_path, dpi=300)
        print(f"[INFO] Combined IMU dynamics plot saved to: {output_path}")
    plt.show()
    print("✅ Combined IMU dynamics — Rotating | Fixed")


def plot_imu_xyz_combined(rot_data, fix_data, output_path=None):
    """Side-by-side IMU X/Y/Z components: Rotating Cage | Fixed Cage.

    Timeline uses get_combined_timeline_limits.
    Each column: accel + gyro per axis (X lateral/roll, Y longitudinal/pitch, Z vertical/yaw).
    """
    rot_arm = rot_data["arming_time"]
    fix_arm = fix_data["arming_time"]

    def _prep(df, arm):
        if df is None or df.empty:
            return None, None
        df_p = df[df['t'] >= arm].copy()
        if df_p.empty:
            df_p = df.copy()
            t_rel = df_p['t']
        else:
            t_rel = df_p['t'] - arm
        return df_p, t_rel

    df_rot_p, t_rot_r = _prep(rot_data["df_imu"], rot_arm)
    df_fix_p, t_fix_r = _prep(fix_data["df_imu"], fix_arm)

    t_min_r, t_max_r = get_combined_timeline_limits(rot_data["wp_events"], rot_arm)
    t_min_f, t_max_f = get_combined_timeline_limits(fix_data["wp_events"], fix_arm)

    fig, axes = plt.subplots(3, 2, figsize=(16, 12), sharex='col', sharey='row')

    components = [
        ('X-Axis (Lateral / Roll)',     'ax', 'gx', '#d62728', '#fc8d59',
         'Roll Rate\n(rad/s)', (-20.0, 6.0),
         (-1.5, 1.5, np.arange(-1.5, 1.6, 0.5))),
        ('Y-Axis (Longitudinal / Pitch)','ay', 'gy', '#2ca02c', '#a6d96a',
         'Pitch Rate\n(rad/s)', (-6.0, 20.0),
         (-1.5, 1.5, np.arange(-1.5, 1.6, 0.5))),
        ('Z-Axis (Vertical / Yaw/Heave)','az', 'gz', '#1f77b4', '#9ecae1',
         'Yaw Rate\n(rad/s)', (-20.0, 0.0),
         (-1.0, 5.0, np.arange(-1.0, 5.1, 1.0))),
    ]

    # Collect legend handles once from the left-column subplots
    legend_handles = []

    for col_idx, (df_p, t_rel, wp_events, arm, angle, t_min, t_max, label) in enumerate([
        (df_rot_p, t_rot_r, rot_data["wp_events"], rot_arm,
         rot_data.get("achieved_angle"), t_min_r, t_max_r, "Rotating Cage"),
        (df_fix_p, t_fix_r, fix_data["wp_events"], fix_arm,
         fix_data.get("achieved_angle"), t_min_f, t_max_f, "Fixed Cage"),
    ]):
        for i, (name, a_col, g_col, c_acc, c_gyr, gyro_lbl, bounds, gyr_lims) in enumerate(components):
            ax = axes[i, col_idx]

            if df_p is None:
                ax.text(0.5, 0.5, "No IMU data", transform=ax.transAxes,
                        ha='center', va='center', fontsize=12, alpha=0.5)
                continue

            y_min, y_max = bounds
            g_min, g_max, g_ticks = gyr_lims

            # Acceleration (left Y-axis — shown only on left column)
            acc_line = ax.plot(t_rel, df_p[a_col], color=c_acc, linewidth=1.5,
                    label=f'Accel {name} (m/s²)')
            if col_idx == 0:
                ax.set_ylabel(f'Accel {name.split()[0]} (m/s²)', color=c_acc)
                ax.tick_params(axis='y', labelcolor=c_acc)
            else:
                ax.set_ylabel('')
                ax.tick_params(axis='y', labelleft=False)
            ax.set_ylim(y_min, y_max)
            ax.set_yticks(np.arange(y_min, y_max + 0.1, 2.0))

            # Gyro (right Y-axis — shown only on right column)
            ax_gyr = ax.twinx()
            gyr_line = ax_gyr.plot(t_rel, df_p[g_col], color=c_gyr, linestyle='-.', alpha=0.8,
                        label=f'Gyro {name} (rad/s)')
            if col_idx == 1:
                ax_gyr.set_ylabel(gyro_lbl, color=c_gyr)
                ax_gyr.tick_params(axis='y', labelcolor=c_gyr)
            else:
                ax_gyr.set_ylabel('')
                ax_gyr.tick_params(axis='y', labelright=False)
            ax_gyr.set_ylim(g_min, g_max)
            ax_gyr.set_yticks(g_ticks)

            # Timeline markers (labels only on top row; no Exp. End-point label)
            draw_timeline_markers(ax, wp_events, arm, (y_min, y_max),
                                  is_absolute=False, achieved_angle=angle,
                                  draw_labels=(i == 0), label_pos='bottom',
                                  draw_end_label=False)
            ax.grid(True)

            # Collect handles from left column only (one legend for all)
            if col_idx == 0:
                legend_handles.extend(acc_line + gyr_line)

    # Set timeline limits and 1s ticks
    for col_idx, (t_min, t_max) in enumerate([(t_min_r, t_max_r), (t_min_f, t_max_f)]):
        axes[-1, col_idx].set_xlim(t_min, t_max)
        axes[-1, col_idx].xaxis.set_major_locator(ticker.MultipleLocator(1.0))
        axes[-1, col_idx].set_xlabel("Time (seconds since Arming)")

    # Column titles
    axes[0, 0].set_title("Rotating Cage", fontsize=13, fontweight="bold", color="#006600")
    axes[0, 1].set_title("Fixed Cage", fontsize=13, fontweight="bold", color="#cc3300")

    fig.suptitle('Raw IMU X/Y/Z Components — Rotating Cage vs Fixed Cage\n'
                 '[X = Lateral/Roll, Y = Longitudinal/Pitch, Z = Vertical/Yaw]',
                 fontsize=14, fontweight='bold', y=0.94)

    # Single stacked legend in right upper corner — above the subplots
    if legend_handles:
        fig.legend(handles=legend_handles, loc='upper right',
                   bbox_to_anchor=(0.98, 0.965), ncol=1, fontsize=8,
                   frameon=True, facecolor='white', edgecolor='#EAEAEA')

    # Per-column source labels: left at left-bottom, right at right-bottom
    rn = rot_data.get("flight_name", "")
    fn = fix_data.get("flight_name", "")
    if rn:
        fig.text(0.02, 0.015, rn, transform=fig.transFigure,
                 ha='left', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA',
                           boxstyle='round,pad=0.2'))
    if fn:
        fig.text(0.98, 0.015, fn, transform=fig.transFigure,
                 ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA',
                           boxstyle='round,pad=0.2'))

    plt.tight_layout(rect=[0, 0.035, 1, 0.925])
    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        fig.savefig(output_path, dpi=300)
        print(f"[INFO] Combined IMU XYZ plot saved to: {output_path}")
    plt.show()
    print("✅ Combined IMU XYZ — Rotating | Fixed")

