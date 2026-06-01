import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

C_MOCAP = '#1F77B4'      # Steel Blue (Actual Ground Truth)
C_CMD = '#D62728'        # Crimson Red (Commanded Setpoint)
C_BAT = '#9467BD'        # Muted Purple (Battery Voltage)

def draw_timeline_markers(ax, wp_events, arming_time, y_lims, is_absolute=False, achieved_angle=None, draw_labels=True):
    """Draws perfectly aligned, standard timeline event markers on the given axis (General Plot Rule 6)."""
    y_min, y_max = y_lims
    y_pos = y_min + (y_max - y_min) * 0.88
    
    t_start = wp_events.get('WP1')
    t_impact = wp_events.get('Column Impact')
    t_end = wp_events.get('WP4')
    t_passed = wp_events.get('Column Center Passed') or wp_events.get('Column Passed')
    
    # Draw Shared Impact Window Shading (General Plot Rule 7 / SSoT)
    if t_impact is not None:
        t_rel_impact = t_impact if is_absolute else (t_impact - arming_time)
        ax.axvspan(t_rel_impact - 0.05, t_rel_impact + 0.35, color='#D62728', alpha=0.10, zorder=3)
        
    events = []
    if t_start is not None:
        events.append(('Exp. Start-point', t_start, '#9467BD', ':', 1.8, '#444444'))
    if t_passed is not None:
        events.append(('Column Center Passed', t_passed, '#FF9900', '--', 1.8, '#995C00'))
    if t_impact is not None:
        angle_str = f" ({achieved_angle:.1f}°)" if achieved_angle is not None else ""
        events.append((f'Impact{angle_str}', t_impact, '#D62728', '-.', 2.5, '#7F0000'))
    if t_end is not None:
        events.append(('Exp. End-point', t_end, '#9467BD', ':', 1.8, '#444444'))
        
    for idx, (name, t_abs, color, style, width, text_color) in enumerate(events):
        t_val = t_abs if is_absolute else (t_abs - arming_time)
        ax.axvline(x=t_val, color=color, linestyle=style, linewidth=width, alpha=0.95, zorder=5)
        # Prevents label text collision by alternating offsets and height positions
        h_fraction = 0.88 - 0.20 * (idx % 3)
        h_pos = y_min + (y_max - y_min) * h_fraction
        
        # Left-align the label for Impact to prevent obscuring post-impact curves (General Plot Rule 7)
        if 'Impact' in name:
            x_pos = t_val - 0.12
            h_align = 'right'
        else:
            x_pos = t_val + 0.12
            h_align = 'left'
            
        if draw_labels:
            ax.text(x_pos, h_pos, name, rotation=90, va='top', ha=h_align, fontsize=8, fontweight='bold',
                    color=text_color, bbox=dict(facecolor='white', alpha=0.90, edgecolor=color, pad=1.5), zorder=6)

def get_timeline_limits(wp_events, arming_time, df_t, is_absolute=False):
    """Calculates the strict [T_start - 1.0s, T_end + 1.0s] display limits (General Plot Rule 3 / SSoT)."""
    t_start = wp_events.get('WP1')
    t_end = wp_events.get('WP4')
    
    if is_absolute:
        t_min = df_t.min() if not df_t.empty else 0.0
        t_max = df_t.max() if not df_t.empty else 20.0
        crop_min = max(t_min, t_start - 1.0) if t_start is not None else t_min
        crop_max = min(t_max, t_end + 1.0) if t_end is not None else t_max
    else:
        t_max_rel = (df_t.max() - arming_time) if not df_t.empty else 20.0
        crop_min = max(0.0, (t_start - arming_time) - 1.0) if t_start is not None else 0.0
        crop_max = (t_end - arming_time) + 1.0 if t_end is not None else t_max_rel
        
    return crop_min, crop_max

def plot_velocity_profile(df_mocap, wp_events, arming_time, takeoff_time, disarming_time, events_log, ax=None, label="", flight_name=None, achieved_angle=None, mocap_rate=240.0, condition=None, output_path=None, show_plot=True):
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

    # ---------------- 1. Top Panel: continuous velocity profile ----------------
    ax_vel.plot(t_rel, df_plot['speed'], color=C_MOCAP, linewidth=2.2, label=f'MoCap SG Velocity ({label})')

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

    y_max_vel = df_plot['speed'].max() if not df_plot.empty else 1.0
    if np.isnan(y_max_vel) or y_max_vel <= 0:
        y_max_vel = 1.0

    ax_vel.set_ylabel('Velocity (m/s)')
    ax_vel.set_ylim(-0.05, y_max_vel * 1.15)
    ax_vel.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#EAEAEA')
    ax_vel.grid(True)

    cond_suffix = f" <{condition}>" if condition else ""
    ax_vel.set_title(f'⚡ Thesis Flight Kinetic Profile: SG Velocity & Event Timeline{cond_suffix}', pad=15)

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

        # Bold zero reference line separating acceleration and deceleration states
        ax_acc.axhline(0.0, color='black', linewidth=1.6, linestyle='-', zorder=5,
                       label='Zero Acceleration (Steady State)')

        # Symmetric Y-limits strictly bounded to [-12.0, 12.0] m/s², tick intervals at 4.0 m/s²
        ax_acc.set_ylim(-12.0, 12.0)
        ax_acc.set_yticks(np.arange(-12.0, 12.1, 4.0))
        ax_acc.set_ylabel('Tangential Accel (m/s²)\n← Decel | Accel →')
        ax_acc.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#EAEAEA')
        ax_acc.grid(True)

    # ---------------- 3. Bottom Panel: dynamic MoCap /poses rate ----------------
    if ax_rate is not None:
        dt = df_plot['t'].diff()
        rate = 1.0 / dt
        rate = rate.clip(0, 450)
        
        # Color coding: Green for healthy, Red for dropouts (< 30Hz)
        colors = np.where(rate < 30.0, '#D62728', '#2CA02C')
        
        ax_rate.scatter(t_rel, rate, c=colors, s=12, alpha=0.8, label='/poses Publish Rate')
        ax_rate.plot(t_rel, rate, color='#888888', linewidth=0.5, alpha=0.3)
        
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
    draw_timeline_markers(ax_vel, wp_events, arming_time, (-0.05, y_max_vel * 1.15), is_absolute=False, achieved_angle=achieved_angle, draw_labels=True)
    if ax_acc is not None:
        draw_timeline_markers(ax_acc, wp_events, arming_time, (-12.0, 12.0), is_absolute=False, achieved_angle=achieved_angle, draw_labels=False)
    if ax_rate is not None:
        draw_timeline_markers(ax_rate, wp_events, arming_time, (-10, 480), is_absolute=False, achieved_angle=achieved_angle, draw_labels=False)

    if flight_name:
        target_ax = ax_rate if ax_rate is not None else ax_vel
        target_ax.text(0.98, 0.02, f"{flight_name}", transform=target_ax.transAxes,
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
    
    ax1.set_title(f'6S LiPo Dynamic Battery Depletion & Motor Load Sag Profile ({label})')
    ax1.grid(True)

    # Draw standard timeline markers on Battery plot (relative time)
    draw_timeline_markers(ax1, wp_events, arming_time, (19.0, 26.0), is_absolute=False, achieved_angle=achieved_angle)

    if flight_name:
        ax1.text(0.98, 0.02, f"{flight_name}", transform=ax1.transAxes,
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
    """Plots the physical IMU collision dynamics overlaid with the flight mission waypoints."""
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
    ax2.set_ylim(0, max(10.0, df_plot['g_mag'].max() * 1.1) if not df_plot.empty else 10.0)
    
    # Combine legends - keep upper right but shift slightly left to prevent interference with final marker line
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right', bbox_to_anchor=(0.85, 0.98), frameon=True, facecolor='white', edgecolor='#EAEAEA')
    
    cond_str = label
    if cond_str and not cond_str.startswith('<'):
        cond_str = f"<{cond_str}>"
    ax1.set_title(f'IMU Collision Dynamic {cond_str}', fontsize=12)
    ax1.grid(True)

    # Draw standard timeline markers (relative time)
    draw_timeline_markers(ax1, wp_events, arming_time, (-1.0, 20.0), is_absolute=False, achieved_angle=achieved_angle)

    if flight_name:
        ax1.text(0.98, 0.02, f"{flight_name}", transform=ax1.transAxes,
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
    """Plots the raw X, Y, Z components of the IMU acceleration and gyroscope.
    Axis labels are professionally annotated to define the physical meaning on the Pixhawk 6C body frame.
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
        
        ax_gyr.set_ylim(df_plot[g_col].min() - 0.5, df_plot[g_col].max() + 0.5)

        # Draw standard timeline markers (relative time). Draw labels only on top plot (index 0) to prevent text collision.
        draw_timeline_markers(ax, wp_events, arming_time, (y_min, y_max), is_absolute=False, achieved_angle=achieved_angle, draw_labels=(i == 0))
        
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
        axs[-1].text(0.98, 0.02, f"{flight_name}", transform=axs[-1].transAxes,
                     ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                     bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA', boxstyle='round,pad=0.2'))
    
    fig.suptitle(f'Raw IMU X/Y/Z Components ({label})\n[X = Lateral/Roll, Y = Longitudinal/Pitch, Z = Vertical/Yaw]', fontsize=14, y=0.98)
    
    plt.tight_layout()
    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        plt.savefig(output_path, dpi=300)
        print(f"[INFO] IMU XYZ components plot saved successfully to: {output_path}")
    if show_plot:
        plt.show()
    plt.close()

def plot_tangential_accel(df_mocap, wp_events, arming_time, takeoff_time, disarming_time, events_log, label="", flight_name=None, achieved_angle=None, output_path=None):
    """Plots the Savitzky-Golay tangential acceleration profile over time.
    Positive values = acceleration, negative = deceleration.
    A bold zero line marks the boundary between the two states.
    """
    if df_mocap.empty or 'accel' not in df_mocap.columns:
        print("[WARN] No tangential acceleration data available. Run compute_velocity() first.")
        return

    df_plot = df_mocap[df_mocap['t'] >= arming_time].copy()
    
    t_min_crop, t_max_crop = get_timeline_limits(wp_events, arming_time, df_plot['t'], is_absolute=False)
    duration = t_max_crop - t_min_crop
    fig_width = max(6.0, duration * 1.5)

    fig, ax = plt.subplots(figsize=(fig_width, 5))
    t_rel = df_plot['t'] - arming_time

    # Fill positive (acceleration) and negative (deceleration) regions
    ax.fill_between(t_rel, df_plot['accel'], 0,
                    where=(df_plot['accel'] >= 0),
                    color='#2CA02C', alpha=0.18, label='_nolegend_')
    ax.fill_between(t_rel, df_plot['accel'], 0,
                    where=(df_plot['accel'] < 0),
                    color='#D62728', alpha=0.18, label='_nolegend_')

    # Main signal line
    ax.plot(t_rel, df_plot['accel'], color='#1F77B4', linewidth=1.8,
            label='Tangential Acceleration (m/s²)')

    # Bold zero reference line
    ax.axhline(0.0, color='black', linewidth=1.6, linestyle='-', zorder=5,
               label='Zero Acceleration (Steady State)')

    # Symmetric Y-axis — auto-scale to a clean range, minimum ±1.5 m/s²
    a_abs_max = max(1.5, df_plot['accel'].abs().quantile(0.99) * 1.2)
    ax.set_ylim(-a_abs_max, a_abs_max)

    # Draw standard timeline markers
    draw_timeline_markers(ax, wp_events, arming_time, (-a_abs_max, a_abs_max), is_absolute=False, achieved_angle=achieved_angle)

    ax.set_xlabel('Time Since Arming (seconds)')
    ax.set_ylabel('Tangential Acceleration (m/s²)\n← Deceleration | Acceleration →')
    ax.set_title(f'⚡ Tangential Acceleration / Deceleration Profile ({label})')
    
    # Set timeline limits (General Plot Rule 3: [T_start - 1s, T_end + 1s])
    ax.set_xlim(t_min_crop, t_max_crop)
    
    # Strictly enforce 1.0s uniform ticks
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    
    ax.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#EAEAEA')
    ax.grid(True, alpha=0.5)

    if flight_name:
        ax.text(0.98, 0.02, f"{flight_name}", transform=ax.transAxes,
                ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA', boxstyle='round,pad=0.2'))

    plt.tight_layout()
    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        plt.savefig(output_path, dpi=300)
        print(f"[INFO] Tangential acceleration plot saved successfully to: {output_path}")
    plt.show()
    plt.close()
