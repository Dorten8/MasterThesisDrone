import numpy as np
import matplotlib.pyplot as plt

C_MOCAP = '#1F77B4'      # Steel Blue (Actual Ground Truth)
C_CMD = '#D62728'        # Crimson Red (Commanded Setpoint)
C_BAT = '#9467BD'        # Muted Purple (Battery Voltage)

def plot_velocity_profile(df_mocap, wp_events, arming_time, takeoff_time, disarming_time, events_log, ax=None, label="", flight_name=None):
    """Plots a continuous Savitzky-Golay velocity profile with chronological event markers
    and horizontal segment average speed indicators, and a dual-subplot for the /poses publishing rate.
    """
    if df_mocap.empty:
        print("[WARN] Empty MoCap DataFrame, skipping velocity profile plot.")
        return None

    if ax is None:
        fig, (ax_vel, ax_rate) = plt.subplots(2, 1, figsize=(12, 8), sharex=True, gridspec_kw={'height_ratios': [3, 1]})
        show_plot = True
    else:
        ax_vel = ax
        ax_rate = None
        show_plot = False

    df_plot = df_mocap[df_mocap['t'] >= arming_time].copy()
    t_rel = df_plot['t'] - arming_time

    # Plot Savitzky-Golay velocity
    ax_vel.plot(t_rel, df_plot['speed'], color=C_MOCAP, linewidth=2.2, label=f'MoCap SG Velocity ({label})')

    # Plot horizontal average velocities per segment
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

    # Draw vertical event lines
    plot_events = []
    if arming_time is not None:
        plot_events.append(("Armed", 0.0))
    if takeoff_time is not None:
        plot_events.append(("Takeoff", takeoff_time - arming_time))
    
    # Sort chronologically by timestamp
    for wp in sorted(wp_events.keys(), key=lambda k: wp_events[k]):
        plot_events.append((wp, wp_events[wp] - arming_time))
        
    if disarming_time is not None:
        plot_events.append(("Landed", disarming_time - arming_time))

    y_max = df_plot['speed'].max() if not df_plot.empty else 1.0
    if np.isnan(y_max) or y_max <= 0:
        y_max = 1.0

    for idx, (evt_name, t_evt) in enumerate(plot_events):
        evt_label = evt_name
        if evt_name == "Column Passed":
            line_color = '#2CA02C'
            line_style = '--'
            line_width = 2.2
            alpha = 0.95
            text_color = '#1E5E1E'
            edge_color = '#2CA02C'
        elif evt_name == "Column Impact":
            line_color = '#D62728'
            line_style = '-.'
            line_width = 2.5
            alpha = 1.0
            text_color = '#7F0000'
            edge_color = '#D62728'
            if events_log:
                for item in events_log:
                    if "Column Impact" in item.get('Event Name', ''):
                        evt_label = item['Event Name'].replace("💥 ", "")
                        break
        else:
            line_color = '#9467BD'
            line_style = ':'
            line_width = 1.5
            alpha = 0.8
            text_color = '#444444'
            edge_color = '#DDDDDD'

        ax_vel.axvline(x=t_evt, color=line_color, linestyle=line_style, alpha=alpha, linewidth=line_width)
        if ax_rate is not None:
            ax_rate.axvline(x=t_evt, color=line_color, linestyle=line_style, alpha=alpha, linewidth=line_width)
        
        # Prevent visual overlap by varying label heights
        y_pos = y_max * (0.80 - 0.15 * (idx % 3))
        ax_vel.text(t_evt + 0.4, y_pos, evt_label, rotation=0, fontsize=9, fontweight='bold', color=text_color,
                 bbox=dict(facecolor='white', alpha=0.9, edgecolor=edge_color, boxstyle='round,pad=0.2'))

    ax_vel.set_title(f'⚡ Thesis Flight Kinetic Profile: SG Velocity & Event Timeline ({label})', pad=15)
    ax_vel.set_ylabel('Velocity Magnitude (m/s)')
    ax_vel.set_ylim(-0.05, y_max * 1.15)
    ax_vel.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#EAEAEA')
    ax_vel.grid(True)

    if flight_name:
        ax_vel.text(0.98, 0.02, f"{flight_name}", transform=ax_vel.transAxes,
                    ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                    bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA', boxstyle='round,pad=0.2'))

    # Plot /poses rate on the second subplot
    if ax_rate is not None:
        # Calculate instantaneous rate (1 / dt)
        dt = df_plot['t'].diff()
        rate = 1.0 / dt
        # Clip rates to reasonable limits (e.g. 0 to 450Hz) to prevent infinite values or spikes from scaling the plot
        rate = rate.clip(0, 450)
        
        # Color coding: Green for healthy, Red for dropouts (< 30Hz)
        colors = np.where(rate < 30.0, '#D62728', '#2CA02C')
        
        ax_rate.scatter(t_rel, rate, c=colors, s=12, alpha=0.8, label='/poses Publish Rate')
        ax_rate.plot(t_rel, rate, color='#888888', linewidth=0.5, alpha=0.3)
        
        # Draw threshold lines
        ax_rate.axhline(30.0, color='#D62728', linestyle='--', linewidth=1.2, label='Min Failsafe Rate (30 Hz)')
        ax_rate.axhline(240.0, color='#2CA02C', linestyle=':', linewidth=1.2, label='Nominal Motive Rate (240 Hz)')
        
        ax_rate.set_ylabel('Mocap Rate (Hz)')
        ax_rate.set_xlabel('Time Since Arming (seconds)')
        ax_rate.set_ylim(-10, 480)
        ax_rate.grid(True)
        ax_rate.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#EAEAEA', fontsize=8)
        
        # Calculate and print stats
        avg_rate = rate.dropna().mean()
        min_rate = rate.dropna().min()
        print(f"📊 MoCap Stream Transmission Diagnostics ({label}):")
        print(f"   - Nominal Streaming Rate:             {avg_rate:.1f} Hz")
        print(f"   - Minimum Observed Rate (Worst Jitter): {min_rate:.1f} Hz")
        total_dropouts = (rate < 30.0).sum()
        print(f"   - Critical Tracking Degradation Events: {total_dropouts} frames below 30Hz failsafe limit")

    ax_vel.set_xlim(0, t_rel.max() if not t_rel.empty else 10)

    if show_plot:
        plt.tight_layout()
        plt.show()

def plot_battery_sag(df_bat, takeoff_time, ax=None, label="", flight_name=None):
    """Plots a double Y-axis LiPo dynamic battery depletion and voltage sag profile."""
    if df_bat is None or df_bat.empty:
        print("[WARN] Empty battery DataFrame, skipping battery plot.")
        return None

    if ax is None:
        fig, ax1 = plt.subplots(figsize=(12, 5))
        show_plot = True
    else:
        ax1 = ax
        show_plot = False

    # Plot Voltage on left axis
    ax1.plot(df_bat['t'], df_bat['voltage'], color=C_BAT, linewidth=2.0, label='Voltage Under Load')
    ax1.axhline(21.6, color='orange', linestyle=':', label='Low Battery Alert (3.6V/cell)')
    ax1.axhline(20.4, color='red', linestyle='--', label='Critical Voltage Threshold (3.4V/cell)')
    
    ax1.set_xlabel('Flight Time (seconds)')
    ax1.set_ylabel('Battery Voltage (V)', color=C_BAT)
    ax1.tick_params(axis='y', labelcolor=C_BAT)
    ax1.set_ylim(19.0, 26.0)
    
    # Twin axis for capacity percentage on right axis
    ax2 = ax1.twinx()
    ax2.plot(df_bat['t'], df_bat['remaining'], color='#A6761D', linestyle='-.', alpha=0.7, label='Capacity Remaining')
    ax2.set_ylabel('Remaining Capacity (%)', color='#A6761D')
    ax2.tick_params(axis='y', labelcolor='#A6761D')
    ax2.set_ylim(0, 105)
    
    # Combine legends
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='lower left')
    
    ax1.set_title(f'🔋 6S LiPo Dynamic Battery Depletion & Motor Load Sag Profile ({label})')
    ax1.grid(True)

    if flight_name:
        ax1.text(0.98, 0.02, f"{flight_name}", transform=ax1.transAxes,
                 ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA', boxstyle='round,pad=0.2'))
    
    t_start = max(0, takeoff_time - 3.0) if takeoff_time is not None else 0.0
    ax1.set_xlim(t_start, df_bat['t'].max() if not df_bat.empty else 10.0)

    if show_plot:
        plt.tight_layout()
        plt.show()

    # Calculate metrics
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

def plot_imu_dynamics(df_imu, wp_events, arming_time, takeoff_time, disarming_time, events_log, label="", flight_name=None):
    """Plots the physical IMU collision dynamics overlaid with the flight mission waypoints."""
    if df_imu is None or df_imu.empty:
        print("[WARN] Empty IMU DataFrame, skipping IMU plot.")
        return

    fig, ax1 = plt.subplots(figsize=(12, 5))
    
    # Plot Acceleration Deviation (Impact Magnitude)
    ax1.plot(df_imu['t'], df_imu['a_deviation'], color='#D62728', linewidth=1.5, label='Linear Accel Deviation (m/s²)')
    
    # Threshold for High-G Event
    ax1.axhline(5.0, color='orange', linestyle='--', linewidth=1.2, label='Severe Impact Threshold (5.0 m/s²)')
    
    ax1.set_xlabel('Time Since Arming (seconds)')
    ax1.set_ylabel('Acceleration Deviation from Gravity (m/s²)', color='#D62728')
    ax1.tick_params(axis='y', labelcolor='#D62728')
    ax1.set_ylim(-1.0, max(15.0, df_imu['a_deviation'].max() * 1.1) if not df_imu.empty else 15.0)
    
    # Plot Gyro Surge on right axis
    ax2 = ax1.twinx()
    ax2.plot(df_imu['t'], df_imu['g_mag'], color='#1F77B4', linestyle='-.', alpha=0.7, label='Gyro Rotational Surge (rad/s)')
    ax2.set_ylabel('Gyro Magnitude (rad/s)', color='#1F77B4')
    ax2.tick_params(axis='y', labelcolor='#1F77B4')
    ax2.set_ylim(0, max(10.0, df_imu['g_mag'].max() * 1.1) if not df_imu.empty else 10.0)
    
    # Plot Waypoints (exact same as kinematics to perfectly align timeframes)
    t_min = takeoff_time - 2.0 if takeoff_time is not None else 0.0
    t_max = disarming_time + 5.0 if disarming_time is not None else df_imu['t'].max()

    y_pos_text = ax1.get_ylim()[1] * 0.9
    for name in sorted(wp_events.keys(), key=lambda k: wp_events[k]):
        t = wp_events[name]
        if t_min <= t <= t_max:
            if name == "Column Passed":
                ax1.axvline(x=t, color='#2CA02C', linestyle='--', alpha=0.9, linewidth=2.2)
                ax1.text(t + 0.1, y_pos_text, name, rotation=90, va='top', fontsize=9, fontweight='bold',
                         color='#1E5E1E', bbox=dict(facecolor='white', alpha=0.9, edgecolor='#2CA02C', pad=1.5))
            elif name == "Column Impact":
                evt_label = name
                if events_log:
                    for item in events_log:
                        if "Column Impact" in item.get('Event Name', ''):
                            evt_label = item['Event Name'].replace("💥 ", "")
                            break
                ax1.axvline(x=t, color='#D62728', linestyle='-.', alpha=1.0, linewidth=2.5)
                ax1.text(t + 0.1, y_pos_text * 0.95, evt_label, rotation=90, va='top', fontsize=9, fontweight='bold',
                         color='#7F0000', bbox=dict(facecolor='white', alpha=0.9, edgecolor='#D62728', pad=1.5))
            else:
                ax1.axvline(x=t, color='black', linestyle=':', alpha=0.5, linewidth=1.5)
                ax1.text(t + 0.1, y_pos_text, name, rotation=90, va='top', fontsize=9, 
                         bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=1))
            
    # Combine legends
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right', frameon=True, facecolor='white', edgecolor='#EAEAEA')
    
    ax1.set_title(f'💥 Physical IMU Collision Dynamics ({label})')
    ax1.grid(True)

    if flight_name:
        ax1.text(0.98, 0.02, f"{flight_name}", transform=ax1.transAxes,
                 ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA', boxstyle='round,pad=0.2'))
    ax1.set_xlim(t_min, t_max)
    
    plt.tight_layout()
    plt.show()

def plot_imu_xyz_components(df_imu, wp_events, arming_time, takeoff_time, disarming_time, events_log, label="", flight_name=None):
    """Plots the raw X, Y, Z components of the IMU acceleration and gyroscope."""
    if df_imu is None or df_imu.empty:
        print("[WARN] Empty IMU DataFrame, skipping IMU XYZ plot.")
        return

    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    t_min = takeoff_time - 2.0 if takeoff_time is not None else 0.0
    t_max = disarming_time + 5.0 if disarming_time is not None else df_imu['t'].max()

    # Classic RGB standard: Red = X, Green = Y, Blue = Z
    # We use a solid primary color for acceleration, and a distinct dashed style for the corresponding gyroscope
    components = [
        ('X-Axis', 'ax', 'gx', '#d62728', '#fc8d59'),  # Crimson Red Accel & Soft Orange/Red Gyro
        ('Y-Axis', 'ay', 'gy', '#2ca02c', '#a6d96a'),  # Forest Green Accel & Muted Lime Green Gyro
        ('Z-Axis', 'az', 'gz', '#1f77b4', '#9ecae1')   # Deep Blue Accel & Light Blue Gyro
    ]

    for i, (name, a_col, g_col, c_acc, c_gyr) in enumerate(components):
        ax = axs[i]
        
        # Acceleration
        line1 = ax.plot(df_imu['t'], df_imu[a_col], color=c_acc, linewidth=1.5, label=f'Accel {name} (m/s²)')
        ax.set_ylabel(f'Accel (m/s²)', color=c_acc)
        ax.tick_params(axis='y', labelcolor=c_acc)
        
        # Gyro
        ax_gyr = ax.twinx()
        line2 = ax_gyr.plot(df_imu['t'], df_imu[g_col], color=c_gyr, linestyle='-.', alpha=0.8, label=f'Gyro {name} (rad/s)')
        ax_gyr.set_ylabel(f'Gyro (rad/s)', color=c_gyr)
        ax_gyr.tick_params(axis='y', labelcolor=c_gyr)
        
        # Waypoints
        y_pos_text = ax.get_ylim()[1] * 0.8
        for wp_name in sorted(wp_events.keys(), key=lambda k: wp_events[k]):
            t = wp_events[wp_name]
            if t_min <= t <= t_max:
                if wp_name == "Column Passed":
                    ax.axvline(x=t, color='#2CA02C', linestyle='--', alpha=0.9, linewidth=2.2)
                    if i == 0:  # Only label on the top plot
                        ax.text(t + 0.1, y_pos_text, wp_name, rotation=90, va='top', fontsize=8, fontweight='bold',
                                color='#1E5E1E', bbox=dict(facecolor='white', alpha=0.9, edgecolor='#2CA02C', pad=1))
                elif wp_name == "Column Impact":
                    evt_label = wp_name
                    if events_log:
                        for item in events_log:
                            if "Column Impact" in item.get('Event Name', ''):
                                evt_label = item['Event Name'].replace("💥 ", "")
                                break
                    ax.axvline(x=t, color='#D62728', linestyle='-.', alpha=1.0, linewidth=2.5)
                    if i == 0:  # Only label on the top plot
                        ax.text(t + 0.1, y_pos_text * 0.95, evt_label, rotation=90, va='top', fontsize=8, fontweight='bold',
                                color='#7F0000', bbox=dict(facecolor='white', alpha=0.9, edgecolor='#D62728', pad=1))
                else:
                    ax.axvline(x=t, color='black', linestyle=':', alpha=0.5, linewidth=1.0)
                    if i == 0:  # Only label on the top plot
                        ax.text(t + 0.1, y_pos_text, wp_name, rotation=90, va='top', fontsize=8, 
                                bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=1))
        
        ax.grid(True)
        
        # Combine legends for the subplot
        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax.legend(lines, labels, loc='upper right', fontsize=8)

    axs[-1].set_xlabel('Time Since Arming (seconds)')
    axs[-1].set_xlim(t_min, t_max)

    if flight_name:
        axs[-1].text(0.98, 0.02, f"{flight_name}", transform=axs[-1].transAxes,
                     ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                     bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA', boxstyle='round,pad=0.2'))
    
    fig.suptitle(f'📐 Raw IMU X/Y/Z Components ({label})', fontsize=14, y=0.98)
    
    plt.tight_layout()
    plt.show()

def plot_tangential_accel(df_mocap, wp_events, arming_time, takeoff_time, disarming_time, events_log, label="", flight_name=None):
    """Plots the Savitzky-Golay tangential acceleration profile over time.
    Positive values = acceleration, negative = deceleration.
    A bold zero line marks the boundary between the two states.
    Vertical event markers match the standard experiment plot convention.
    """
    if df_mocap.empty or 'accel' not in df_mocap.columns:
        print("[WARN] No tangential acceleration data available. Run compute_velocity() first.")
        return

    fig, ax = plt.subplots(figsize=(12, 5))

    df_plot = df_mocap[df_mocap['t'] >= arming_time].copy()
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

    # --- Event markers (same convention as other experiment plots) ---
    t_min = t_rel.min() if not t_rel.empty else 0
    t_max = t_rel.max() if not t_rel.empty else 10
    y_pos_text = a_abs_max * 0.88

    plot_events = []
    if arming_time is not None:
        plot_events.append(("Armed", 0.0))
    if takeoff_time is not None:
        plot_events.append(("Takeoff", takeoff_time - arming_time))

    for wp_name, t_abs in sorted(wp_events.items(), key=lambda kv: kv[1]):
        t_rel_wp = t_abs - arming_time
        is_impact = 'Impact' in wp_name
        if is_impact:
            # Shade a 0.4s window around impact
            ax.axvspan(t_rel_wp - 0.05, t_rel_wp + 0.35,
                       color='#D62728', alpha=0.10, zorder=3)
            ax.axvline(x=t_rel_wp, color='#D62728', linestyle='-.', alpha=1.0,
                       linewidth=2.5)
            ax.text(t_rel_wp + 0.1, y_pos_text * 0.88, wp_name.replace('💥 ', ''),
                    rotation=90, va='top', fontsize=8, fontweight='bold',
                    color='#7F0000',
                    bbox=dict(facecolor='white', alpha=0.9, edgecolor='#D62728', pad=1))
        else:
            ax.axvline(x=t_rel_wp, color='black', linestyle=':', alpha=0.5, linewidth=1.0)
            ax.text(t_rel_wp + 0.1, y_pos_text, wp_name,
                    rotation=90, va='top', fontsize=8,
                    bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=1))

    for label_name, t_rel_val in plot_events:
        ax.axvline(x=t_rel_val, color='grey', linestyle='--', alpha=0.6, linewidth=1.0)
        ax.text(t_rel_val + 0.1, -y_pos_text, label_name,
                rotation=90, va='bottom', fontsize=8, color='grey')

    ax.set_xlabel('Time Since Arming (seconds)')
    ax.set_ylabel('Tangential Acceleration (m/s²)\n← Deceleration | Acceleration →')
    ax.set_title(f'⚡ Tangential Acceleration / Deceleration Profile ({label})')
    ax.set_xlim(t_min, t_max)
    ax.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#EAEAEA')
    ax.grid(True, alpha=0.5)

    if flight_name:
        ax.text(0.98, 0.02, f"{flight_name}", transform=ax.transAxes,
                ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA', boxstyle='round,pad=0.2'))

    plt.tight_layout()
    plt.show()
