import numpy as np
import matplotlib.pyplot as plt

C_MOCAP = '#1F77B4'      # Steel Blue (Actual Ground Truth)
C_CMD = '#D62728'        # Crimson Red (Commanded Setpoint)
C_BAT = '#9467BD'        # Muted Purple (Battery Voltage)

def plot_velocity_profile(df_mocap, wp_events, arming_time, takeoff_time, disarming_time, events_log, ax=None, label=""):
    """Plots a continuous Savitzky-Golay velocity profile with chronological event markers
    and horizontal segment average speed indicators.
    """
    if df_mocap.empty:
        print("[WARN] Empty MoCap DataFrame, skipping velocity profile plot.")
        return None

    if ax is None:
        fig, ax = plt.subplots(figsize=(12, 6))
        show_plot = True
    else:
        show_plot = False

    df_plot = df_mocap[df_mocap['t'] >= arming_time]
    t_rel = df_plot['t'] - arming_time

    # Plot Savitzky-Golay velocity
    ax.plot(t_rel, df_plot['speed'], color=C_MOCAP, linewidth=2.2, label=f'MoCap SG Velocity ({label})')

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
        ax.hlines(y=avg_v, xmin=t_p, xmax=t_c, colors='#2CA02C', linestyles='--', linewidth=2.0, label=lbl)
        has_avg_legend = True

    # Draw vertical event lines
    plot_events = []
    if arming_time is not None:
        plot_events.append(("Armed", 0.0))
    if takeoff_time is not None:
        plot_events.append(("Takeoff", takeoff_time - arming_time))
    for wp in sorted(wp_events.keys()):
        plot_events.append((wp, wp_events[wp] - arming_time))
    if disarming_time is not None:
        plot_events.append(("Landed", disarming_time - arming_time))

    y_max = df_plot['speed'].max() if not df_plot.empty else 1.0
    if np.isnan(y_max) or y_max <= 0:
        y_max = 1.0

    for idx, (evt_name, t_evt) in enumerate(plot_events):
        ax.axvline(x=t_evt, color='#D62728', linestyle=':', alpha=0.8, linewidth=1.5)
        # Prevent visual overlap by varying label heights
        y_pos = y_max * (0.80 - 0.15 * (idx % 3))
        ax.text(t_evt + 0.4, y_pos, evt_name, rotation=0, fontsize=9, fontweight='bold', color='#444444',
                 bbox=dict(facecolor='white', alpha=0.85, edgecolor='#DDDDDD', boxstyle='round,pad=0.2'))

    ax.set_title(f'⚡ Thesis Flight Kinetic Profile: SG Velocity & Event Timeline ({label})', pad=15)
    ax.set_xlabel('Time Since Arming (seconds)')
    ax.set_ylabel('Velocity Magnitude (m/s)')
    ax.set_xlim(0, t_rel.max() if not t_rel.empty else 10)
    ax.set_ylim(-0.05, y_max * 1.15)
    ax.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#EAEAEA')
    ax.grid(True)

    if show_plot:
        plt.tight_layout()
        plt.show()

def plot_battery_sag(df_bat, takeoff_time, ax=None, label=""):
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
