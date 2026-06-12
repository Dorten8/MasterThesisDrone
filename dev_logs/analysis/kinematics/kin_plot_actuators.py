import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.ticker as ticker
import numpy as np
import pyulog
import os

def plot_actuators_and_status(ulg_path, offset_sec, bag_start_ns, wp_events, arming_time, flight_name, condition, output_path, show_plot=False):
    """
    Reads actuator_motors, actuator_outputs, and vehicle_status directly from the ULog.
    Aligns them using offset_sec and bag_start_ns.
    Plots an 'inventive' multi-panel visualization of motor health and system state.
    """
    try:
        ul = pyulog.ULog(ulg_path, message_name_filter_list=["actuator_motors", "actuator_outputs"])
    except Exception as e:
        print(f"[WARN] Could not load ULog for actuator plot: {e}")
        return

    def to_rel_t(t_us):
        return (t_us * 1e-6) + offset_sec - (bag_start_ns * 1e-9)

    # 1. Extract actuator_motors (Commands)
    motor_t = []
    motor_ctrl = []
    best_ds_motors = None
    for ds in ul.data_list:
        if ds.name == "actuator_motors":
            if best_ds_motors is None or len(ds.data["timestamp"]) > len(best_ds_motors.data["timestamp"]):
                best_ds_motors = ds
    if best_ds_motors is not None:
        try:
            motor_t = np.array([to_rel_t(t) for t in best_ds_motors.data["timestamp"]])
            motor_ctrl = [best_ds_motors.data[f"control[{i}]"] for i in range(4)]
        except Exception:
            pass

    # 2. Extract actuator_outputs (Actual DShot values to ESCs)
    out_t = []
    out_vals = []
    best_ds_out = None
    for ds in ul.data_list:
        if ds.name == "actuator_outputs":
            if best_ds_out is None or len(ds.data["timestamp"]) > len(best_ds_out.data["timestamp"]):
                best_ds_out = ds
    if best_ds_out is not None:
        try:
            out_t = np.array([to_rel_t(t) for t in best_ds_out.data["timestamp"]])
            out_vals = [best_ds_out.data[f"output[{i}]"] for i in range(4)]
        except Exception:
            pass

    fig = plt.figure(figsize=(14, 8))
    gs = gridspec.GridSpec(2, 1, height_ratios=[1.5, 1.5], hspace=0.3)
    
    # Standard timeline window: [WP2 - 1s, WP3 + 1s] in bag-relative time (data is absolute via to_rel_t)
    esp_time = wp_events.get('WP2')   # experiment start (refined forward movement)
    eep_time = wp_events.get('WP3')   # experiment end (sweep end)
    impact_t = wp_events.get('Column Impact') or wp_events.get('WP2')
    t_min = max(0.0, esp_time - 1.0) if esp_time is not None else 0.0
    t_max = eep_time + 1.0 if eep_time is not None else (impact_t + 5.0 if impact_t is not None else 10.0)

    colors = ['#FF4B4B', '#4B8BFF', '#4BFF4B', '#FFB34B']
    labels = ['Motor 1 (Front Right)', 'Motor 2 (Rear Left)', 'Motor 3 (Front Left)', 'Motor 4 (Rear Right)']
    wp1_t = wp_events.get('WP1')
    wp3_t = wp_events.get('WP3')

    # Helper function to add background phase shadings to each axis
    def add_phase_shading(ax):
        sweep_start = esp_time or wp1_t
        if sweep_start is not None and impact_t is not None:
            ax.axvspan(sweep_start, impact_t, color='#FFE0B2', alpha=0.3, label='Sweep Phase')
        if impact_t is not None and wp3_t is not None:
            ax.axvspan(impact_t, wp3_t, color='#C8E6C9', alpha=0.3, label='Recovery Phase')
        if impact_t is not None:
            ax.axvline(x=impact_t, color='#D32F2F', linestyle='--', linewidth=2, label='Impact')

    # Panel 1: Actuator Motors (Commands)
    ax1 = fig.add_subplot(gs[0])
    add_phase_shading(ax1)
    if len(motor_t) > 0:
        for i in range(4):
            ax1.plot(motor_t, motor_ctrl[i], color=colors[i], label=labels[i], alpha=0.8, linewidth=1.5)
        if impact_t:
            ax1.annotate('IMPACT', xy=(impact_t, 0.8), xytext=(impact_t+0.5, 0.9),
                         arrowprops=dict(facecolor='#D32F2F', shrink=0.05), color='#D32F2F', fontweight='bold')
            
    ax1.set_xlim(t_min, t_max)
    ax1.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    ax1.set_ylim(0.4, 1.0)
    ax1.set_ylabel('Motor Cmd\n(normalized)')
    ax1.set_title(f'Actuator Motor Commands <{condition}>', fontweight='bold')
    ax1.grid(True, linestyle='--', alpha=0.6)
    # Put legend outside or tidy up
    ax1.legend(loc='upper right', ncol=2, framealpha=1.0)

    # Panel 2: Actuator Outputs (PWM/DShot)
    ax2 = fig.add_subplot(gs[1], sharex=ax1)
    add_phase_shading(ax2)
    if len(out_t) > 0:
        for i in range(4):
            ax2.plot(out_t, out_vals[i], color=colors[i], label=labels[i], alpha=0.8, linewidth=1.5)

    ax2.set_xlim(t_min, t_max)
    ax2.set_ylabel('Motor Output\n(DShot value → RPM)')
    ax2.set_ylim(750, 2000)
    ax2.set_title('Raw Actuator Outputs to ESCs (DShot Protocol)')
    ax2.set_xlabel('Time (s)')
    ax2.grid(True, linestyle='--', alpha=0.6)

    # Flight name annotation (bottom-right)
    if flight_name:
        ax2.text(0.98, 0.02, f"{flight_name}",
                 transform=ax2.transAxes,
                 ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA',
                           boxstyle='round,pad=0.2'))

    plt.tight_layout()
    if output_path is not None:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
    if show_plot:
        plt.show()
    plt.close()


def plot_control_allocator_saturation(ulg_path, offset_sec, bag_start_ns, wp_events, flight_name, condition, output_path, show_plot=False):
    import matplotlib.pyplot as plt
    import numpy as np
    import pyulog
    import os

    try:
        ul = pyulog.ULog(ulg_path, message_name_filter_list=["actuator_motors", "control_allocator_status"])
    except Exception as e:
        print(f"[WARN] Could not load ULog for control allocator saturation plot: {e}")
        return

    def to_rel_t(t_us):
        return (t_us * 1e-6) + offset_sec - (bag_start_ns * 1e-9)

    # Extract actuator_motors
    motor_t = []
    motor_ctrl = []
    try:
        ds_motors = ul.get_dataset("actuator_motors")
        motor_t = np.array([to_rel_t(t) for t in ds_motors.data["timestamp"]])
        motor_ctrl = [ds_motors.data[f"control[{i}]"] for i in range(4)]
    except Exception:
        pass

    # Extract control_allocator_status
    alloc_t = []
    sat = []
    unallocated_torque_norm = []
    torque_achieved = []
    thrust_achieved = []
    try:
        ds_alloc = ul.get_dataset("control_allocator_status")
        alloc_t = np.array([to_rel_t(t) for t in ds_alloc.data["timestamp"]])
        sat = [ds_alloc.data[f"actuator_saturation[{i}]"] for i in range(4)]
        unallocated_torque_norm = np.sqrt(
            ds_alloc.data["unallocated_torque[0]"]**2 +
            ds_alloc.data["unallocated_torque[1]"]**2 +
            ds_alloc.data["unallocated_torque[2]"]**2
        )
        torque_achieved = ds_alloc.data["torque_setpoint_achieved"]
        thrust_achieved = ds_alloc.data["thrust_setpoint_achieved"]
    except Exception:
        pass

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True, dpi=150)
    
    t_impact = wp_events.get('Column Impact', None)
    if t_impact is None or np.isnan(t_impact):
        t_impact = wp_events.get('WP2', None)
    if t_impact is None or np.isnan(t_impact):
        t_impact = wp_events.get('WP1', 0.0)

    # Standard timeline window: [WP2 - 1s, WP3 + 1s] in bag-relative time (data is absolute via to_rel_t)
    esp_time = wp_events.get('WP2')
    eep_time = wp_events.get('WP3')
    t_min = max(0.0, esp_time - 1.0) if esp_time is not None else (t_impact - 2.0)
    t_max = eep_time + 1.0 if eep_time is not None else (t_impact + 3.0)

    colors = ['#FF4B4B', '#4B8BFF', '#4BFF4B', '#FFB34B']
    labels = ['Motor 1 (FR)', 'Motor 2 (RL)', 'Motor 3 (FL)', 'Motor 4 (RR)']

    # Subpanel 1: Motor Command Input
    ax = axes[0]
    if len(motor_t) > 0:
        for i in range(4):
            ax.plot(motor_t, motor_ctrl[i], color=colors[i], label=labels[i], alpha=0.8, linewidth=1.5)
    ax.set_ylabel('Motor Command (normalized, 0.0 to 1.0)')
    ax.set_title(f'Control Allocator Saturation Analysis <{condition}>', fontweight='bold')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper left', ncol=2)
    ax.set_ylim(0.0, 1.1)

    # Subpanel 2: Saturation State
    ax = axes[1]
    if len(alloc_t) > 0:
        for i in range(4):
            ax.step(alloc_t, sat[i], color=colors[i], label=labels[i], alpha=0.8, linewidth=1.5, where='post')
        # Highlight saturation duration
        sat_any = np.any([sat[i] != 0 for i in range(4)], axis=0)
        # shade saturated intervals
        ax.fill_between(alloc_t, -2.5, 2.5, where=sat_any, color='red', alpha=0.1, label='Saturated Interval')
    ax.set_ylabel('Saturation State\n(-2=Min, 2=Max)')
    ax.set_ylim(-2.5, 2.5)
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper left', ncol=2)

    # Subpanel 3: Control Allocation Deficiency
    ax = axes[2]
    if len(alloc_t) > 0:
        ax.plot(alloc_t, unallocated_torque_norm, color='purple', label='Norm of Unallocated Torque (N·m)', linewidth=1.8)
        ax_twin = ax.twinx()
        ax_twin.step(alloc_t, torque_achieved, color='green', linestyle='--', label='Torque SP Achieved', linewidth=1.2, where='post')
        ax_twin.step(alloc_t, thrust_achieved, color='blue', linestyle=':', label='Thrust SP Achieved', linewidth=1.2, where='post')
        ax_twin.set_ylabel('Setpoint Achieved (0/1)')
        ax_twin.set_ylim(-0.1, 1.1)
        # Merge legends
        lines, labels_lines = ax.get_legend_handles_labels()
        lines2, labels2 = ax_twin.get_legend_handles_labels()
        ax.legend(lines + lines2, labels_lines + labels2, loc='upper left')
    ax.set_ylabel('Unallocated Torque (N·m)')
    ax.set_xlabel('Time (s)')
    ax.grid(True, linestyle=':', alpha=0.6)

    # Draw vertical dashed line at t_impact, uniform 1.0s ticks
    for a in axes:
        a.axvline(x=t_impact, color='#D32F2F', linestyle='--', linewidth=2, label='Impact')
        a.set_xlim(t_min, t_max)
    axes[0].xaxis.set_major_locator(ticker.MultipleLocator(1.0))

    # Flight name annotation (bottom-right)
    if flight_name:
        axes[-1].text(0.98, 0.02, f"{flight_name}",
                      transform=axes[-1].transAxes,
                      ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                      bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA',
                                boxstyle='round,pad=0.2'))

    plt.tight_layout()
    if output_path is not None:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
    if show_plot:
        plt.show()
    plt.close()


def plot_pid_rate_tracking(ulg_path, offset_sec, bag_start_ns, wp_events, flight_name, condition, output_path, show_plot=False):
    import matplotlib.pyplot as plt
    import numpy as np
    import pyulog
    import os

    try:
        ul = pyulog.ULog(ulg_path, message_name_filter_list=["vehicle_rates_setpoint", "vehicle_angular_velocity"])
    except Exception as e:
        print(f"[WARN] Could not load ULog for PID rate tracking plot: {e}")
        return

    def to_rel_t(t_us):
        return (t_us * 1e-6) + offset_sec - (bag_start_ns * 1e-9)

    # Extract vehicle_rates_setpoint
    sp_t = []
    sp_roll = []
    sp_pitch = []
    sp_yaw = []
    try:
        ds_sp = ul.get_dataset("vehicle_rates_setpoint")
        sp_t = np.array([to_rel_t(t) for t in ds_sp.data["timestamp"]])
        sp_roll = ds_sp.data["roll"]
        sp_pitch = ds_sp.data["pitch"]
        sp_yaw = ds_sp.data["yaw"]
    except Exception:
        pass

    # Extract vehicle_angular_velocity
    vel_t = []
    vel_roll = []
    vel_pitch = []
    vel_yaw = []
    try:
        ds_vel = ul.get_dataset("vehicle_angular_velocity")
        vel_t = np.array([to_rel_t(t) for t in ds_vel.data["timestamp"]])
        vel_roll = ds_vel.data["xyz[0]"]
        vel_pitch = ds_vel.data["xyz[1]"]
        vel_yaw = ds_vel.data["xyz[2]"]
    except Exception:
        pass

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True, dpi=150)
    
    t_impact = wp_events.get('Column Impact', None)
    if t_impact is None or np.isnan(t_impact):
        t_impact = wp_events.get('WP2', None)
    if t_impact is None or np.isnan(t_impact):
        t_impact = wp_events.get('WP1', 0.0)

    # Standard timeline window: [WP2 - 1s, WP3 + 1s] in bag-relative time (data is absolute via to_rel_t)
    esp_time = wp_events.get('WP2')
    eep_time = wp_events.get('WP3')
    t_min = max(0.0, esp_time - 1.0) if esp_time is not None else (t_impact - 2.0)
    t_max = eep_time + 1.0 if eep_time is not None else (t_impact + 3.0)

    axes_labels = ['Roll Rate Tracking (rad/s)', 'Pitch Rate Tracking (rad/s)', 'Yaw Rate Tracking (rad/s)']
    axes_data = [
        (sp_roll, vel_roll, 'Roll'),
        (sp_pitch, vel_pitch, 'Pitch'),
        (sp_yaw, vel_yaw, 'Yaw')
    ]

    for idx, (sp_axis, vel_axis, name) in enumerate(axes_data):
        ax = axes[idx]
        if len(sp_t) > 0 and len(vel_t) > 0:
            ax.plot(sp_t, sp_axis, color='#444444', linestyle='--', label=f'Commanded {name} Rate', linewidth=1.5)
            ax.plot(vel_t, vel_axis, color='#1F77B4' if condition == 'Rotating Cage' else '#D62728', linestyle='-', label=f'Actual {name} Rate', linewidth=1.5)
            
            # Interpolate setpoint to actual velocity timestamps to compute shaded tracking error
            sp_interp = np.interp(vel_t, sp_t, sp_axis)
            abs_err = np.abs(vel_axis - sp_interp)
            
            # Plot error as a shaded band at the bottom
            ax.fill_between(vel_t, 0, abs_err, color='purple', alpha=0.15, label='Absolute Tracking Error')
            ax.plot(vel_t, abs_err, color='purple', alpha=0.3, linewidth=0.8)
            
        ax.set_ylabel(axes_labels[idx])
        ax.grid(True, linestyle=':', alpha=0.6)
        ax.legend(loc='upper left', ncol=3)

    axes[0].set_title(f'PID Rate Controller Tracking Performance <{condition}>', fontweight='bold')
    axes[2].set_xlabel('Time (s)')

    # Draw vertical dashed line at t_impact, uniform 1.0s ticks
    for a in axes:
        a.axvline(x=t_impact, color='#D32F2F', linestyle='--', linewidth=2, label='Impact')
        a.set_xlim(t_min, t_max)
    axes[0].xaxis.set_major_locator(ticker.MultipleLocator(1.0))

    # Flight name annotation (bottom-right)
    if flight_name:
        axes[-1].text(0.98, 0.02, f"{flight_name}",
                      transform=axes[-1].transAxes,
                      ha='right', va='bottom', fontsize=8, alpha=0.7, zorder=10,
                      bbox=dict(facecolor='white', alpha=0.8, edgecolor='#EAEAEA',
                                boxstyle='round,pad=0.2'))

    plt.tight_layout()
    if output_path is not None:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
    if show_plot:
        plt.show()
    plt.close()


