#!/usr/bin/env python3
import os
import glob
import pyulog
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mcap_ros2.reader import read_ros2_messages

def get_offset_and_bag_start(flight_dir, pass_mcap):
    # Find timesync offset
    offset = None
    bag_start_ns = None
    
    # Check current pass
    for msg in read_ros2_messages(pass_mcap):
        if bag_start_ns is None:
            bag_start_ns = msg.log_time_ns
        bag_start_ns = min(bag_start_ns, msg.log_time_ns)
        
        if msg.channel.topic == "/fmu/out/timesync_status" and offset is None:
            offset = msg.ros_msg.observed_offset
            
    if offset is None:
        mcaps = glob.glob(os.path.join(flight_dir, "*.mcap"))
        for m in mcaps:
            for msg in read_ros2_messages(m):
                if msg.channel.topic == "/fmu/out/timesync_status":
                    offset = msg.ros_msg.observed_offset
                    break
            if offset is not None:
                break
                
    if offset is not None:
        return -(offset * 1e-6), bag_start_ns
    return 0, bag_start_ns

def get_wp_times_from_mcap(pass_mcap, bag_start_ns):
    # Read flight director active waypoint transitions
    wp_times = {}
    for msg in read_ros2_messages(pass_mcap):
        if msg.channel.topic == "/flight_director/active_waypoint":
            wp_val = msg.ros_msg.data
            t_rel = (msg.log_time_ns - bag_start_ns) * 1e-9
            if wp_val not in wp_times:
                wp_times[wp_val] = t_rel
    # Map them:
    # 0 = Takeoff, 1 = Gate (WP1), 2 = Sweep/Impact (WP2), 3 = Recovery (WP3)
    mapped_events = {}
    if 1 in wp_times:
        mapped_events['WP1'] = wp_times[1]
    if 2 in wp_times:
        mapped_events['WP2'] = wp_times[2]
    if 3 in wp_times:
        mapped_events['WP3'] = wp_times[3]
    return mapped_events

def generate_actuator_plot(flight_dir, pass_mcap, ulg_path, output_path):
    print(f"Plotting for {flight_dir}...")
    offset_sec, bag_start_ns = get_offset_and_bag_start(flight_dir, pass_mcap)
    
    wp_events = get_wp_times_from_mcap(pass_mcap, bag_start_ns)
    
    ul = pyulog.ULog(ulg_path, message_name_filter_list=["actuator_motors", "actuator_outputs", "vehicle_status"])
    
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

    # 2. Extract actuator_outputs - robust matching
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

    # 3. Extract vehicle_status
    status_t = []
    nav_state = []
    arming_state = []
    try:
        ds_status = ul.get_dataset("vehicle_status")
        status_t = np.array([to_rel_t(t) for t in ds_status.data["timestamp"]])
        nav_state = ds_status.data["nav_state"]
        arming_state = ds_status.data["arming_state"]
    except Exception:
        pass

    fig = plt.figure(figsize=(14, 10))
    gs = gridspec.GridSpec(3, 1, height_ratios=[1.5, 1.5, 0.8], hspace=0.3)
    
    t_min = wp_events.get('WP1', 0.0) - 2.0
    t_max = wp_events.get('WP3', t_min + 12.0) + 3.0

    colors = ['#FF4B4B', '#4B8BFF', '#4BFF4B', '#FFB34B']
    labels = ['Motor 1 (Front Right)', 'Motor 2 (Rear Left)', 'Motor 3 (Front Left)', 'Motor 4 (Rear Right)']
    
    wp1_t = wp_events.get('WP1')
    impact_t = wp_events.get('WP2')
    wp3_t = wp_events.get('WP3')

    def add_phase_shading(ax):
        if wp1_t is not None and impact_t is not None:
            ax.axvspan(wp1_t, impact_t, color='#FFE0B2', alpha=0.3, label='Sweep Phase')
        if impact_t is not None and wp3_t is not None:
            ax.axvspan(impact_t, wp3_t, color='#C8E6C9', alpha=0.3, label='Recovery Phase')
        if impact_t is not None:
            ax.axvline(x=impact_t, color='#D32F2F', linestyle='--', linewidth=2, label='Impact (WP2)')

    # Panel 1
    ax1 = fig.add_subplot(gs[0])
    add_phase_shading(ax1)
    if len(motor_t) > 0:
        for i in range(4):
            ax1.plot(motor_t, motor_ctrl[i], color=colors[i], label=labels[i], alpha=0.8, linewidth=1.5)
        if impact_t:
            ax1.annotate('IMPACT', xy=(impact_t, 0.8), xytext=(impact_t+0.5, 0.9),
                         arrowprops=dict(facecolor='#D32F2F', shrink=0.05), color='#D32F2F', fontweight='bold')
            
    ax1.set_xlim(t_min, t_max)
    ax1.set_ylim(0.2, 1.0)
    ax1.set_ylabel('Motor Command (0-1)')
    ax1.set_title(f'Actuator Motor Commands\nFlight: {os.path.basename(flight_dir)}', fontweight='bold')
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend(loc='upper left', ncol=2)

    # Panel 2
    ax2 = fig.add_subplot(gs[1], sharex=ax1)
    add_phase_shading(ax2)
    if len(out_t) > 0:
        for i in range(4):
            ax2.plot(out_t, out_vals[i], color=colors[i], label=labels[i], alpha=0.8, linewidth=1.5)
            
    ax2.set_xlim(t_min, t_max)
    ax2.set_ylabel('Actuator Output (PWM/DShot)')
    ax2.set_title('Raw Actuator Outputs to ESCs')
    ax2.grid(True, linestyle='--', alpha=0.6)

    # Panel 3
    ax3 = fig.add_subplot(gs[2], sharex=ax1)
    add_phase_shading(ax3)
    if len(status_t) > 0:
        ax3.step(status_t, nav_state, color='purple', label='Nav State (14=Offboard)', linewidth=2, where='post')
        
    ax3.set_xlim(t_min, t_max)
    ax3.set_ylim(-1, 20)
    ax3.set_ylabel('Nav State ID')
    ax3.set_title('PX4 Vehicle Status')
    ax3.grid(True, linestyle='--', alpha=0.6)
    
    # Arming state secondary axis
    ax3_twin = ax3.twinx()
    if len(status_t) > 0:
        ax3_twin.step(status_t, arming_state, color='#E91E63', linestyle=':', label='Arming State (2=Armed)', linewidth=2, where='post')
    ax3_twin.set_ylabel('Arming State ID')
    ax3_twin.set_ylim(0.5, 2.5)
    
    # Merge legends
    lines, labels_lines = ax3.get_legend_handles_labels()
    lines2, labels2 = ax3_twin.get_legend_handles_labels()
    ax3.legend(lines + lines2, labels_lines + labels2, loc='upper left')

    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Saved {output_path}")

if __name__ == "__main__":
    test_flight = "/home/dorten/MasterThesisDrone/dev_logs/flights/flight_20260528-1239_75°_column_collision_loop_rotating_cage"
    test_pass = glob.glob(os.path.join(test_flight, "*-pass02.mcap"))[0]
    test_ulg = glob.glob(os.path.join(test_flight, "*.ulg"))[0]
    out_png = os.path.join(test_flight, "actuator_health_plot.png")
    generate_actuator_plot(test_flight, test_pass, test_ulg, out_png)

