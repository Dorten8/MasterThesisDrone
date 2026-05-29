import os
import sys
import numpy as np
import pandas as pd

project_root = "/home/dorten/pi_drone_sshfs"
sys.path.append(os.path.join(project_root, "dev_logs", "analysis"))

from experiments_analysis.exa_loader import load_mcap, build_dataframes, load_drone_metadata

def quaternion_to_euler(w, x, y, z):
    # returns yaw, pitch, roll in radians
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)
    
    return yaw, pitch, roll

def main():
    flight_name = "flight_20260524-1007_75°_column_collision_loop"
    flight_path = os.path.join(project_root, "dev_logs", "flights", flight_name)
    
    print(f"Analyzing Yaw and Odometry for: {flight_name}")
    topic_data, bag_start_ns = load_mcap(flight_path)
    drone_tracker_name, _ = load_drone_metadata(project_root)
    dfs = build_dataframes(topic_data, drone_tracker_name, bag_start_ns)
    
    df_mocap = dfs['mocap']
    df_odom = dfs['odom']
    df_setpoint = dfs['setpoint']
    arming_time = dfs['arming_time']
    
    # Calculate yaw for MoCap
    mocap_ypr = [quaternion_to_euler(r['qw'], r['qx'], r['qy'], r['qz']) for _, r in df_mocap.iterrows()]
    df_mocap['yaw'] = [ypr[0] for ypr in mocap_ypr]
    
    # Calculate yaw for EKF2 (odom)
    odom_ypr = [quaternion_to_euler(r['q0'], r['q1'], r['q2'], r['q3']) for _, r in df_odom.iterrows()]
    df_odom['yaw'] = [ypr[0] for ypr in odom_ypr]
    
    merged = pd.merge_asof(df_mocap.sort_values('t'), df_odom.sort_values('t'), on='t', direction='nearest')
    
    # Calculate Yaw error
    merged['yaw_err'] = np.degrees(np.arctan2(np.sin(merged['yaw_y'] - merged['yaw_x']), np.cos(merged['yaw_y'] - merged['yaw_x'])))
    
    print("\n--- Yaw Yaw Alignment Timeline (Degrees) ---")
    timeline_times = [0.0, arming_time, arming_time + 5.0, arming_time + 10.0, arming_time + 20.0, arming_time + 30.0, merged['t'].max()]
    for t in timeline_times:
        if t > merged['t'].max():
            continue
        row = merged.loc[(merged['t'] - t).abs().idxmin()]
        print(f"t={row['t']:.2f}s | MoCap Yaw: {np.degrees(row['yaw_x']):.1f}° | EKF2 Yaw: {np.degrees(row['yaw_y']):.1f}° | Error: {row['yaw_err']:.1f}°")
        
    print("\n--- Tracking Loss/Gaps check ---")
    # Let's count how many times the gap between visual odometry messages exceeds 50ms
    vvo_msgs = topic_data.get("/fmu/in/vehicle_visual_odometry", [])
    if len(vvo_msgs) > 1:
        vvo_times = np.array([m.log_time_ns * 1e-9 for m in vvo_msgs])
        gaps = np.diff(vvo_times)
        large_gaps = np.where(gaps > 0.05)[0]
        print(f"Total gaps > 50ms in vehicle_visual_odometry: {len(large_gaps)}")
        for idx in large_gaps[:10]:
            print(f"  Gap of {gaps[idx]*1000:.1f}ms at t={vvo_times[idx] - bag_start_ns*1e-9:.2f}s")
            
    # Check for ROS 2 logger warning messages
    ros_logs = topic_data.get("/rosout", [])
    print(f"\nTotal /rosout messages: {len(ros_logs)}")
    warnings = []
    for m in ros_logs:
        if m.msg.level >= 3: # WARN or higher
            warnings.append(f"[{m.msg.stamp.sec}.{m.msg.stamp.nanosec}] [{m.msg.name}] {m.msg.msg}")
    print(f"Found {len(warnings)} Warnings/Errors:")
    for w in warnings[:20]:
        print(f"  {w}")

if __name__ == "__main__":
    main()
