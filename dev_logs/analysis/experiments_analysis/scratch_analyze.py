import os
import sys
import numpy as np
import pandas as pd

# Add the parent directories to path so we can import experiments_analysis
project_root = "/home/dorten/pi_drone_sshfs"
sys.path.append(os.path.join(project_root, "dev_logs", "analysis"))

from experiments_analysis.exa_loader import load_mcap, build_dataframes, load_drone_metadata

def main():
    flight_name = "flight_20260524-1007_75°_column_collision_loop"
    flight_path = os.path.join(project_root, "dev_logs", "flights", flight_name)
    
    print(f"Analyzing flight: {flight_name}")
    
    # Load MCAP
    topic_data, bag_start_ns = load_mcap(flight_path)
    drone_tracker_name, _ = load_drone_metadata(project_root)
    print(f"Primary tracker name: {drone_tracker_name}")
    
    dfs = build_dataframes(topic_data, drone_tracker_name, bag_start_ns)
    
    df_mocap = dfs['mocap']
    df_setpoint = dfs['setpoint']
    df_odom = dfs['odom']
    arming_time = dfs['arming_time']
    disarming_time = dfs['disarming_time']
    
    print(f"Loaded dataframes:")
    print(f"  mocap rows: {len(df_mocap)}")
    print(f"  setpoint rows: {len(df_setpoint)}")
    print(f"  odom rows: {len(df_odom)}")
    print(f"  arming time: {arming_time:.2f}s, disarming time: {disarming_time if disarming_time else 'N/A'}")
    
    if df_mocap.empty or df_odom.empty:
        print("Error: Empty mocap or odom dataframe!")
        return
        
    # Let's inspect the offset at initialization (first few seconds, t < 5.0)
    init_mocap = df_mocap[df_mocap['t'] < 5.0]
    init_odom = df_odom[df_odom['t'] < 5.0]
    
    print("\n--- Initial State Analysis (First 5 seconds) ---")
    if not init_mocap.empty:
        print(f"Mean MoCap: X={init_mocap['x'].mean():.3f}, Y={init_mocap['y'].mean():.3f}, Z={init_mocap['z'].mean():.3f}")
    if not init_odom.empty:
        print(f"Mean EKF2:  X={init_odom['x_ekf'].mean():.3f}, Y={init_odom['y_ekf'].mean():.3f}, Z={init_odom['z_ekf'].mean():.3f}")
        
    # Let's check when the drone was armed
    print(f"\n--- State at Arming (t = {arming_time:.2f}s) ---")
    # Find closest MoCap and EKF points to arming_time
    idx_mocap_arm = (df_mocap['t'] - arming_time).abs().idxmin()
    idx_odom_arm = (df_odom['t'] - arming_time).abs().idxmin()
    
    m_arm = df_mocap.loc[idx_mocap_arm]
    o_arm = df_odom.loc[idx_odom_arm]
    
    print(f"MoCap at arming: t={m_arm['t']:.2f}s, X={m_arm['x']:.3f}, Y={m_arm['y']:.3f}, Z={m_arm['z']:.3f}")
    print(f"EKF2 at arming:  t={o_arm['t']:.2f}s, X={o_arm['x_ekf']:.3f}, Y={o_arm['y_ekf']:.3f}, Z={o_arm['z_ekf']:.3f}")
    
    # EKF NED coordinates to ENU: EKF_X = ENU_X, EKF_Y = -ENU_Y, EKF_Z = -ENU_Z
    # So:
    # m_ned_x = m_arm['x']
    # m_ned_y = -m_arm['y']
    # m_ned_z = -m_arm['z']
    # dx = o_arm['x_ekf'] - m_ned_x
    # dy = o_arm['y_ekf'] - m_ned_y
    # dz = o_arm['z_ekf'] - m_ned_z
    m_ned_x = m_arm['x']
    m_ned_y = -m_arm['y']
    m_ned_z = -m_arm['z']
    dx = o_arm['x_ekf'] - m_ned_x
    dy = o_arm['y_ekf'] - m_ned_y
    dz = o_arm['z_ekf'] - m_ned_z
    print(f"Calculated offset (EKF - MoCap_NED) at arming: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}")
    
    # Let's see the actual trajectory setpoint sent when armed vs MoCap
    # In _send_mocap_setpoint:
    # msg.position = [ m_ned_x + dx, m_ned_y + dy, m_ned_z + dz ]
    # We reconstruct the commanded ENU from df_setpoint:
    # cmd_x = msg.position[0] - dx
    # cmd_y = -(msg.position[1] - dy)
    
    # Let's inspect the active flight phase (t > arming_time)
    flight_setpoint = df_setpoint[df_setpoint['t'] > arming_time + 2.0]
    flight_mocap = df_mocap[df_mocap['t'] > arming_time + 2.0]
    flight_odom = df_odom[df_odom['t'] > arming_time + 2.0]
    
    print("\n--- Active Flight Trajectory Analysis ---")
    if not flight_setpoint.empty and not flight_mocap.empty:
        # Find maximum Y in setpoint commands
        max_y_cmd_idx = flight_setpoint['y_cmd'].idxmax()
        sp_max_y = flight_setpoint.loc[max_y_cmd_idx]
        
        # Find corresponding MoCap coordinate at that timestamp
        idx_mocap_max_y = (df_mocap['t'] - sp_max_y['t']).abs().idxmin()
        m_max_y = df_mocap.loc[idx_mocap_max_y]
        
        print(f"Setpoint Target near Y_max (t={sp_max_y['t']:.2f}s): X_cmd={sp_max_y['x_cmd']:.3f}, Y_cmd={sp_max_y['y_cmd']:.3f}")
        print(f"Actual MoCap Position at that time:       X_mocap={m_max_y['x']:.3f}, Y_mocap={m_max_y['y']:.3f}")
        print(f"Discrepancy (Actual - Commanded):          dX={m_max_y['x'] - sp_max_y['x_cmd']:.3f}, dY={m_max_y['y'] - sp_max_y['y_cmd']:.3f}")
        
        # Let's look at the sweep start (where Y = 1.200)
        # Find setpoints close to WP1 (0, 1.2) or WP2 (-0.186, 1.2)
        # Let's print out a few setpoints around the time the drone reached Y_cmd ≈ 1.200
        wp1_sps = flight_setpoint[(flight_setpoint['y_cmd'] > 1.15) & (flight_setpoint['y_cmd'] < 1.25)].head(5)
        print("\nSetpoints sent near Y_cmd ≈ 1.2m:")
        for idx, sp in wp1_sps.iterrows():
            idx_m = (df_mocap['t'] - sp['t']).abs().idxmin()
            m = df_mocap.loc[idx_m]
            print(f"  t={sp['t']:.2f}s | Cmd: ({sp['x_cmd']:.3f}, {sp['y_cmd']:.3f}) | Actual MoCap: ({m['x']:.3f}, {m['y']:.3f}) | Delta: ({m['x']-sp['x_cmd']:.3f}, {m['y']-sp['y_cmd']:.3f})")

    # Rate and dropout analysis
    print("\n--- Rate & Dropout Analysis ---")
    poses_msgs = topic_data.get("/poses", [])
    vvo_msgs = topic_data.get("/fmu/in/vehicle_visual_odometry", [])
    
    print(f"Total /poses messages: {len(poses_msgs)}")
    print(f"Total VVO messages: {len(vvo_msgs)}")
    
    if len(poses_msgs) > 1:
        poses_times = [m.log_time_ns * 1e-9 for m in poses_msgs]
        poses_gaps = np.diff(poses_times)
        print(f"/poses rate: mean={1.0/np.mean(poses_gaps):.1f}Hz, max_gap={np.max(poses_gaps)*1000:.1f}ms, median_gap={np.median(poses_gaps)*1000:.1f}ms")
        large_poses_gaps = [g for g in poses_gaps if g > 0.033]
        print(f"  Gaps > 33ms in /poses: {len(large_poses_gaps)} / {len(poses_gaps)} ({len(large_poses_gaps)/len(poses_gaps)*100:.1f}%)")
        
    if len(vvo_msgs) > 1:
        vvo_times = [m.log_time_ns * 1e-9 for m in vvo_msgs]
        vvo_gaps = np.diff(vvo_times)
        print(f"VVO rate:   mean={1.0/np.mean(vvo_gaps):.1f}Hz, max_gap={np.max(vvo_gaps)*1000:.1f}ms, median_gap={np.median(vvo_gaps)*1000:.1f}ms")
        large_vvo_gaps = [g for g in vvo_gaps if g > 0.033]
        print(f"  Gaps > 33ms in VVO:   {len(large_vvo_gaps)} / {len(vvo_gaps)} ({len(large_vvo_gaps)/len(vvo_gaps)*100:.1f}%)")

if __name__ == "__main__":
    main()
