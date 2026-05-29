import os
import sys
import numpy as np
import pandas as pd

project_root = "/home/dorten/pi_drone_sshfs"
sys.path.append(os.path.join(project_root, "dev_logs", "analysis"))

from experiments_analysis.exa_loader import load_mcap, build_dataframes, load_drone_metadata

def main():
    flight_name = "flight_20260524-1007_75°_column_collision_loop"
    flight_path = os.path.join(project_root, "dev_logs", "flights", flight_name)
    
    print(f"Analyzing EKF2 vs MoCap drift for: {flight_name}")
    topic_data, bag_start_ns = load_mcap(flight_path)
    drone_tracker_name, _ = load_drone_metadata(project_root)
    dfs = build_dataframes(topic_data, drone_tracker_name, bag_start_ns)
    
    df_mocap = dfs['mocap']
    df_odom = dfs['odom']
    df_setpoint = dfs['setpoint']
    arming_time = dfs['arming_time']
    
    merged = pd.merge_asof(df_mocap.sort_values('t'), df_odom.sort_values('t'), on='t', direction='nearest')
    merged = pd.merge_asof(merged, df_setpoint.sort_values('t'), on='t', direction='nearest')
    
    # Calculate offset in ENU (both are already in ENU since exa_loader handles NED->ENU conversion!)
    merged['dx_enu'] = merged['x_ekf'] - merged['x']
    merged['dy_enu'] = merged['y_ekf'] - merged['y']
    merged['dz_enu'] = merged['z_ekf'] - merged['z']
    
    print("\n--- Corrected Telemetry ENU Coordinate Alignment Timeline ---")
    timeline_times = [0.0, arming_time, arming_time + 5.0, arming_time + 10.0, arming_time + 20.0, arming_time + 30.0, merged['t'].max()]
    for t in timeline_times:
        if t > merged['t'].max():
            continue
        row = merged.loc[(merged['t'] - t).abs().idxmin()]
        print(f"t={row['t']:.2f}s | MoCap ENU: ({row['x']:.3f}, {row['y']:.3f}) | EKF2 ENU: ({row['x_ekf']:.3f}, {row['y_ekf']:.3f}) | Offset (EKF-MoCap): ({row['dx_enu']:.3f}, {row['dy_enu']:.3f})")
        
    print("\n--- Trajectory Tracking Accuracy Timeline (Commanded vs Actual) ---")
    for t in [arming_time + 5.0, arming_time + 10.0, arming_time + 15.0, arming_time + 20.0, arming_time + 25.0]:
        if t > merged['t'].max():
            continue
        row = merged.loc[(merged['t'] - t).abs().idxmin()]
        print(f"t={row['t']:.2f}s | Commanded Target ENU: ({row['x_cmd']:.3f}, {row['y_cmd']:.3f}) | Actual MoCap ENU: ({row['x']:.3f}, {row['y']:.3f}) | Error: ({row['x']-row['x_cmd']:.3f}, {row['y']-row['y_cmd']:.3f})")

if __name__ == "__main__":
    main()
