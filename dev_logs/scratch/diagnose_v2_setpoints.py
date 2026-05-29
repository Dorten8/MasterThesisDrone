import os
import sys
import numpy as np
import pandas as pd

# Setup path
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, "..", "..", ".."))
sys.path.insert(0, os.path.join(project_root, "dev_logs", "analysis", "experiments_analysis"))

from exa_loader import load_mcap, build_dataframes

flight_path = os.path.join(project_root, "dev_logs", "flights", "flight_20260527-1408_75°_column_collision_loop_v2_rotating_cage")
# Try low level loading
from mcap_segmenter import try_load_mcap
topic_data, bag_start_ns, source_path = try_load_mcap(flight_path)

print("=== UNIQUE POSE NAMES ===")
pose_names = set()
for m in topic_data.get('/poses', []):
    for p in m.ros_msg.poses:
        pose_names.add(p.name)
print("Unique pose names found:", pose_names)

dfs = build_dataframes(topic_data, "jake_drone_frame_01", bag_start_ns)
df_sp = dfs['setpoint']
df_mocap = dfs['mocap']
df_odom = dfs.get('odom', pd.DataFrame())

# Let's read EKF odom
odom_list = []
for m in topic_data.get('/fmu/out/vehicle_odometry', []):
    odom_list.append({
        't': (m.log_time_ns - bag_start_ns) * 1e-9,
        'x_ekf': m.ros_msg.position[0], # NED North
        'y_ekf': m.ros_msg.position[1], # NED East
    })
df_odom_test = pd.DataFrame(odom_list)

if not df_mocap.empty and not df_odom_test.empty:
    df_merged = pd.merge_asof(
        df_mocap[['t', 'x', 'y']].sort_values('t'),
        df_odom_test[['t', 'x_ekf', 'y_ekf']].sort_values('t'),
        on='t',
        direction='nearest'
    )
    # EKF North (x_ekf) = MoCap X + offset_x => offset_x = EKF North - MoCap X
    # EKF East (y_ekf) = -MoCap Y + offset_y => offset_y = EKF East + MoCap Y
    offset_x = (df_merged['x_ekf'] - df_merged['x']).median()
    offset_y = (df_merged['y_ekf'] + df_merged['y']).median()
    print("Corrected Offset X:", offset_x)
    print("Corrected Offset Y:", offset_y)
    
    print("\n=== Trajectory Comparison (First 30 rows) ===")
    print(df_merged[['t', 'x', 'y', 'x_ekf', 'y_ekf']].head(30).to_string())
    
    # Apply to all setpoints
    corrected_sps = []
    for m in topic_data.get('/fmu/in/trajectory_setpoint', []):
        cx = m.ros_msg.position[0] - offset_x
        cy = -m.ros_msg.position[1] + offset_y
        corrected_sps.append((cx, cy))
    
    df_sps = pd.DataFrame(corrected_sps, columns=['x_cmd', 'y_cmd']).round(3).drop_duplicates()
    print("\n=== UNIQUE COMMANDED SETPOINTS ===")
    print(df_sps.to_string())
    
    print("\n=== MoCap Bounds ===")
    print("MoCap X range:", df_mocap['x'].min(), "to", df_mocap['x'].max())
    print("MoCap Y range:", df_mocap['y'].min(), "to", df_mocap['y'].max())
else:
    print("No odom/mocap to merge!")
