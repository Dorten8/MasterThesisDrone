#!/usr/bin/env python3
import os
import sys
import numpy as np
import pandas as pd
from mcap_ros2.reader import read_ros2_messages as _read_msgs

# Add project root to path
project_root = "/home/dorten/pi_drone_sshfs"
sys.path.insert(0, project_root)

from dev_logs.analysis.experiments_analysis.exa_loader import build_dataframes, load_drone_metadata
from dev_logs.analysis.experiments_analysis.exa_kinematics import compute_velocity, calculate_metrics
from dev_logs.analysis.experiments_analysis.exa_database import insert_or_replace_flight

# Target flight details
folder_name = "flight_20260527-1408_75°_column_collision_loop_v2_rotating_cage"
pass_path = os.path.join(project_root, "dev_logs", "flights", folder_name, "flight_20260527-1408_75°_column_collision_loop_v2_rotating_cage_0-pass01.mcap")
pass_name = f"{folder_name} - Pass-01"

print(f"🎬 Standalone Populator for: {pass_name}")

# Load messages
topic_data = {}
for msg in _read_msgs(pass_path):
    t = msg.channel.topic
    if t not in topic_data:
        topic_data[t] = []
    topic_data[t].append(msg)

bag_start_ns = min(m.log_time_ns for msgs in topic_data.values() for m in msgs)
drone_tracker_name, _ = load_drone_metadata(project_root)
dfs = build_dataframes(topic_data, drone_tracker_name, bag_start_ns)

df_mocap  = dfs['mocap']
df_setpoint = dfs.get('setpoint', pd.DataFrame())
df_bat    = dfs['battery']
df_column = dfs.get('column', pd.DataFrame())
arming_time       = dfs['arming_time']
dynamic_waypoints = dfs.get('dynamic_waypoints', [])

df_mocap = compute_velocity(df_mocap)

col_x, col_y = (df_column['x'].head(50).mean(), df_column['y'].head(50).mean()) \
    if (df_column is not None and not df_column.empty) else (0.408, 0.358)

# 1. Determine waypoints directly for V2
wp2_coords = (0.186, 1.100)
wp3_coords = (0.186, -1.200)

dist_to_wp2 = np.sqrt((df_mocap['x'] - wp2_coords[0])**2 + (df_mocap['y'] - wp2_coords[1])**2)
dist_to_wp3 = np.sqrt((df_mocap['x'] - wp3_coords[0])**2 + (df_mocap['y'] - wp3_coords[1])**2)

t_wp2 = df_mocap.loc[dist_to_wp2.idxmin(), 't']
t_wp3 = df_mocap.loc[dist_to_wp3.idxmin(), 't']

# Robust correction for pass files: if t_wp2 > t_wp3, set t_wp2 to start of file!
if t_wp2 >= t_wp3:
    print(f"⚠️ Backwards proximity detected (t_wp2={t_wp2:.3f} >= t_wp3={t_wp3:.3f}). Applying robust start-of-file fallback.")
    t_wp2 = df_mocap['t'].min()

wp_events = {
    'WP1': t_wp2,
    'WP2': t_wp2,
    'WP3': t_wp3,
    'WP4': df_mocap['t'].max()
}

# Post-process Column Passed and Column Impact detection
sweep_data = df_mocap[(df_mocap['t'] >= t_wp2) & (df_mocap['t'] <= t_wp3)]
if not sweep_data.empty:
    idx_min = (sweep_data['y'] - col_y).abs().idxmin()
    wp_events['Column Passed'] = sweep_data.loc[idx_min, 't']
    
    dist_profile = np.sqrt((sweep_data['x'] - col_x)**2 + (sweep_data['y'] - col_y)**2)
    if not dist_profile.empty:
        idx_min_dist = dist_profile.idxmin()
        min_dist = dist_profile.loc[idx_min_dist]
        t_geom = sweep_data.loc[idx_min_dist, 't']
        
        # Onset of deceleration (speed peak search around closest approach t_geom)
        window_mask = (sweep_data['t'] >= t_geom - 0.25) & (sweep_data['t'] <= t_geom + 0.1)
        window_data = sweep_data[window_mask]
        if not window_data.empty:
            idx_peak_speed = window_data['speed'].idxmax()
            wp_events['Column Impact'] = window_data.loc[idx_peak_speed, 't']
        else:
            wp_events['Column Impact'] = t_geom
else:
    wp_events['Column Passed'] = t_wp2 + (t_wp3 - t_wp2)/2.0
    wp_events['Column Impact'] = wp_events['Column Passed']

print(f"Computed wp_events: {wp_events}")

# 2. Calculate metrics
metrics = calculate_metrics(df_mocap, wp_events, col_x, col_y, 0.045, 0.179)

nom_sp = (0.186, 1.100, 0.500)
nom_ep = (0.186, -1.200, 0.500)
metrics['nom_sp'] = nom_sp
metrics['nom_ep'] = nom_ep

def _get_mocap_coords(df, t):
    if df.empty or t is None or np.isnan(t):
        return (None, None, None)
    idx = (df['t'] - t).abs().idxmin()
    return (float(df.loc[idx, 'x']), float(df.loc[idx, 'y']), float(df.loc[idx, 'z']))

metrics['act_sp'] = _get_mocap_coords(df_mocap, wp_events.get('WP2') or wp_events.get('WP1'))
metrics['act_ep'] = _get_mocap_coords(df_mocap, wp_events.get('WP3'))
metrics['sweep_speed'] = 0.30

# Battery % at Exp. Start-point
t_exp_start = wp_events.get('WP1') or wp_events.get('WP2')
if t_exp_start is not None and not df_bat.empty:
    from dev_logs.analysis.experiments_analysis.exa_kinematics import query_battery as _qb
    bat_pct_str, _ = _qb(df_bat, t_exp_start)
    try:
        metrics['battery_at_start'] = float(bat_pct_str.replace('%', ''))
    except (ValueError, AttributeError):
        metrics['battery_at_start'] = None
else:
    metrics['battery_at_start'] = None

# Condition
condition = "Rotating Cage"

print("\n--- Calculated Metrics ---")
for k, v in metrics.items():
    print(f"  {k}: {v}")

insert_or_replace_flight(pass_name, condition, metrics)
print("\n✅ Successfully populated database for flight 1408!")
sys.exit(0)
