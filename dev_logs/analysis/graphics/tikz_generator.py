import os
import sys
import numpy as np

# Resolve relative directories based on project root layout
project_root = os.path.dirname(os.path.dirname(os.path.abspath('__file__' if '__file__' in locals() else os.getcwd())))
analysis_dir = os.path.join(project_root, "dev_logs", "analysis")

if project_root not in sys.path:
    sys.path.insert(0, project_root)
if analysis_dir not in sys.path:
    sys.path.insert(0, analysis_dir)

from graphics import generate_trajectory_tikz
from dev_logs.analysis.database.db_loader import load_drone_metadata, load_mcap, build_dataframes
from dev_logs.analysis.kinematics.kin_calculator import compute_mocap_kinematics, find_waypoint_events, compute_flight_metrics

# Define which experiment and flight path you want to build the LaTeX figure for
TIKZ_FLIGHT_NAME = "flight_20260526-0931_75°_column_collision_loop_rotating_cage"
tikz_flight_path = os.path.join(
    project_root, "dev_logs", "flights", TIKZ_FLIGHT_NAME, 
    "flight_20260526-0931_75°_column_collision_loop_rotating_cage_0-pass04.mcap"
)

print(f"🔮 Generating TikZ vector code from: {os.path.basename(tikz_flight_path)}")

# Load configs and data
drone_tracker_name, system_config = load_drone_metadata(project_root)
primary_body = next((b for b in system_config.get("tracked_bodies", []) if b.get("role") == "primary"), {})
obstacle_body = next((b for b in system_config.get("tracked_bodies", []) if b.get("role") == "obstacle"), {})

cage_diameter = primary_body.get("cage_diameter_m", 0.358)
column_diameter = obstacle_body.get("diameter_m", 0.09)
cage_radius = cage_diameter / 2.0
column_radius = column_diameter / 2.0

topic_data, bag_start_ns = load_mcap(tikz_flight_path)
dfs = build_dataframes(topic_data, drone_tracker_name, bag_start_ns)
df_mocap = compute_mocap_kinematics(dfs['mocap'])

takeoff_mask = df_mocap['z'] > 0.15
takeoff_time = df_mocap.loc[takeoff_mask, 't'].iloc[0] if takeoff_mask.any() else dfs['arming_time'] + 2.0
df_setpoint = dfs.get('setpoint', __import__('pandas').DataFrame())
dynamic_waypoints = dfs.get('dynamic_waypoints', [])
wp_events = find_waypoint_events(
    df_mocap, df_setpoint, takeoff_time, 
    label="75° Column Collision", 
    column_x=0.408, column_y=0.358, 
    return_all=True, dynamic_waypoints=dynamic_waypoints
)[0]

metrics = compute_flight_metrics(df_mocap, wp_events, 0.408, 0.358, column_radius, cage_radius)

# Extract sweep segment
t_start = wp_events.get('WP1', df_mocap['t'].iloc[0])
t_end = wp_events.get('WP4', df_mocap['t'].iloc[-1])
df_mocap_sweep = df_mocap[(df_mocap['t'] >= t_start) & (df_mocap['t'] <= t_end)]

# Query positions at WP2 and WP3 transitions
def get_coords(wp_name, fallback):
    t_wp = wp_events.get(wp_name)
    if t_wp is not None:
        idx = np.searchsorted(df_mocap['t'], t_wp)
        idx = min(max(0, idx), len(df_mocap) - 1)
        return df_mocap['x'].iloc[idx], df_mocap['y'].iloc[idx]
    return fallback

wp2_x, wp2_y = get_coords('WP2', (0.1, 1.2))
wp3_x, wp3_y = get_coords('WP3', (0.1, -1.2))

# Generate and print standalone .tikz file
generate_trajectory_tikz(
    df_mocap=df_mocap_sweep,
    wps=np.array([[0.0, 1.2], [0.1, 1.2], [0.1, -1.2], [0.0, -1.2]]),
    target_flight_path=os.path.dirname(tikz_flight_path),
    flight_folder_name=TIKZ_FLIGHT_NAME,
    column_x=0.408,
    column_y=0.358,
    column_radius=column_radius,
    cage_radius=cage_radius,
    closest_t=metrics['closest_t'],
    closest_x=df_mocap.iloc[np.argmin(np.sqrt((df_mocap['x']-0.408)**2 + (df_mocap['y']-0.358)**2))]['x'],
    closest_y=df_mocap.iloc[np.argmin(np.sqrt((df_mocap['x']-0.408)**2 + (df_mocap['y']-0.358)**2))]['y'],
    closest_clearance=metrics['closest_clearance'],
    wp2_x=wp2_x,
    wp2_y=wp2_y,
    wp3_x=wp3_x,
    wp3_y=wp3_y
)
