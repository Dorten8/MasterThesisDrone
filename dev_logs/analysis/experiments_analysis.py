#!/usr/bin/env python
# coding: utf-8

# # 📊 Thesis Flight Telemetry & Kinetic Analysis Pipeline
# **Author:** Dorten  
# **Context:** Master's Thesis — Autonomous Obstacle Sweep & Collision Experiments  
# 
# This notebook serves as our **dynamic, thesis-grade data science interface**. It automatically parses high-frequency ROS 2 MCAP bags, extracts OptiTrack Ground Truth tracking data, and aligns coordinate transforms. 
# 
# <details>
# <summary><b>📐 Mathematical Signal Processing Rationale (Click to expand)</b></summary>
# 
# 
# #### 1. The Finite-Difference Noise Amplification Trap
# A naive numerical derivative of a discretely sampled position signal $x(t)$ is given by:
# $$v_{\text{raw}}(t_i) = \frac{x(t_{i+1}) - x(t_i)}{\Delta t}$$
# If our OptiTrack MoCap coordinates are measured at $100\text{ Hz}$ with a tiny spatial noise component (jitter) $e_i \sim \mathcal{N}(0, \sigma^2)$ where $\sigma \approx 1.5\text{ mm}$, the noise in the velocity estimate becomes:
# $$\text{Noise}[v_{\text{raw}}] \approx \frac{\sqrt{2}\sigma}{\Delta t}$$
# For $\Delta t = 0.01\text{ s}$ ($100\text{ Hz}$), the velocity noise standard deviation is scaled up by a factor of **141**, generating a massive high-frequency noise floor (approx. $\pm 0.2\text{ m/s}$ velocity spikes on a $0.4\text{ m/s}$ signal) that renders the raw derivative mathematically useless for thesis kinetic analysis.
# 
# #### 2. The Savitzky-Golay Solution
# To resolve this, we employ a **Savitzky-Golay filter**. Rather than smoothing the positions and *then* taking the derivative, Savitzky-Golay performs a local least-squares polynomial fit of degree $p$ over a moving window of size $M$ (where $M$ is odd):
# $$x(t) \approx a_0 + a_1 (t - t_0) + a_2 (t - t_0)^2 + \dots + a_p (t - t_0)^p$$
# The analytical first derivative is then computed directly from the fitted polynomial coefficients at the center of the window:
# $$v_{\text{filtered}}(t_0) = a_1$$
# This preserves local dynamic features (such as sharp acceleration profiles) far better than moving averages or low-pass Butterworth filters, which tend to distort signal peaks and introduce artificial time-delays.
# </details>

# ### 🛠️ 1. SETUP & INITIALIZATION
# Configure imports, set a consistent publication-grade plot styling SSoT, and invoke the Jupyter module cache-clearing guard to ensure changes in our package are updated instantly.

# In[1]:


import sys
import os
import matplotlib.pyplot as plt
import numpy as np

# Resolve relative directories based on project root layout
project_root = os.path.dirname(os.path.dirname(os.path.abspath('__file__' if '__file__' in locals() else os.getcwd())))
analysis_dir = os.path.join(project_root, "dev_logs", "analysis")

if analysis_dir not in sys.path:
    sys.path.insert(0, analysis_dir)

# 🛑 SELF-HEALING RELOAD GUARD (Clears Jupyter memory cache of our custom module)
for key in list(sys.modules.keys()):
    if key.startswith('experiments_analysis') or key == 'graphics':
        del sys.modules[key]

from experiments_analysis import run, compare_all_angles

# Set elegant, professional plotting style for the thesis manuscript
plt.rcParams.update({
    'font.family': 'sans-serif',
    'font.size': 11,
    'axes.labelsize': 12,
    'axes.titlesize': 13,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 10,
    'figure.titlesize': 14,
    'grid.color': '#DDDDDD',
    'grid.linestyle': '--',
    'grid.linewidth': 0.7,
    'axes.grid': True
})

print("✅ Universal Modular Analysis Pipeline Active.")
print(f"📌 Project Workspace: {project_root}")


# ### 📐 2. 90° SWEEP COLLISION EXPERIMENTS (5x Passes With vs. Without Cage)
# This runs the complete analytics and visualization pipeline for the 90° collision sweep experiments.
# It automatically loads all passes, prints a detailed events table, and renders top-down trajectories, velocity profiles, battery sag, and comparative boxplots.

# In[2]:


results_90 = run(
    label="90° Column Collision",
    angle_deg=90,
    column_x=0.500,
    column_y=0.000,
    flights_rotating_cage=[
        "flight_20260526-0931_75°_column_collision_loop_rotating_cage - Pass-04"
    ],
    flights_fixed_cage=[],
    representative_rotating_cage=0,       # Use the 1638 flight as the example
    representative_fixed_cage=0,
    project_root=project_root
)


# from graphics import generate_trajectory_tikz
# from experiments_analysis.exa_loader import load_drone_metadata, load_mcap, build_dataframes
# from experiments_analysis.exa_kinematics import compute_velocity, find_waypoint_events, calculate_metrics
# 
# # Define which experiment and flight path you want to build the LaTeX figure for
# TIKZ_FLIGHT_NAME = "flight_20260522-1638_column_sweep_loop_hardcoded_rotating_cage"
# tikz_flight_path = os.path.join(project_root, "dev_logs", "flights", TIKZ_FLIGHT_NAME)
# 
# print(f"🔮 Generating TikZ vector code for: {TIKZ_FLIGHT_NAME}")
# 
# # Load configs and data
# drone_tracker_name, system_config = load_drone_metadata(project_root)
# primary_body = next((b for b in system_config.get("tracked_bodies", []) if b.get("role") == "primary"), {})
# obstacle_body = next((b for b in system_config.get("tracked_bodies", []) if b.get("role") == "obstacle"), {})
# 
# cage_diameter = primary_body.get("cage_diameter_m", 0.358)
# column_diameter = obstacle_body.get("diameter_m", 0.09)
# cage_radius = cage_diameter / 2.0
# column_radius = column_diameter / 2.0
# 
# topic_data, bag_start_ns = load_mcap(tikz_flight_path)
# dfs = build_dataframes(topic_data, drone_tracker_name, bag_start_ns)
# df_mocap = compute_velocity(dfs['mocap'])
# 
# takeoff_mask = df_mocap['z'] > 0.15
# takeoff_time = df_mocap.loc[takeoff_mask, 't'].iloc[0] if takeoff_mask.any() else dfs['arming_time'] + 2.0
# wp_events = find_waypoint_events(df_mocap, takeoff_time)
# 
# metrics = calculate_metrics(df_mocap, wp_events, 0.500, 0.000, column_radius, cage_radius)
# 
# # Extract sweep segment
# t_start = wp_events.get('WP1', df_mocap['t'].iloc[0])
# t_end = wp_events.get('WP4', df_mocap['t'].iloc[-1])
# df_mocap_sweep = df_mocap[(df_mocap['t'] >= t_start) & (df_mocap['t'] <= t_end)]
# 
# # Query positions at WP2 and WP3 transitions
# def get_coords(wp_name, fallback):
#     t_wp = wp_events.get(wp_name)
#     if t_wp is not None:
#         idx = np.searchsorted(df_mocap['t'], t_wp)
#         idx = min(max(0, idx), len(df_mocap) - 1)
#         return df_mocap['x'].iloc[idx], df_mocap['y'].iloc[idx]
#     return fallback
# 
# wp2_x, wp2_y = get_coords('WP2', (0.1, 1.2))
# wp3_x, wp3_y = get_coords('WP3', (0.1, -1.2))
# 
# # Generate and print standalone .tikz file
# generate_trajectory_tikz(
#     df_mocap=df_mocap_sweep,
#     wps=np.array([[0.0, 1.2], [0.1, 1.2], [0.1, -1.2], [0.0, -1.2]]),
#     target_flight_path=tikz_flight_path,
#     flight_folder_name=TIKZ_FLIGHT_NAME,
#     column_x=0.500,
#     column_y=0.000,
#     column_radius=column_radius,
#     cage_radius=cage_radius,
#     closest_t=metrics['closest_t'],
#     closest_x=df_mocap.iloc[np.argmin(np.sqrt((df_mocap['x']-0.5)**2 + df_mocap['y']**2))]['x'],
#     closest_y=df_mocap.iloc[np.argmin(np.sqrt((df_mocap['x']-0.5)**2 + df_mocap['y']**2))]['y'],
#     closest_clearance=metrics['closest_clearance'],
#     wp2_x=wp2_x,
#     wp2_y=wp2_y,
#     wp3_x=wp3_x,
#     wp3_y=wp3_y
# )
# 

# In[3]:


# ==============================================================================
# 75-degree Sweep Collision Experiments
# ==============================================================================
results_75 = run(
    label="75° Column Collision",
    angle_deg=75,
    column_x=0.408, column_y=0.358,
    flights_rotating_cage=[
        "flight_20260526-0931_75°_column_collision_loop_rotating_cage - Pass-04",
        "flight_20260524-0928_75°_column_collision_loop_rotating_cage"
    ],
    flights_fixed_cage=[
        "flight_20260525-1716_75°_column_collision_loop_fixed_cage - Pass-05",
        "flight_20260524-1007_75°_column_collision_loop_rotating_cage"
    ],
    representative_rotating_cage=0,       # Use 1143 (completed full sweep collision loop)
    representative_fixed_cage=0,    # Use 1136 (completed full sweep baseline loop)
    project_root=project_root
)


# ### 📈 4. CROSS-ANGLE SCIENTIFIC METRICS TREND COMPARISON
# This function takes all computed per-angle results dictionaries and compares them side-by-side. It generates clear line trend plots showing clearances, velocities, and tracking errors, which directly form the primary finding of your thesis.

# In[4]:


# Re-run statistics across all processed angles. 
# As you populate more angles, simply add their result variables to this list: 
# e.g. compare_all_angles([results_90, results_75, results_60, ...])

compare_all_angles([results_90, results_75])


# ### ✍️ 5. AUTOMATIC LATEX PGF/TIKZ VECTOR GRAPHICS CODE GENERATOR
# Generate publication-grade TikZ code for a specific representative flight to paste directly into your Overleaf manuscript. It draws grids, waypoints, obstacles, MoCap paths, and scaled vector CAD drone representations.

# In[5]:


from graphics import generate_trajectory_tikz
from experiments_analysis.exa_loader import load_drone_metadata, load_mcap, build_dataframes
from experiments_analysis.exa_kinematics import compute_velocity, find_waypoint_events, calculate_metrics

# Define which experiment and flight path you want to build the LaTeX figure for
TIKZ_FLIGHT_NAME = "flight_20260526-0931_75°_column_collision_loop_rotating_cage"
tikz_flight_path = os.path.join(project_root, "dev_logs", "flights", TIKZ_FLIGHT_NAME)

print(f"🔮 Generating TikZ vector code for: {TIKZ_FLIGHT_NAME}")

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
df_mocap = compute_velocity(dfs['mocap'])

takeoff_mask = df_mocap['z'] > 0.15
takeoff_time = df_mocap.loc[takeoff_mask, 't'].iloc[0] if takeoff_mask.any() else dfs['arming_time'] + 2.0
df_setpoint = dfs.get('setpoint', __import__('pandas').DataFrame())
dynamic_waypoints = dfs.get('dynamic_waypoints', [])
wp_events = find_waypoint_events(df_mocap, df_setpoint, takeoff_time, return_all=True, dynamic_waypoints=dynamic_waypoints)[0]

metrics = calculate_metrics(df_mocap, wp_events, 0.500, 0.000, column_radius, cage_radius)

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
    target_flight_path=tikz_flight_path,
    flight_folder_name=TIKZ_FLIGHT_NAME,
    column_x=0.500,
    column_y=0.000,
    column_radius=column_radius,
    cage_radius=cage_radius,
    closest_t=metrics['closest_t'],
    closest_x=df_mocap.iloc[np.argmin(np.sqrt((df_mocap['x']-0.5)**2 + df_mocap['y']**2))]['x'],
    closest_y=df_mocap.iloc[np.argmin(np.sqrt((df_mocap['x']-0.5)**2 + df_mocap['y']**2))]['y'],
    closest_clearance=metrics['closest_clearance'],
    wp2_x=wp2_x,
    wp2_y=wp2_y,
    wp3_x=wp3_x,
    wp3_y=wp3_y
)

