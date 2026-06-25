import json
import os

ipynb_path = "/home/dorten/pi_drone_sshfs/dev_logs/analysis/diagnostics.ipynb"

# Rebuild cells with a centralized SELECTED_FLIGHT loader at the top
cells = [
    {
        "cell_type": "markdown",
        "id": "intro",
        "metadata": {},
        "source": [
            "# 📊 Thesis Flight Diagnostics & Envelope Audits\n",
            "\n",
            "This serves as your **dynamic thesis flight dashboard**. To analyze a specific flight, simply specify the folder name in the **Flight Selection** loader cell below. All subsequent diagnostics cells will automatically parse and plot that selected flight's telemetry!"
        ]
    },
    {
        "cell_type": "code",
        "execution_count": None,
        "id": "loader_cell",
        "metadata": {},
        "outputs": [],
        "source": [
            "# ⚙️ CENTRAL FLIGHT LOADER\n",
            "# Set SELECTED_FLIGHT to None to automatically load the LATEST flight,\n",
            "# or enter a specific folder name to load a previous test flight.\n",
            "SELECTED_FLIGHT = \"flight_20260524-1143_75°_column_collision_loop_rotating_cage\""
        ]
    },
    {
        "cell_type": "markdown",
        "id": "h1",
        "metadata": {},
        "source": [
            "## 🔋 1. Dynamic Flight Envelope & Battery Sag Analysis"
        ]
    },
    {
        "cell_type": "code",
        "execution_count": None,
        "id": "cell1",
        "metadata": {},
        "outputs": [],
        "source": [
            "from diagnostics import analyze_flight\n",
            "\n",
            "# Analyze MoCap tracking errors, active flight bounds, and dynamic battery sag profiles under motor loading\n",
            "analyze_flight(target=SELECTED_FLIGHT)"
        ]
    },
    {
        "cell_type": "markdown",
        "id": "h2",
        "metadata": {},
        "source": [
            "## 🔬 2. High-Frequency Telemetry & EKF2 Fusion Rejection Dissection"
        ]
    },
    {
        "cell_type": "code",
        "execution_count": None,
        "id": "cell2",
        "metadata": {},
        "outputs": [],
        "source": [
            "import os\n",
            "from diagnostics import analyze_deep_dissection\n",
            "\n",
            "# Resolve the path for deep dissection from the selected flight\n",
            "if SELECTED_FLIGHT:\n",
            "    bag_path = f\"/home/dorten/pi_drone_sshfs/dev_logs/flights/{SELECTED_FLIGHT}/{SELECTED_FLIGHT}_0.mcap\"\n",
            "else:\n",
            "    bag_path = None\n",
            "    \n",
            "analyze_deep_dissection(bag_path=bag_path)"
        ]
    },
    {
        "cell_type": "markdown",
        "id": "h3",
        "metadata": {},
        "source": [
            "## 📐 3. High-Fidelity Velocity Profiles & EKF2 Convergence Timeline\n",
            "\n",
            "Visualizes the dual-subplot **smoothed flight velocity magnitude** and **instantaneous /poses stream rate (Hz)**, and displays a timeline auditing **EKF2 absolute coordinate convergence**."
        ]
    },
    {
        "cell_type": "code",
        "execution_count": None,
        "id": "cell3",
        "metadata": {},
        "outputs": [],
        "source": [
            "import os\n",
            "import sys\n",
            "import pandas as pd\n",
            "import numpy as np\n",
            "import matplotlib.pyplot as plt\n",
            "\n",
            "project_root = \"/home/dorten/pi_drone_sshfs\"\n",
            "sys.path.append(os.path.join(project_root, \"dev_logs\", \"analysis\"))\n",
            "\n",
            "from dev_logs.analysis.database.db_loader import load_mcap, build_dataframes, load_drone_metadata\n",
            "from dev_logs.analysis.kinematics.kin_calculator import compute_mocap_kinematics, find_waypoint_events, build_events_log\n",
            "from dev_logs.analysis.kinematics.kin_plot_kinematics import plot_velocity_profile, plot_imu_dynamics, plot_imu_xyz_components\n",
            "\n",
            "# Resolve the active flight folder dynamically\n",
            "if SELECTED_FLIGHT:\n",
            "    active_flight = SELECTED_FLIGHT\n",
            "else:\n",
            "    import glob\n",
            "    flight_folders = glob.glob(\"/home/dorten/pi_drone_sshfs/dev_logs/flights/flight_*\")\n",
            "    flight_folders = [d for d in flight_folders if os.path.isdir(d) and \"px4_sd_logs\" not in d]\n",
            "    active_flight = os.path.basename(max(flight_folders, key=os.path.getmtime)) if flight_folders else None\n",
            "\n",
            "flight_path = os.path.join(project_root, \"dev_logs\", \"flights\", active_flight)\n",
            "print(f\"🎬 Performing High-Fidelity Kinematic Diagnostics on: {active_flight}\\n\")\n",
            "\n",
            "# Load telemetry dataframes\n",
            "topic_data, bag_start_ns = load_mcap(flight_path)\n",
            "drone_tracker_name, _ = load_drone_metadata(project_root)\n",
            "dfs = build_dataframes(topic_data, drone_tracker_name, bag_start_ns)\n",
            "\n",
            "df_mocap = dfs['mocap']\n",
            "df_odom = dfs['odom']\n",
            "df_setpoint = dfs['setpoint']\n",
            "df_bat = dfs['battery']\n",
            "df_imu = dfs['imu']\n",
            "arming_time = dfs['arming_time']\n",
            "disarming_time = dfs['disarming_time']\n",
            "\n",
            "# Process kinematics\n",
            "df_mocap = compute_mocap_kinematics(df_mocap)\n",
            "\n",
            "# Detect Takeoff Trigger (using MoCap Z height crossing 0.15m)\n",
            "takeoff_mask = df_mocap['z'] > 0.15 if not df_mocap.empty else pd.Series(dtype=bool)\n",
            "takeoff_time = df_mocap.loc[takeoff_mask, 't'].iloc[0] if takeoff_mask.any() else arming_time + 2.0\n",
            "\n",
            "# Detect waypoint events dynamically loading from mission definition!\n",
            "wp_events = find_waypoint_events(df_mocap, takeoff_time, label=active_flight)\n",
            "\n",
            "# Build dynamic events log\n",
            "events = build_events_log(df_mocap, df_bat, arming_time, takeoff_time, disarming_time, wp_events)\n",
            "\n",
            "# Plot Velocity Profile & MoCap /poses Instantaneous Publishing Rate\n",
            "plot_velocity_profile(df_mocap, wp_events, arming_time, takeoff_time, disarming_time, events, label=\"Selected Experiment\")\n",
            "\n",
            "# Plot Physical IMU Impact Dynamics\n",
            "plot_imu_dynamics(df_imu, wp_events, arming_time, takeoff_time, disarming_time, events, label=\"Selected Experiment\")\n",
            "plot_imu_xyz_components(df_imu, wp_events, arming_time, takeoff_time, disarming_time, events, label=\"Selected Experiment\")\n",
            "\n",
            "# Align EKF2 and MoCap coordinate timeline to audit convergence\n",
            "merged = pd.merge_asof(df_mocap.sort_values('t'), df_odom.sort_values('t'), on='t', direction='nearest')\n",
            "merged = pd.merge_asof(merged, df_setpoint.sort_values('t'), on='t', direction='nearest')\n",
            "\n",
            "merged['dx_enu'] = merged['x_ekf'] - merged['x']\n",
            "merged['dy_enu'] = merged['y_ekf'] - merged['y']\n",
            "\n",
            "print(\"\\n\" + \"=\"*70)\n",
            "print(\"⚖️ EKF2 ESTIMATOR CONVERGENCE TIMELINE AUDIT\")\n",
            "print(\"=\"*70)\n",
            "timeline_times = [0.0, arming_time, arming_time + 2.0, arming_time + 5.0, arming_time + 10.0, arming_time + 20.0, arming_time + 30.0]\n",
            "for t in timeline_times:\n",
            "    if t > merged['t'].max():\n",
            "        continue\n",
            "    row = merged.loc[(merged['t'] - t).abs().idxmin()]\n",
            "    print(f\"  t={row['t']-arming_time:>5.2f}s since arming | MoCap ENU: ({row['x']:.3f}, {row['y']:.3f}) | EKF2 ENU: ({row['x_ekf']:.3f}, {row['y_ekf']:.3f}) | EKF2 Offset: ({row['dx_enu']:.3f}, {row['dy_enu']:.3f})\")\n",
            "print(\"=\"*70 + \"\\n\")"
        ]
    }
]

# Write to file
with open(ipynb_path, 'r') as f:
    orig_data = json.load(f)

orig_data['cells'] = cells

with open(ipynb_path, 'w') as f:
    json.dump(orig_data, f, indent=1)

print("SUCCESS: Programmatically updated diagnostics.ipynb to include plot_imu_xyz_components!")
