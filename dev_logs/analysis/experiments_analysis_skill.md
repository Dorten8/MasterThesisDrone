# Experiments Analysis Pipeline - Skill & Cookbook

This document serves as the Single Source of Truth (SSoT) for the telemetry, kinematics, and database analysis pipeline. It dictates the required directory architecture, code standards, data processing rules, and specific formatting for all generated plots and data access.

## What is the purpose of these experiments?
These physical experiments involve flying a PX4-controlled quadcopter into a static column obstacle to study physical collision dynamics and recovery.
* **Rotational Inertia & Stabilisation:** We analyze how the drone reacts to column contact across two principal safety cage configurations:
  * **Rotating Cage:** The outer protective cage is free to rotate around the drone's physical Z-axis.
  * **Fixed Cage:** The protective cage is rigidly fixed to the vehicle frame (no Z-axis rotation).
* **IMU Impact Vector Estimation:** We investigate whether structural accelerometer and gyroscope signatures can reliably predict the physical contact vector compared to high-fidelity OptiTrack Motion Capture (MoCap) ground truth.

---

# 📐 Modular Workspace Architecture

The workspace is strictly partitioned into specialized sub-packages under the root `dev_logs/analysis/` package to ensure clean separation of concerns:

```
dev_logs/analysis/
├── __init__.py                # Root package initialization
├── database/                  # Ingestion, Slicing, Caching, and SQLite managers
│   ├── __init__.py            # Exposes: run, get_database_df
│   ├── db_loader.py           # Multi-topic MCAP parser and temporal EKF alignment
│   ├── db_manager.py          # SQLite database schema, caching, and pandas query helpers
│   ├── db_pipeline.py         # Main analytical ingestion and tabular box-metric manager
│   ├── db_mcap_event_segmenter.py  # Event-driven flight slicer using waypoint triggers
│   └── db_mcap_legacy_segmenter.py # Legacy waypoint coordinate-based slicer
│
├── kinematics/                # Flight physics, deviations, and visualization
│   ├── __init__.py            # Exposes: compare_all_angles
│   ├── kin_calculator.py      # Core physical equations (forces, angles, deviations)
│   ├── kin_plot_trajectory.py # Top-down 2D spatial CAD drone path plotting
│   ├── kin_plot_kinematics.py # Tangential acceleration, velocity, and raw IMU plots
│   └── kin_plot_statistics.py # Comparative statistics (box/bar charts)
│
├── diagnostics/               # Notebook rebuilding and parsing tests
│   └── dia_pipeline_rebuild.py # Automated Jupyter Notebook rebuilder
│
├── bin/                       # Executable utility shells
├── graphics/                  # Output plot directory (PNG, SVG)
├── dev_logs/scratch/          # Root-level central sandbox folder for temporary scripts
└── experiments_summary.db     # SQLite Single Source of Truth database
```

### Critical Workspace & Scratchpad Rules:
1. **Never use local scratch folders:** All temporary playground or test scripts (e.g. `scratch_*.py`) MUST live in the root `/dev_logs/scratch/` directory. No scratch files are allowed inside `/database/`, `/kinematics/`, or `/diagnostics/`.
2. **Expose package-level functions:** Any shared API functions (like `run`, `get_database_df`, `compare_all_angles`) must be imported and exposed in their package `__init__.py` files so they can be neatly accessed.

---

# General Plot Rules & File Naming

These rules apply universally to all visualizations within the pipeline:

1. **SSHFS-Safe Plot Filenames:** Due to character encoding and metadata caching issues over network mounts (SSHFS), all output plot filenames **MUST NOT** contain the degree symbol `°`.
   * **Rule:** Programmatically replace `°` with `deg` when writing files (e.g., save as `trajectory_75deg_rotating_cage.png` instead of `trajectory_75°_rotating_cage.png`).
2. **Dynamic Waypoint Ground Truth:** Waypoints MUST be dynamically extracted from the `/fmu/in/trajectory_setpoint` topic inside the `.mcap` file rather than being hardcoded. This guarantees absolute physical synchronization with what the flight controller commanded.
3. **Dynamic MoCap Publish Rate:** The nominal tracking rate (e.g., 240 Hz vs. 120 Hz) must be dynamically computed directly from the `/poses` message frequency in the `.mcap` file, ensuring no hardcoded rates are displayed in diagnostic subplots.
4. **Strict Data Truncation:** To keep plots clean and relevant, all flight data must be dynamically cropped to exactly **5 seconds before the "Exp. Start-point"** and **5 seconds after the "Exp. End-point"**. All telemetry outside this window should be discarded.
5. **Standardized Terminology SSoT:**
   * **WP1 / Entry Point** is renamed to: **`Exp. Start-point`**
   * **WP2 / WP4 / Exit Point** is renamed to: **`Exp. End-point`**
   * **Cage Condition Labels:** Update all labels, titles, and legends to strictly read **`Rotating Cage`** vs. **`Fixed Cage`** (do not use "no_cage" or "cage vs no_cage" since all test flights are flown with a physical cage).
6. **Label Origin of Data:** All figures must have a small label in the bottom-right corner indicating the origin of the data (the exact flight pass name, e.g. `flight_20260524-1813_75deg_column_collision_loop_fixed_cage - Pass-01`).
7. **Timeline Event Alignment:** Every plot that uses time ($t$) as the X-axis MUST display three perfectly aligned, vertical event marker lines across all subplots, denoting:
   * **`Exp. Start-point`** (vertical dotted purple/grey line)
   * **💥 `Impact`** (vertical dash-dotted crimson line at time of closest approach / actual impact)
   * **`Exp. End-point`** (vertical dotted purple/grey line)

---

# Plot Reference Guide & Visual Standards

Below is the definitive visual and physical specification for the five core plots in the thesis experiments pipeline.

### 1. Top-Down 2D Spatial Trajectory
* **Purpose:** Visualizes the spatial tracking path of the drone relative to the static column obstacle, illustrating lateral deviations, closest approach vector, and orientation transitions.
* **What it Shows:** 
  * The raw physical $X/Y$ coordinates tracked by MoCap.
  * The physical dimensions (circles) of both the Column and the Protective Cage.
  * The closest physical approach separation vector.
  * Shaded drone CAD vector illustrations representing physical pose and heading.
  * **Perpendicular Maximum Deviation ($d_{\text{max}}$):** An orange dashed line perpendicular to the nominal path line ($WP2 \rightarrow WP3$), indicating the peak rebound displacement.
  * **Hatched Recovery Envelope Area:** A light purple diagonal hatched region (`//`) between the actual recovery trajectory and the nominal commanded line segment after column contact.
* **Visual Standard & Requirements:**
  * Must maintain a strictly equal aspect ratio ($1:1$ physical meter mapping) to prevent skewing.
  * **Eliminate Box Outlines:** All text labels MUST use borderless white background boxes (`edgecolor='none'`, `alpha=0.9`) to prevent clipping frames or text overlaps.
  * **Subtle Leader Pointer Lines:** Use light gray indicators (`arrowprops=dict(arrowstyle="->", color='#888888', lw=0.8)`) pointing from the labels directly to their coordinate targets.
  * **Unified Multi-Line Labels:** To prevent coordinate values from overlapping headers under varying DPI settings, combine coordinates and metadata into single, multi-line strings inside a single label call.
  * **Grey shaded drones must be drawn at actual MoCap-registered positions**, not theoretical waypoints. This includes:
    1. Actual position at `Exp. Start-point` (annotated with actual MoCap $(X, Y)$ coords).
    2. Actual position at `Exp. End-point` (annotated with actual MoCap $(X, Y)$ coords).
    3. Actual position at the moment of closest approach (annotated with minimum physical clearance value in mm, impact speed, and impact acceleration).
* **Associated Script:** [kin_plot_trajectory.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/kinematics/kin_plot_trajectory.py)

### 2. Tangential Acceleration / Deceleration Profile
* **Purpose:** Displays the linear kinematics of the sweep, showing the transition from steady-state cruise speed to impact deceleration.
* **What it Shows:** 
  * Filtered tangential acceleration over time (computed by derivative filtering of velocity).
  * Positive acceleration (cruise ramp) vs. negative deceleration (braking/impact).
* **Visual Standard & Requirements:**
  * Bold horizontal steady-state reference line at $0.0\text{ m/s}^2$.
  * Color-filled zones: Green fill for acceleration, red fill for deceleration.
  * Three vertical timeline event lines marking: `Exp. Start-point`, `Impact`, and `Exp. End-point` (per General Plot Rule 7).
* **Associated Script:** [kin_plot_kinematics.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/kinematics/kin_plot_kinematics.py)

### 3. Thesis Flight Kinetic Profile (Velocity & Timeline)
* **Purpose:** Combines the smoothed velocity magnitude profile with live MoCap diagnostic streaming rates to correlate estimator/tracking state with flight behavior.
* **What it Shows:**
  * Top Panel: Velocity magnitude ($m/s$) with horizontal dashed lines showing average speed during active flight segments.
  * Bottom Panel: Live frame rate ($Hz$) of the `/poses` stream with a red dashed line marking the $30\text{Hz}$ critical tracking limit.
* **Visual Standard & Requirements:**
  * Timeline vertical event markers for `Exp. Start-point`, `Impact`, and `Exp. End-point` must align perfectly across both subplots (per General Plot Rule 7).
  * Nominal rate line must reflect the dynamically extracted MoCap rate (e.g. 240 Hz).
* **Associated Script:** [kin_plot_kinematics.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/kinematics/kin_plot_kinematics.py)

### 4. Physical IMU Collision Dynamics
* **Purpose:** Isolates the high-frequency physical impact event and the drone's immediate structural and rotational response.
* **What it Shows:**
  * **Linear Accel Deviation ($\text{m/s}^2$):** Measures the sudden peak G-forces experienced by the IMU during column contact.
  * **Gyro Rotational Surge ($\text{rad/s}$):** Displays the sudden angular velocity surge (roll/pitch/yaw perturbations) resulting from cage contact, showing the physical rebound dynamics before active FC stabilization takes over.
* **Visual Standard & Requirements:**
  * Dual Y-axes: Left (red) for linear acceleration deviation, Right (blue) for gyro rotational surge.
  * A horizontal gold dashed line marking the "Severe Impact Threshold" at $5.0\text{ m/s}^2$.
  * Three vertical timeline event lines marking: `Exp. Start-point`, `Impact`, and `Exp. End-point` (per General Plot Rule 7).
* **Associated Script:** [kin_plot_kinematics.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/kinematics/kin_plot_kinematics.py)

### 5. RAW IMU X/Y/Z Components
* **Purpose:** Breaks down the high-frequency IMU linear and angular readings along the Pixhawk 6C's principal physical body axes.
* **What it Shows:**
  * Raw/Filtered linear acceleration ($a_x, a_y, a_z$) and rotational rates ($\omega_x, \omega_y, \omega_z$).
* **Visual Standard & Requirements:**
  * Axis mappings must be professionally labeled to explain Pixhawk 6C axes:
    * **X-Axis (Lateral / Roll):** Side-to-side dynamics.
    * **Y-Axis (Longitudinal / Pitch):** Fore-and-aft dynamics along the main flight vector.
    * **Z-Axis (Vertical / Yaw / Heave):** Up-and-down dynamics and heading rotations.
  * Three vertical timeline event lines marking: `Exp. Start-point`, `Impact`, and `Exp. End-point` (per General Plot Rule 7).
* **Associated Script:** [kin_plot_kinematics.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/kinematics/kin_plot_kinematics.py)

---

# Fully Implemented & Validated Features

The following architectural, kinematics, and visual features have been fully implemented and verified:

### 1. Robust EKF-to-MoCap Coordinate Alignment (`db_loader.py`)
* **Dynamic Offset Estimation:** Rather than relying on static coordinate offsets, the pipeline dynamically merges `df_mocap` and `df_odom` using `pd.merge_asof` during dataframe loading. It estimates the exact $X$ and $Y$ EKF-to-MoCap coordinate alignment translation offsets using:
  $$\text{offset}_x = \text{median}(x_{ekf} - x)$$
  $$\text{offset}_y = \text{median}(-y_{ekf} + y)$$
* **Setpoint Restoring:** Restores raw NED trajectory setpoints back into their native MoCap ENU frame targets using a mathematically exact inverse mapping:
  $$x_{cmd} = \text{position}[0] - \text{offset}_x$$
  $$y_{cmd} = -(\text{position}[1] - \text{offset}_y)$$
  This cleanly resolves the coordinate double-swap bug and aligns setpoint commands perfectly with flight tracking!

### 2. Mission-Class Waypoint Mapping — Absolute SSoT (`kin_calculator.py`)
* **Mission SSoT Waypoints:** The pipeline reads the 4 nominal waypoints directly from the active mission class (`ExpCollision75Deg`, `ExpCollision45Deg`, or `ColumnSweepLoop`). This is the **absolute first priority** — the mission file defines the exact geometry.
* **Why not dynamic extraction?** Segmented pass `.mcap` files contain PX4 trajectory interpolation commands (15+ intermediate setpoints every ~150mm), not the real 4 mission waypoints. Using these produces completely wrong sweep boundaries and cascading clearance/impact errors.

### 3. Timeline Event Alignment
* **Axvline Markers:** All time-domain plots strictly overlay three perfectly synchronized vertical line markers denoting `Exp. Start-point`, `Impact`, and `Exp. End-point` boundaries, cropping the display limit to the exact $T_{start}-5s$ to $T_{end}+5s$ window.

---

# 💾 SQLite Experiments Database Schema

To enable rigorous statistical comparison and thesis compilation, all processed flight passes automatically cache their calculated physical metrics inside a shared SQLite database located at `dev_logs/analysis/experiments_summary.db`.

### Resilient SSHFS Caching & Locking
Because we develop remotely over network mounts (SSHFS), POSIX advisory file locks are highly unstable. To prevent write corruption and database locks, the pipeline implements:
1. **resilient connections:** All SQLite accesses use a connection timeout of **30 seconds** (`timeout=30.0`).
2. **busy timeouts:** Executes `PRAGMA busy_timeout = 30000;` on connection to gracefully queue transactions rather than throwing immediate crash exceptions.

### Table Schema: `flights_summary`

| Column Name | Data Type | Units / Range | Description |
| :--- | :--- | :--- | :--- |
| **`flight_name`** | TEXT (Primary Key) | String | Consolidated pass name: `<flight_folder_name> - Pass-<Index>` |
| **`condition`** | TEXT | `'Rotating Cage'` or `'Fixed Cage'` | Safety cage rotational configuration condition. |
| **`sweep_speed`** | REAL | $\text{m/s}$ | Declared sweep speed from mission SSoT. |
| **`battery_at_start`** | REAL | $\%$ | Battery remaining percentage at experiment start. |
| **`impact_speed`** | REAL | $\text{m/s}$ | Linear speed magnitude at closest approach to obstacle. |
| **`before_impact_accel`** | REAL | $\text{m/s}^2$ | Average tangential acceleration in window $[t_{closest} - 0.4s, t_{closest} - 0.2s]$. |
| **`impact_accel`** | REAL | $\text{m/s}^2$ | Tangential acceleration rate at closest approach. |
| **`impact_angle`** | REAL | degrees ($0^\circ - 90^\circ$) | 2D spatial contact angle normal to the column surface. |
| **`avg_dev_after`** | REAL | $\text{mm}$ | Mean perpendicular trajectory deviation along recovery segment. |
| **`max_dev_after`** | REAL | $\text{mm}$ | Peak perpendicular trajectory deviation along recovery segment. |
| **`recovery_area`** | REAL | $\text{cm}^2$ | Integrated Spatial Integral of Absolute Error (SIAE) recovery envelope. |
| **`closest_clearance`** | REAL | $\text{cm}$ | Minimum spatial clearance between outer cage surface and column. Negative = physical contact. |
| **`impact_detected`** | INTEGER | `0` or `1` | Impact classification: `1` if `closest_clearance < 0` (cage penetrated column boundary). |
| **`nom_sp_x/y/z`** | REAL | $\text{m}$ | Command/Target nominal experiment start-point coordinates from mission SSoT. |
| **`act_sp_x/y/z`** | REAL | $\text{m}$ | Actual physical vehicle coordinates from OptiTrack at experiment entry timestamp. |
| **`nom_ep_x/y/z`** | REAL | $\text{m}$ | Command/Target nominal experiment end-point coordinates from mission SSoT. |
| **`act_ep_x/y/z`** | REAL | $\text{m}$ | Actual physical vehicle coordinates from OptiTrack at experiment exit timestamp. |
| **`timestamp`** | TEXT | `YYYY-MM-DD HH:MM:SS` | Caching timestamp. |

---

# 🚀 Programmatic Notebook Querying: `get_database_df()`

You should **never** open a raw `.db` file in an external database explorer over SSHFS, as this causes metadata page corruption and locking loops. Instead, **query the database directly inside your Jupyter Notebooks as a standard pandas DataFrame**!

### Load & View Database
```python
from dev_logs.analysis.database import get_database_df

# Fetch the entire SQLite cache into an in-memory Pandas DataFrame
df = get_database_df()

# Display the first 5 records in a beautiful notebook table
df.head()
```

### Common Notebook Queries

#### 1. Find all flights where a collision occurred
```python
collisions = df[df['impact_detected'] == 1]
collisions[['flight_name', 'condition', 'sweep_speed', 'impact_speed']]
```

#### 2. Query specific conditions and speeds
```python
# Select only fast rotating cage passes
fast_rotating = df[(df['condition'] == 'Rotating Cage') & (df['sweep_speed'] > 0.25)]
```

#### 3. Export to Excel instantly
```python
# Creates an Excel sheet safely without touching SQL locks
df.to_excel("experiments_summary.xlsx", index=False)
```

---

# ✂️ MCAP Pass Segmenter Tool

The MCAP segmenter is a standalone tool that physically slices a full raw flight recording into separate, pass-level `-passXX.mcap` slice files immediately after landing.

**Script Location:** [`database/db_mcap_event_segmenter.py`](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/database/db_mcap_event_segmenter.py)

### What It Does
1. Loads the primary `.mcap` from the flight folder (ignores any existing `-passXX.mcap` slice files automatically).
2. Runs waypoint-event boundary detection to find the exact start/end timestamps of each sweep pass ($WP2 \rightarrow WP3$).
3. Slices all ROS 2 messages within the detected window (plus a configurable padding, default `±2s`) into a new standalone bag.
4. Saves each slice as `<original_bag_name>-pass01.mcap`, `-pass02.mcap`, etc. **inside the same flight folder**.

### Running the Segmenter
```bash
# Run on all approved flights from the repository root:
PYTHONPATH=. python3 dev_logs/analysis/database/db_mcap_event_segmenter.py
```
