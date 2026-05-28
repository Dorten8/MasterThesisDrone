# Experiments Analysis Pipeline - Skill & Cookbook

This document serves as the Single Source of Truth (SSoT) for the `experiments_analysis` pipeline. It dictates the required architecture, data processing steps, and specific formatting for all generated plots.

## What is it the purpose of these experiments?
These experiments are about colliding with a PX4 controlled drone into a static column. And seeing: 
-   How does the drone react to the collision (in terms of velocity, angular velocity, acceleration, etc.)?
-   Can we establish a model which could inform us at what vector did the drone impact its obstacle by getting information only from the IMU? We know the velocity vector just before impact from MoCap as ground truth so based on that we can see whether we could estimate the impact vector only from the IMU. 

There are 2 conditions: 
-   Rotating cage: The cage is free to rotate around the Z axis of the drone
-   Fixed cage: The cage is fixed to a frame, so it doesn't rotate around the Z axis of the drone.

---

# General plot rules

These rules apply universally to all visualizations within the pipeline:

1.  **Dynamic Waypoint Ground Truth:** Waypoints MUST be dynamically extracted from the `/fmu/in/trajectory_setpoint` topic inside the `.mcap` file rather than being hardcoded. This guarantees absolute physical synchronization with what the flight controller commanded.
2.  **Dynamic MoCap Publish Rate:** The nominal tracking rate (e.g., 240 Hz vs. 120 Hz) must be dynamically computed directly from the `/poses` message frequency in the `.mcap` file, ensuring no hardcoded rates are displayed in diagnostic subplots.
3.  **Strict Data Truncation:** To keep plots clean and relevant, all flight data must be dynamically cropped to exactly **5 seconds before the "Exp. Start-point"** and **5 seconds after the "Exp. End-point"**. All telemetry outside this window should be discarded.
4.  **Standardized Terminology SSoT:**
    *   **WP1 / Entry Point** is renamed to: **`Exp. Start-point`**
    *   **WP2 / WP4 / Exit Point** is renamed to: **`Exp. End-point`**
    *   **Cage Condition Labels:** Update all labels, titles, and legends to strictly read **`Rotating Cage`** vs. **`Fixed Cage`** (do not use "no_cage" or "cage vs no_cage" since all test flights are flown with a physical cage).
5.  **Label Origin of Data:** All figures must have a small label in the bottom-right corner indicating the origin of the data (the exact flight pass name, e.g. `flight_20260524-1813_75°_column_collision_loop_fixed_cage - Pass-01`).
6.  **Timeline Event Alignment:** Every plot that uses time ($t$) as the X-axis MUST display three perfectly aligned, vertical event marker lines across all subplots, denoting:
    *   **`Exp. Start-point`** (vertical dotted purple/grey line)
    *   **💥 `Impact`** (vertical dash-dotted crimson line at time of closest approach / actual impact)
    *   **`Exp. End-point`** (vertical dotted purple/grey line)

---

# Plot Reference Guide & Visual Standards

Below is the definitive visual and physical specification for the five core plots in the thesis experiments pipeline.

### 1. Top-Down 2D Spatial Trajectory
*   **Purpose:** Visualizes the spatial tracking path of the drone relative to the static column obstacle, illustrating lateral deviations, closest approach vector, and orientation transitions.
*   **What it Shows:** 
    *   The raw physical $X/Y$ coordinates tracked by MoCap.
    *   The physical dimensions (circles) of both the Column and the Protective Cage.
    *   The closest physical approach separation vector.
    *   Shaded drone CAD vector illustrations representing physical pose and heading.
    *   **Perpendicular Maximum Deviation ($d_{\text{max}}$):** An orange dashed line perpendicular to the nominal path line ($WP2 \rightarrow WP3$), indicating the peak rebound displacement.
    *   **Hatched Recovery Envelope Area:** A light purple diagonal hatched region (`//`) between the actual recovery trajectory and the nominal commanded line segment after column contact.
*   **Visual Standard & Requirements:**
    *   Must maintain a strictly equal aspect ratio ($1:1$ physical meter mapping) to prevent skewing.
    *   **Eliminate Box Outlines:** All text labels MUST use borderless white background boxes (`edgecolor='none'`, `alpha=0.9`) to prevent clipping frames or text overlaps.
    *   **Subtle Leader Pointer Lines:** Use light gray indicators (`arrowprops=dict(arrowstyle="->", color='#888888', lw=0.8)`) pointing from the labels directly to their coordinate targets.
    *   **Unified Multi-Line Labels:** To prevent coordinate values from overlapping headers under varying DPI settings, combine coordinates and metadata into single, multi-line strings inside a single label call.
    *   **Grey shaded drones must be drawn at actual MoCap-registered positions**, not theoretical waypoints. This includes:
        1.  Actual position at `Exp. Start-point` (annotated with actual MoCap $(X, Y)$ coords).
        2.  Actual position at `Exp. End-point` (annotated with actual MoCap $(X, Y)$ coords).
        3.  Actual position at the moment of closest approach (annotated with minimum physical clearance value in mm, impact speed, and impact acceleration).
*   **Associated Script:** [exa_plot_trajectory.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/experiments_analysis/exa_plot_trajectory.py)

### 2. Tangential Acceleration / Deceleration Profile
*   **Purpose:** Displays the linear kinematics of the sweep, showing the transition from steady-state cruise speed to impact deceleration.
*   **What it Shows:** 
    *   Filtered tangential acceleration over time (computed by derivative filtering of velocity).
    *   Positive acceleration (cruise ramp) vs. negative deceleration (braking/impact).
*   **Visual Standard & Requirements:**
    *   Bold horizontal steady-state reference line at $0.0\text{ m/s}^2$.
    *   Color-filled zones: Green fill for acceleration, red fill for deceleration.
    *   Three vertical timeline event lines marking: `Exp. Start-point`, `Impact`, and `Exp. End-point` (per General Plot Rule 6).
*   **Associated Script:** [exa_plot_kinematics.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/experiments_analysis/exa_plot_kinematics.py)

### 3. Thesis Flight Kinetic Profile (Velocity & Timeline)
*   **Purpose:** Combines the smoothed velocity magnitude profile with live MoCap diagnostic streaming rates to correlate estimator/tracking state with flight behavior.
*   **What it Shows:**
    *   Top Panel: Velocity magnitude ($m/s$) with horizontal dashed lines showing average speed during active flight segments.
    *   Bottom Panel: Live frame rate ($Hz$) of the `/poses` stream with a red dashed line marking the $30\text{Hz}$ critical tracking limit.
*   **Visual Standard & Requirements:**
    *   Timeline vertical event markers for `Exp. Start-point`, `Impact`, and `Exp. End-point` must align perfectly across both subplots (per General Plot Rule 6).
    *   Nominal rate line must reflect the dynamically extracted MoCap rate (e.g. 240 Hz).
*   **Associated Script:** [exa_plot_kinematics.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/experiments_analysis/exa_plot_kinematics.py)

### 4. Physical IMU Collision Dynamics
*   **Purpose:** Isolates the high-frequency physical impact event and the drone's immediate structural and rotational response.
*   **What it Shows:**
    *   **Linear Accel Deviation ($\text{m/s}^2$):** Measures the sudden peak G-forces experienced by the IMU during column contact.
    *   **Gyro Rotational Surge ($\text{rad/s}$):** Displays the sudden angular velocity surge (roll/pitch/yaw pertubations) resulting from cage contact, showing the physical rebound dynamics before active FC stabilization takes over.
*   **Visual Standard & Requirements:**
    *   Dual Y-axes: Left (red) for linear acceleration deviation, Right (blue) for gyro rotational surge.
    *   A horizontal gold dashed line marking the "Severe Impact Threshold" at $5.0\text{ m/s}^2$.
    *   Three vertical timeline event lines marking: `Exp. Start-point`, `Impact`, and `Exp. End-point` (per General Plot Rule 6).
*   **Associated Script:** [exa_plot_kinematics.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/experiments_analysis/exa_plot_kinematics.py)

### 5. RAW IMU X/Y/Z Components
*   **Purpose:** Breaks down the high-frequency IMU linear and angular readings along the Pixhawk 6C's principal physical body axes.
*   **What it Shows:**
    *   Raw/Filtered linear acceleration ($a_x, a_y, a_z$) and rotational rates ($\omega_x, \omega_y, \omega_z$).
*   **Visual Standard & Requirements:**
    *   Axis mappings must be professionally labeled to explain Pixhawk 6C axes:
        *   **X-Axis (Lateral / Roll):** Side-to-side dynamics. (e.g. A stable gyro offset here indicates the drone's heading relative to the flight path).
        *   **Y-Axis (Longitudinal / Pitch):** Fore-and-aft dynamics along the main flight vector. (e.g. Shows steady cruise acceleration/deceleration profiles).
        *   **Z-Axis (Vertical / Yaw / Heave):** Up-and-down dynamics and heading rotations. (e.g. Captures takeoff climb, landing, and turn-around heading corrections).
    *   Three vertical timeline event lines marking: `Exp. Start-point`, `Impact`, and `Exp. End-point` (per General Plot Rule 6).
*   **Associated Script:** [exa_plot_kinematics.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/experiments_analysis/exa_plot_kinematics.py)

---

# Fully Implemented & Validated Features

The following architectural, kinematics, and visual features have been fully implemented, validated, and verified:

### 1. Robust EKF-to-MoCap Coordinate Alignment & Setpoint Restoring (`exa_loader.py`)
-   **Dynamic Offset Estimation:** Rather than relying on static coordinate offsets, the pipeline now dynamically merges `df_mocap` and `df_odom` using `pd.merge_asof` during dataframe loading. It estimates the exact $X$ and $Y$ EKF-to-MoCap coordinate alignment translation offsets using:
    $$\text{offset}_x = \text{median}(x_{ekf} - x)$$
    $$\text{offset}_y = \text{median}(-y_{ekf} + y)$$
-   **Setpoint Restoring:** Restores raw NED trajectory setpoints back into their native MoCap ENU frame targets using a mathematically exact inverse mapping:
    $$x_{cmd} = \text{position}[0] - \text{offset}_x$$
    $$y_{cmd} = -(\text{position}[1] - \text{offset}_y)$$
    This cleanly resolves the coordinate double-swap bug and aligns setpoint commands perfectly with flight tracking!

### 2. Mission-Class Waypoint Mapping — Absolute SSoT (`exa_kinematics.py`)
-   **Mission SSoT Waypoints:** The pipeline reads the 4 nominal waypoints directly from the active mission class (`ExpCollision75Deg` or `ColumnSweepLoop`). This is the **absolute first priority** — the mission file defines the exact geometry.
-   **Why not dynamic extraction?** Segmented pass `.mcap` files contain PX4 trajectory interpolation commands (15+ intermediate setpoints every ~150mm), not the real 4 mission waypoints. Using these produces completely wrong sweep boundaries and cascading clearance/impact errors.
-   **Fallback chain:** Mission class → `dynamic_waypoints` (only if exactly 4 found) → hardcoded defaults.

### 3. Integrated Cage & Flight Conditions Terminology
-   **Standardized Naming:** All notebooks, pipelines, and plotting scripts strictly map parameters and data structures using `flights_rotating_cage`, `flights_fixed_cage`, `representative_rotating_cage`, and `representative_fixed_cage`.
-   **Bags Restored:** Reclaimed and verified previously broken 75° collision bags (`flight_20260524-1813` and `flight_20260524-1904`), demonstrating perfect segmentation and analysis.

### 4. Dynamic Contact Angle & Physical Clearance Analysis
-   **Clearance Estimation:** Correctly tracks closest approach times, physical clearance levels (accounting for a 0.358m cage diameter and 0.090m column diameter), and calculates actual physical contact normal angles during rigid body contact.

### 5. Timeline Event Alignment
-   **Axvline Markers:** All time-domain plots strictly overlay three perfectly synchronized vertical line markers denoting `Exp. Start-point`, `Impact`, and `Exp. End-point` boundaries, cropping the display limit to the exact $T_{start}-5s$ to $T_{end}+5s$ window.

---

# Explicit Flight Dataset Table

The following high-quality flight recordings are approved and registered as the primary experimental dataset for thesis analysis.

| Flight Date/Time | Angle | Condition | Representative? | Flight Bag Folder Name | Key Analytical Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **2026-05-24 19:04** | 75° | Fixed Cage | **Yes** (Fixed Primary) | `flight_20260524-1904_75°_column_collision_loop_fixed_cage` | **The primary fixed cage flight recording.** Completed full sweep collision loop with excellent grazing alignment. |
| **2026-05-27 14:02** | 75° | Rotating Cage | **Yes** (V1 Rotating) | `flight_20260527-1402_75°_column_collision_loop_rotating_cage` | **V1 Sweep Gate.** Completed full rotating cage collision sweep. |
| **2026-05-27 14:08** | 75° | Rotating Cage | **Yes** (V2 Rotating) | `flight_20260527-1408_75°_column_collision_loop_v2_rotating_cage` | **V2 Sweep Gate.** Shortened staging U-turn runway, stabilizing the entrance trajectory perfectly. |

---

# 🔍 3-Second IMU-MoCap Alignment Discrepancy Diagnostics

A ~3-second offset has been noted in some plots between the MoCap-calculated closest approach (spatial impact) and the high-frequency IMU shock wave signal (IMU physical accelerometer spike). 

### Potential Causes:
1.  **Clock Epoch Drift:** The OptiTrack system uses standard ROS time (system clock of the ROS Master), while the Pixhawk internally tracks time in microseconds since flight controller boot (`timestamp` field). If the ROS 2 bag recorder captures these without dynamic clock synchronization, they can drift or lag.
2.  **DDS Transport / Buffering Latency:** ROS 2 DDS middleware (like eProsima FastDDS or CycloneDDS) can buffer or queue high-frequency `/poses` messages if the wireless connection drops packets, leading to a visual lag.
3.  **Active Yaw Turning Maneuver Signals vs. Structural Impacts:** Gyro Z spikes at the boundaries are deliberate yaw turns (U-turn staging and recovery), not structural impacts. Actual rigid column contact creates structural shocks in $a_x/a_y$ and small angular rate spikes.

### Diagnostic Checklist & Resolution Protocol:
- [ ] **Auditing Time-Sync Offset:** Merging using `pd.merge_asof` requires shared temporal coordinate spaces. Verify that `/poses` (MoCap) and `/fmu/out/sensor_combined` (IMU) are converted using consistent system-level timestamps.
- [ ] **Dynamic Cross-Correlation Calibration:**
  $$\text{CrossCorr}(\tau) = \sum_t a_{\text{mocap}}(t) \cdot a_{\text{imu}}(t - \tau)$$
  Locate the peak offset $\tau_{\text{peak}}$ where double-differentiated MoCap acceleration matches Pixhawk IMU acceleration, and apply this calibration shift in `exa_loader.py`.
- [ ] **ROS 2 Topic Delay Analysis:** Run `ros2 topic delay /poses` during playback to determine streaming transport latency.
- [ ] **Structural Contact Signature Isolation:** Isolate physical shocks in body $a_x/a_y$ and ignore high-frequency yaw rate ($w_z$) spikes that correspond to U-turns.

---

# 💾 SQLite Collision Experiments Database Schema

To enable rigorous statistical comparison and thesis compilation, all processed flight passes automatically cache their calculated physical metrics inside a shared SQLite database located at `dev_logs/analysis/collision_experiments.db` (resolved dynamically relative to the package, never hardcoded).

### Purpose: What the Database Tracks and How

The collision experiments database acts as the central, unified repository of all quantitative kinematics and dynamics telemetry extracted from live physical flight experiments. It is designed to track three major facets of drone dynamics during safety cage contacts:

1. **Experimental Control Conditions & Energy States:**
   - Standardizes the physical configurations (`Rotating Cage` vs. `Fixed Cage`), the commanded reference velocity (`sweep_speed`), and the current electrical state of the vehicle (`battery_at_start`) to account for changes in thrust-to-weight and voltage sag during high-energy sweeps.
2. **Impact & Bypass Kinematics:**
   - Captures the exact vehicle velocities (`impact_speed`), contact angles normal to the column's surface (`impact_angle`), and acceleration/deceleration rates (`before_impact_accel`, `impact_accel`) at the moment of closest spatial approach. 
   - Tracks spatial clearance boundaries (`closest_clearance`) in centimeters where negative values reflect physical penetration and cage compression, automatically classifying structural contact sweeps (`impact_detected = 1`).
3. **Flight Trajectory Deviation & Controller Recovery:**
   - Tracks spatial deviation along the recovery path (`avg_dev_after`, `max_dev_after`, `recovery_area`) representing the Spatial Integral of Absolute Error recovery envelope.
   - Saves side-by-side coordinate pairs of both the mathematical commanded coordinates (**Nominal** targets defined by the mission SSoT) and the physically captured coordinates (**Actual** 3D positions logged by OptiTrack at pass start and end bounds).

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

### Accessing & Exporting the Data
To query this database in Python for custom stats or Overleaf manuscript tables:
```python
import os
import sqlite3
import pandas as pd

# Resolve database path dynamically relative to this script
db_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "collision_experiments.db")
conn = sqlite3.connect(db_path)
df = pd.read_sql_query("SELECT * FROM flights_summary", conn)
conn.close()

# Export directly to Overleaf-compatible LaTeX code
latex_table = df.to_latex(index=False)
```

---

# ✂️ MCAP Pass Segmenter Tool

The MCAP segmenter is a standalone CLI utility that physically slices a full raw flight recording into separate, self-contained pass-level `.mcap` files immediately after landing.

**Script Location:** [`experiments_analysis/mcap_segmenter.py`](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/experiments_analysis/mcap_segmenter.py)

### What It Does
1. Loads the primary `.mcap` from the flight folder (ignores any existing `-passXX.mcap` slice files automatically).
2. Runs the same `find_waypoint_events` boundary detection as the analysis pipeline to find the start/end timestamps of each implicit sweep pass ($WP2 \rightarrow WP3$).
3. Slices all ROS 2 messages within the detected window (plus a configurable padding, default `±2s`) into a new standalone bag.
4. Saves each slice as `<original_bag_name>-pass01.mcap`, `-pass02.mcap`, etc. **inside the same flight folder**.

### Output Naming Convention
```
flight_YYMMDD-HHMM_<description>_0-pass01.mcap
flight_YYMMDD-HHMM_<description>_0-pass02.mcap
...
```

### Running the Segmenter
```bash
# Run on the default approved flight:
python3 dev_logs/analysis/experiments_analysis/mcap_segmenter.py

# Run on any explicit flight folder path:
python3 dev_logs/analysis/experiments_analysis/mcap_segmenter.py <path_to_flight_folder>
```

### Important Rules
- **Never hardcode paths** inside the segmenter. All paths are resolved dynamically via `os.path`.
- The segmenter **will not overwrite** existing slice files — delete old slices manually if re-slicing.
- The main analysis pipeline (`exa_loader.py`) automatically ignores `-passXX.mcap` files so slicing never corrupts existing analysis runs.
- The `±2s` padding ensures the full approach and recovery are always captured in the slice.
