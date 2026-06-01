# Experiments Analysis Pipeline - Skill & Cookbook

This document serves as the Single Source of Truth (SSoT) for the telemetry, kinematics, and database analysis pipeline. It dictates the required directory architecture, code standards, data processing rules, and specific formatting for all generated plots and data access.

## What is the purpose of these experiments?
These physical experiments involve flying a PX4-controlled quadcopter into a static column obstacle to study physical collision dynamics and recovery.
* **Rotational Inertia & Stabilisation:** We analyze how the drone reacts to column contact across two principal safety cage configurations:
  * **`<Rotating Cage>`:** The outer protective cage is free to rotate around the drone's physical Z-axis.
  * **`<Fixed Cage>`:** The protective cage is rigidly fixed to the vehicle frame (no Z-axis rotation).
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
3. **Dynamic MoCap Publish Rate & Telemetry Jitter:** The nominal tracking rate (e.g., 240 Hz vs. 120 Hz) must be dynamically computed directly from the `/poses` message frequency in the `.mcap` file, ensuring no hardcoded rates are displayed in diagnostic subplots.
   * **Telemetry Dropout Artifacts:** When the publish rate collapses (specifically dropping below $50\text{Hz}$ down toward the $30\text{Hz}$ failsafe threshold), numerical derivative calculations are subject to non-physical spikes. In these periods, packet lag followed by queued bursts creates artificial coordinate "teleportation" and time step compression ($dt \rightarrow 0$), skewing tangential velocity and acceleration.
   * **Smoothed Signal Processing Standard [Fully Implemented & Verified]:** To generate clean, publication-grade figures that ignore these dropouts, a parallel smoothed kinematic plot option is supported. This is achieved by **Uniform Grid Resampling & Spline Interpolation**—resampling the raw position coordinate tracks ($x, y, z$) onto a perfectly uniform $100\text{Hz}$ grid and interpolating across dropout gaps using a cubic spline *before* running the Savitzky-Golay derivative filter, mathematically neutralizing packet-loss spikes.
4. **Strict Data Truncation:** To keep plots clean and relevant, all flight data must be dynamically cropped to exactly **1 second before the "Exp. Start-point"** and **1 second after the "Exp. End-point"**. All telemetry outside this window should be discarded.
5. **Standardized Terminology SSoT:**
   * **WP1 / Entry Point** is renamed to: **`Exp. Start-point`**
   * **WP2 / WP4 / Exit Point** is renamed to: **`Exp. End-point`**
   * **Cage Condition Labels:** Whenever signifying the cage configuration type (in labels, titles, legends, or indicators), strictly format them as **`<Rotating Cage>`** and **`<Fixed Cage>`** (enclosed in angle brackets `< >`).
6. **Label Origin of Data:** All figures must have a small label in the bottom-right corner indicating the origin of the data (the exact flight pass name, e.g. `flight_20260524-1813_75deg_column_collision_loop_fixed_cage - Pass-01`).
7. **Timeline Event Alignment & Shading:** Every plot that uses time ($t$) as the X-axis (specifically, all timestamped plots) MUST display:
   * **Three perfectly aligned vertical event lines** across all subplots, denoting:
     * **`Exp. Start-point`** (vertical dotted purple/grey line)
     * **💥 `Impact`** (vertical dash-dotted crimson line at time of closest approach / actual impact)
     * **`Exp. End-point`** (vertical dotted purple/grey line)
   * **Impact Label Non-Obscuration Rule:** The text label for the **💥 `Impact`** event MUST ALWAYS be positioned to the **left (before)** the vertical crimson marker line (e.g. at `t_val - 0.12` with horizontal alignment `ha='right'`). This ensures that the immediate post-impact rebound, decay, and stabilization telemetry curves are completely unobstructed and legible.
   * **Shared Impact Window Shading:** A perfectly synchronized, shared light red vertical shaded region (`axvspan` spanning from $t_{\text{impact}} - 0.05\text{s}$ to $t_{\text{impact}} + 0.35\text{s}$ with `color='#D62728'` and `alpha=0.10` and `zorder=3`) representing the physical column contact window. This shared indicator allows the reader to instantly correlate speed drops, deceleration peaks, and structural shockwaves across all timestamped plots.

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
  * **Perpendicular Maximum Deviation ($d_{\text{max}}$):** An orange dashed line perpendicular to the nominal path line ($WP2 \rightarrow WP3$), indicating the peak post-impact rebound displacement. The search is strictly constrained to post-collision samples ($t \ge t_{\text{collision}}$) to isolate physical rebound from stabilization drift. The text background bounding box uses a compact `pad=1.0` padding.
  * **Hatched Recovery Envelope Area:** A light purple diagonal hatched region (`//`) between the actual recovery trajectory and the nominal commanded line segment after column contact.
* **Visual Standard & Requirements:**
  * **Title:** Must be changed strictly to `Experiment 2D horizontal visualization <Condition>` (bold, 12pt), where `<Condition>` is either `<Rotating Cage>` or `<Fixed Cage>`.
  * **Coordinate Axis Color-Coding:** Axis labels are simplified strictly to:
    * Horizontal Axis: `Y coordinate, meters` in Green (`#2CA02C`) with green ticks.
    * Vertical Axis: `X coordinate, meters` in Red (`#D62728`) with red ticks.
  * **Laboratory Rotation Disclaimer:** Printed in the lower-left corner as a left-aligned, 3-line multiline italicized block to prevent horizontal overlapping with the bottom-right flight name stamp:
    *"X and Y axes have been rotated on this plot\nto strictly adhere to the physical mapping of X-Y\nin the motion capture system used"*
  * Must maintain a strictly equal aspect ratio ($1:1$ physical meter mapping) to prevent skewing.
  * **Eliminate Box Outlines:** All text labels MUST use borderless white background boxes (`edgecolor='none'`, `alpha=0.9`) to prevent clipping frames or text overlaps.
  * **Subtle Leader Pointer Lines:** Use light gray indicators (`arrowprops=dict(arrowstyle="->", color='#888888', lw=0.8)`) pointing from the labels directly to their coordinate targets.
  * **Exp. Start-point Repositioning:** To completely prevent legend layering, the starting waypoint annotation box (`Exp. Start-point`) is permanently positioned in the lower-left of the data coordinates (`(-1.4, -0.25)`), connected via pointer arrow to the start drone outline.
  * **Exp. End-point Repositioning:** The `Exp. End-point` label is positioned directly vertically above the end drone center (`xytext=(0, 50)` offset points, `ha='center'`) to completely prevent horizontal overlapping with the right y-axis plot boundary.
  * **Table-Like Monospace Legend:** Legend labels are aligned in monospace font with custom character padding so that metric values and parentheses align in straight vertical columns:
    ```
    Actual Path       (MoCap ENU)
    Safety Cage       (D=35.8cm)
    Separation Vector
    Recovery Area     (829.9 cm²)
    Max Deviation     (110 mm)
    Column Obstacle   (D=9.0cm)
    ```
  * **Grey shaded drones must be drawn at actual MoCap-registered positions**, not theoretical waypoints. This includes:
    1. Actual position at `Exp. Start-point` (annotated with actual MoCap $(X, Y)$ coords).
    2. Actual position at `Exp. End-point` (annotated with actual MoCap $(X, Y)$ coords).
    3. Actual position at the moment of closest approach (annotated with minimum physical clearance value in mm, impact speed, and impact acceleration).
* **Associated Script:** [kin_plot_trajectory.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/kinematics/kin_plot_trajectory.py)

### 2. Consolidated Flight Kinetic & Kinematic Profile
* **Purpose:** Combines the smoothed velocity magnitude, linear tangential acceleration, and live MoCap tracking rates into a single 3-subplot vertical stack to align all kinematic and stream transmission diagnostics perfectly in time.
* **What it Shows:**
  * **Subplot 1 (Top):** Velocity magnitude ($m/s$) with horizontal dashed lines showing average speed during active flight segments.
  * **Subplot 2 (Middle):** Filtered tangential acceleration ($m/s^2$) with green color-filled acceleration zones, red color-filled deceleration zones, and a bold zero-acceleration steady-state reference line.
  * **Subplot 3 (Bottom):** Live frame rate ($Hz$) of the `/poses` stream with a red dashed line marking the $30\text{Hz}$ critical tracking limit and a green dotted line marking the dynamic nominal tracking rate.
* **Visual Standard & Requirements:**
  * **Consolidated Figure Layout:** The three subplots are vertically stacked, share the same time X-axis (`sharex=True`), and are perfectly aligned in time (display window cropped strictly to $1\text{s}$ before/after waypoint markers).
  * **Title Requirements:**
    * Top Subplot: `Flight Kinetic profile (Savitzky-Golay filter applied) <Condition>` (e.g. `<Rotating Cage>` or `<Fixed Cage>`)
    * Middle Subplot: `Tangential Acceleration Profile <Condition>`
  * **Column Center Passed Line Color:** The vertical event line marking the `Column Center Passed` (or `Column Passed`) event must be **orange** (`#FF9900`) to match the color of the column obstacle.
  * **Synchronized Impact Window Shading:** A perfectly synchronized vertical light red shaded region (`axvspan` spanning from $t_{\text{impact}} - 0.05\text{s}$ to $t_{\text{impact}} + 0.35\text{s}$) must overlay ALL three subplots to allow immediate visual correlation.
  * **Impact Label Non-Obscuration Rule:** The **💥 Impact** label text must be positioned to the **left (before)** the vertical crimson marker line (`t_val - 0.12` and `ha='right'`) across all subplots, keeping post-impact recovery curves completely legible.
* **Associated Script:** [kin_plot_kinematics.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/kinematics/kin_plot_kinematics.py)

### 4. Physical IMU Collision Dynamics
* **Purpose:** Isolates the high-frequency physical impact event and the drone's immediate structural and rotational response.
* **What it Shows:**
  * **Linear Accel Deviation ($\text{m/s}^2$):** Measures the sudden peak G-forces experienced by the IMU during column contact.
  * **Gyro Rotational Surge ($\text{rad/s}$):** Displays the sudden angular velocity surge (roll/pitch/yaw perturbations) resulting from cage contact, showing the physical rebound dynamics before active FC stabilization takes over.
  * **Core Shock & Vibration Metrics [Fully Implemented & Verified]:**
    * **Peak G-Shock & Rotation:** Tracks peak linear acceleration deviation ($A_{\text{peak}} = \max(a_{\text{dev}})$) and peak angular rate ($\Omega_{\text{peak}} = \max(\omega_{\text{mag}})$) during the impact contact window.
    * **Integrated Energy (Total Shock Impulse):** Calculates total cumulative shock impulse ($I_{\text{accel}} = \int a_{\text{dev}} \, dt$ in $\text{m/s}$) and rotational disturbance energy ($I_{\text{gyro}} = \int \omega_{\text{mag}} \, dt$ in $\text{rad}$) integrated strictly over the **$0.40\text{-second}$ Contact Window** ($[t_{\text{impact}} - 0.05\text{s}, t_{\text{impact}} + 0.35\text{s}]$).
    * **Settling Time (Damping & Recovery):** Tracks linear ringing decay settling time (duration from impact until $a_{\text{dev}}$ drops and stays below $1.5\text{ m/s}^2$) and attitude stabilization settling time (duration from impact until $\omega_{\text{mag}}$ drops and stays below $0.5\text{ rad/s}$).
* **Visual Standard & Requirements:**
  * **Title:** strictly `IMU Collision Dynamic <Condition>` (bold, 12pt).
  * **Dual Y-axes:** Left (red) for linear acceleration deviation, Right (blue) for gyro rotational surge.
  * **Left Y-axis Range & Divisions:** The left Y-axis must always have a strict range of **$-1.0\text{ m/s}^2$ to $20.0\text{ m/s}^2$** with ticks placed exactly at **$2.0\text{ m/s}^2$ intervals** (`MultipleLocator(2.0)`).
  * **X-axis Divisions & Truncation:** The X-axis must have ticks placed exactly at **$1.0\text{s}$ intervals** (`MultipleLocator(1.0)`). The timeline display window is strictly cropped to exactly **$1\text{s}$ before the `Exp. Start-point` and $1\text{s}$ after the `Exp. End-point`**.
  * A horizontal gold dashed line marking the "Severe Impact Threshold" at $5.0\text{ m/s}^2$.
  * Three vertical timeline event lines marking: `Exp. Start-point`, `Impact`, and `Exp. End-point` (per General Plot Rule 7).
  * **Impact Window Shading:** Includes a unified light red vertical shaded region overlaying the exact contact duration.
* **Associated Script:** [kin_plot_kinematics.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/kinematics/kin_plot_kinematics.py)

### 5. RAW IMU X/Y/Z Components
* **Purpose:** Breaks down the high-frequency IMU linear and angular readings along the Pixhawk 6C's principal physical body axes.
* **What it Shows:**
  * Raw/Filtered linear acceleration ($a_x, a_y, a_z$) and rotational rates ($\omega_x, \omega_y, \omega_z$).
* **Visual Standard & Requirements:**
  * **Title:** strictly `Raw IMU X / Y / Z Components <Condition>` (bold, 12pt), with subtitle `[X = Lateral/Roll, Y = Longitudinal/Pitch, Z = Vertical/Yaw]`.
  * **Axis Mappings & Labels:** Mappings must be professionally labeled to explain Pixhawk 6C axes:
    * **X-Axis (Lateral / Roll):** Side-to-side dynamics.
    * **Y-Axis (Longitudinal / Pitch):** Fore-and-aft dynamics along the main flight vector.
    * **Z-Axis (Vertical / Yaw / Heave):** Up-and-down dynamics and heading rotations.
  * **Proportional Subplot Heights (Equal Physical Scaling):** To keep the vertical physical scale ($1\text{ m/s}^2$ per inch) exactly identical across all three subplots for direct visual magnitude comparison, the figure subplots must use proportional GridSpec height ratios: **`gridspec_kw={'height_ratios': [26, 26, 20]}`**.
  * **Strict Y-axis Range & Divisions:**
    * **X-Axis Panel:** Locked strictly to range **`[-20.0, 6.0]`** `m/s²` with ticks at **`2.0 m/s²` intervals**.
    * **Y-Axis Panel:** Locked strictly to range **`[-6.0, 20.0]`** `m/s²` with ticks at **`2.0 m/s²` intervals**.
    * **Z-Axis Panel:** Locked strictly to range **`[-20.0, 0.0]`** `m/s²` with ticks at **`2.0 m/s²` intervals**.
  * **X-axis Truncation:** Timeline display window is strictly cropped using the **1-second crop window** standard (1 second before `Exp. Start-point` and 1 second after `Exp. End-point`).
  * Three vertical timeline event lines marking: `Exp. Start-point`, `Impact`, and `Exp. End-point` (per General Plot Rule 7).
* **Associated Script:** [kin_plot_kinematics.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/kinematics/kin_plot_kinematics.py)

### 6. Post-Impact Stabilisation Deviation (<Rotating Cage> Only)
* **Purpose:** Correlates nominal transit speed with average tracking error after collision to quantify vehicle stability under different translational kinetic energies.
* **What it Shows:**
  * Sourced strictly from `<Rotating Cage>` flights with confirmed impacts (`df_rot`).
  * X-Axis: Nominal sweep speed ($m/s$).
  * Y-Axis: Average perpendicular trajectory deviation after contact, scaled to centimeters ($cm$) via:
    $$\text{dev}_{\text{cm}} = \text{avg\_dev\_after} / 10.0$$
  * Dash-dotted regression trendline: Overlay of a linear best-fit line showing the drift scaling rate, with the exact mathematical slope ($m$) displayed in the legend.
* **Associated Script/Notebook:** [experiments_analysis_summary.ipynb](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/experiments_analysis_summary.ipynb)

### 7. Deviation vs. Measured Impact Angle (<Rotating Cage>)
* **Purpose:** Analyzes how the actual geometric angle of contact influences post-collision drift, while mapping start LiPo voltage states to isolate power system correlation.
* **What it Shows:**
  * Sourced strictly from `<Rotating Cage>` flights with confirmed impacts (`df_rot`).
  * X-Axis: `'Impact Angle'`, scaling strictly from $0^\circ$ to $90^\circ$ (`xlim(0, 90)`).
  * Y-Axis: Average post-impact deviation scaled to centimeters ($cm$), hardcoded strictly from $0$ to $15$ cm (`ylim(0, 15)`).
  * 4-bin LiPo Battery State Color-Gradient:
    * $[0\%, 40\%]$ $\rightarrow$ **Red** (`#D62728`)
    * $(40\%, 60\%]$ $\rightarrow$ **Orange** (`#FF7F0E`)
    * $(60\%, 80\%]$ $\rightarrow$ **Yellow-Green** (`#BCBD22`)
    * $(80\%, 100\%]$ $\rightarrow$ **Green** (`#2CA02C`)
  * **Statistical Overlays (Linear Trend Lines):** Linear regression best-fit lines matching each bin's color (plotted for battery states with $n > 1$) demonstrating performance trends across impact angles.
  * **Enhanced Legend Metadata:**
    * Compact, table-like layout: Percentage ranges and occurrence counts `(n = x)` are vertically aligned in columns. Emojis and verbose description parenthesis are omitted.
    * Cumulative total: Features a clear total label `Total Flights (N = XX)` positioned at the very top of the legend above all entries.
* **Associated Script/Notebook:** [experiments_analysis_summary.ipynb](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/experiments_analysis_summary.ipynb)

### 8. Deviation vs. Measured Impact Angle (<Fixed Cage>)
* **Purpose:** Analyzes how the actual geometric angle of contact influences post-collision drift for the rigidly Fixed Cage safety configuration across start LiPo battery states.
* **What it Shows:**
  * Sourced strictly from `<Fixed Cage>` flights with confirmed impacts (`df_fix`).
  * X-Axis: `'Impact Angle'`, scaling strictly from $0^\circ$ to $90^\circ$ (`xlim(0, 90)`).
  * Y-Axis: Average post-impact deviation scaled to centimeters ($cm$), hardcoded strictly from $0$ to $15$ cm (`ylim(0, 15)`).
  * 4-bin LiPo Battery State Color-Gradient:
    * $[0\%, 40\%]$ $\rightarrow$ **Red** (`#D62728`)
    * $(40\%, 60\%]$ $\rightarrow$ **Orange** (`#FF7F0E`)
    * $(60\%, 80\%]$ $\rightarrow$ **Yellow-Green** (`#BCBD22`)
    * $(80\%, 100\%]$ $\rightarrow$ **Green** (`#2CA02C`)
* **Associated Script/Notebook:** [experiments_analysis_summary.ipynb](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/experiments_analysis_summary.ipynb)

### 9. Comparative Stabilization Overlay (<Rotating Cage> vs. <Fixed Cage>)
* **Purpose:** Direct thesis comparison of recovery deflection performance across all impact angles, mathematically isolating the stabilization efficacy of the Rotating Cage safety design against the Fixed Cage baseline.
* **What it Shows:**
  * Overlays both `<Rotating Cage>` (`df_rot`) and `<Fixed Cage>` (`df_fix`) impact-only flights.
  * X-Axis: `'Impact Angle'`, scaling strictly from $0^\circ$ to $90^\circ$ (`xlim(0, 90)`).
  * Y-Axis: Average post-impact deviation scaled to centimeters ($cm$), hardcoded strictly from $0$ to $15$ cm (`ylim(0, 15)`).
  * Scatter points: actual flight passes drawn lightly in the background (`alpha=0.25`) using matching battery colors to preserve line focus.
  * Overlaid 8x Mathematical trendlines (linear regression):
    * **Rotating Cage Trends:** **Dashed Lines** (`linestyle='--'`) in matching colors for each of the 4 battery bins.
    * **Fixed Cage Trends:** **Solid / Full Lines** (`linestyle='-'`) in matching colors for each of the 4 battery bins.
    * Legend dynamically displays the exact mathematical slopes ($m$) for immediate comparative validation.
* **Associated Script/Notebook:** [experiments_analysis_summary.ipynb](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/experiments_analysis_summary.ipynb)

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
3. **CRITICAL PIPELINE REBUILD CORE MEMORY RULE FOR CO-PILOTS:** Rebuilding or populating the database pipeline (`db_pipeline.py` or executing the full `experiments_analysis.ipynb`) triggers high-volume physical parsing of 172+ raw MCAP flight passes and takes extensive computing time. **YOU MUST ALWAYS EXPLICITLY ASK THE USER FOR CONFIRMATION before invoking a full pipeline rebuild or running any batch backfill command!** Whenever possible, ask the user to execute the command themselves inside their local environment rather than running it asynchronously or automatically behind the scenes to ensure perfect visibility and avoid unexpected system slowdowns.

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
| **`imu_peak_accel`** | REAL | $\text{m/s}^2$ | **[Fully Implemented & Verified]** Peak linear acceleration deviation during the $0.4\text{s}$ impact window. |
| **`imu_peak_accel_x/y/z`** | REAL | $\text{m/s}^2$ | **[Fully Implemented & Verified]** Peak linear acceleration along individual body axes ($X$ = lateral, $Y$ = longitudinal, $Z$ = vertical) during impact. |
| **`imu_peak_gyro`** | REAL | $\text{rad/s}$ | **[Fully Implemented & Verified]** Peak angular rate magnitude during the $0.4\text{s}$ impact window. |
| **`imu_peak_gyro_x/y/z`** | REAL | $\text{rad/s}$ | **[Fully Implemented & Verified]** Peak angular rates (Roll $X$, Pitch $Y$, Yaw $Z$) during the $0.4\text{s}$ impact window. |
| **`imu_accel_energy`** | REAL | $\text{m/s}$ | **[Fully Implemented & Verified]** Integrated Linear Shock Impulse over the $0.4\text{s}$ impact window. |
| **`imu_accel_energy_x/y/z`** | REAL | $\text{m/s}$ | **[Fully Implemented & Verified]** Integrated shock impulse components along body $X$, $Y$, and $Z$ axes over the $0.4\text{s}$ contact window. |
| **`imu_gyro_energy`** | REAL | $\text{rad}$ | **[Fully Implemented & Verified]** Integrated Rotational Disturbance Energy over the $0.4\text{s}$ impact window. |
| **`imu_gyro_energy_x/y/z`** | REAL | $\text{rad}$ | **[Fully Implemented & Verified]** Integrated rotational disturbance energy (Roll, Pitch, Yaw) over the $0.4\text{s}$ contact window. |
| **`imu_accel_settling`** | REAL | seconds | **[Fully Implemented & Verified]** Linear Ringing Settling Time (duration from impact until $a_{\text{dev}}$ drops and stays below $1.5\text{ m/s}^2$). |
| **`imu_gyro_settling`** | REAL | seconds | **[Fully Implemented & Verified]** Attitude Stabilization Settling Time (duration from impact until $\omega_{\text{mag}}$ drops and stays below $0.5\text{ rad/s}$). |
| **`timestamp`** | TEXT | `YYYY-MM-DD HH:MM:SS` | Caching timestamp. |

# 📊 Comparative Summary Dashboard: `experiments_analysis_summary.ipynb`

The summary dashboard provides a centralized, publication-ready analytical environment for comparative experimental studies between Rotating and Fixed cage configurations. It is designed to be lightweight, zero-dependency (using 100% pure Matplotlib), and robust to SSHFS mount network latencies.

**Dashboard Location:** [`dev_logs/analysis/experiments_analysis_summary.ipynb`](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/experiments_analysis_summary.ipynb)

### 1. Zero-Dependency & Resilient Architecture
* **Path Resiliency Header:** The notebook begins with a dynamic traversal path injection header to guarantee execution from any arbitrary terminal working directory:
  ```python
  import sys, os
  project_root = os.path.abspath(os.path.join(os.path.abspath(''), "../../"))
  if project_root not in sys.path:
      sys.path.insert(0, project_root)
  ```
* **No Seaborn Dependency:** To ensure flawless runtime execution on minimal system environments, all plotting codes are written using pure standard Matplotlib rather than importing Seaborn.

### 2. Client-Side Pandas Filtering (SSoT Loader Chain)
To prevent SQLite file lock loops over SSHFS, the dashboard enforces **Option A: Client-Side Pandas Filtering**. It loads the entire database into memory once and splits the segments using Pandas query strings:
1. **Raw Database Ingestion:** Loads all telemetry: `df_all = get_database_df()`.
2. **Baseline Impact Filter:** Discards all non-impact flights: `df_impacts = df_all.query("impact_detected == 1")`.
3. **Sub-Dataset Splits (Pre-Filtered by Impact):**
   * **Global Cage Groups:** `df_rot` (Rotating Cage only), `df_fix` (Fixed Cage only).
   * **Nominal Geometry Bins:** `df_75` (75° nominal sweeps), `df_45` (45° nominal sweeps).
   * **Measured Angle Ranges:** `df_range_30_40` to `df_range_80_90` (6x custom 10-degree bins based on actual tangential contact angle).
   * **Measured Ranges sub-split by Cage:** `df_range_50_60_rot` / `df_range_50_60_fix` etc.

### 3. Dynamic Plot Swapping
All thesis-quality scatter and box plots inside the dashboard are designed for rapid iteration. To update the underlying data of a plot instantly, simply redefine the local `plot_data` variable at the top of the cell:
```python
# SWAP THIS variable to any pre-filtered loader to plot instantly!
plot_data = df_range_50_60_rot
```

---

## 🚀 Programmatic Notebook Querying: `get_database_df()`

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

---

# 📈 Core Flight & Trajectory Design Principles

These fundamental design rules dictate both how the physical drone is commanded in flight and how post-flight telemetry is structured for publication-grade thesis figures.

### 1. Spatial Aspect Ratio & Grid Discipline for Thesis Graphics
* **The Visual Standard:** All top-down 2D spatial trajectory plots (specifically when generating final figures for Chapter 5/6) **MUST** enforce a strictly equal aspect ratio ($1:1$ physical meter mapping) and a normalized grid of **$0.5\text{ m}$ squares**.
* **Why:** Default plotting axes auto-scale independently based on the data bounds. This distorts the spatial perspective, making a minor $10\text{ cm}$ lateral tracking deviation look like a massive $2\text{ m}$ overshoot, which severely undermines academic and scientific credibility. Enforcing equal mapping provides a physically truthful representation of the drone's collision deflection and recovery.

### 2. PX4 Offboard Velocity Feedforward Control
* **The Control Standard:** In all autonomous waypoint sweep loops (defined inside `drone_control/missions/`), velocity feedforward commands must be actively calculated and sent alongside position targets:
  ```python
  hb.velocity = True
  hb.vx = cmd_vel[0]
  hb.vy = cmd_vel[1]
  hb.vz = cmd_vel[2]
  ```
* **Why:** Position-setpoint-only commands force the PX4 flight controller to hunt aggressively toward target coordinates, leading to highly jerky speed profiles and massive vehicle attitude oscillations upon waypoint transition. Velocity feedforward acts as an explicit control hint, informing the flight controller not just *where* to fly, but *how fast* to be traveling at each point. This produces butter-smooth sweeping transits and stable column contact approaches.



