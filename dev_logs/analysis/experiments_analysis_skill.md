# Experiments Analysis Pipeline - Skill & Cookbook

This Single Source of Truth (SSoT) defines the directory architecture, coding standards, data processing, and visual formatting for the thesis experiments (Comparing `<Rotating Cage>` vs `<Fixed Cage>` collision dynamics and IMU/MoCap correlation).

## 1. Modular Workspace & Cleanliness Rules
The workspace (`dev_logs/analysis/`) is strictly partitioned.
* **`database/`**: Ingestion, MCAP slicing, and SQLite managers.
* **`kinematics/`**: Physics calculations and plot generators.
* **`dev_logs/scratch/`**: **MANDATORY** location for all temporary/playground scripts (`scratch_*.py`), cell editors, or one-time-use helper tools.
* **Cleanliness Policy:** `analysis/` MUST contain ONLY permanent analysis scripts, active databases, and graphics. Helper scripts and temporary files must be kept in `dev_logs/scratch/` and deleted or archived once executed.
* **API Access:** Expose shared functions in package `__init__.py` files.

## 2. Core Data Processing & Flight Control
* **Mission SSoT:** Nominal waypoints must be dynamically extracted from the active mission class (e.g., `ExpCollision75Deg`), NOT from interpolated `.mcap` setpoints.
* **Coordinate Alignment:** EKF-to-MoCap alignment offsets are dynamically estimated using `pd.merge_asof` (median difference), and setpoints are restored to the ENU frame to fix orientation bugs.
* **MCAP Segmenter:** A standalone tool (`db_mcap_event_segmenter.py`) slices raw flights into `-passXX.mcap` bags based on waypoint events.
* **Velocity Feedforward:** Autonomous sweeps must use velocity feedforward control (`hb.velocity = True`) for smooth transit, preventing jerky position-only hunting.

## 3. SQLite Database (`experiments_summary.db`)
* **SSHFS Safety:** To prevent locking loops over network mounts, never open `.db` files externally. Use a 30s timeout and `PRAGMA busy_timeout`.
* **Querying:** Always load the DB into Pandas (`get_database_df()`) and filter client-side (e.g., `df.query("impact_detected == 1")`).
* **Table `flights_summary` Key Metrics:**
  * `condition` ('Rotating Cage' or 'Fixed Cage')
  * `impact_angle` (0-90° normal contact angle)
  * `battery_at_start` (0-100% scale)
  * `avg_dev_after`, `max_dev_after` (Trajectory deviations)
  * `recovery_area` (Integrated spatial error envelope)
  * `imu_peak_accel`, `imu_peak_gyro` (Peak shock metrics)
  * `imu_accel_energy`, `imu_gyro_energy` (Integrated shock impulse over 0.4s window)

## 4. Universal Plotting Standards
* **Zero Dependency:** Use 100% pure Matplotlib (no Seaborn).
* **SSHFS Filenames:** Never use the `°` symbol in filenames (use `deg`).
* **Data Truncation:** Dynamically crop all timestamped plots to **1s before `Exp. Start-point` and 1s after `Exp. End-point`**.
* **Strict Coordinate Ticks & Ranges:** All plots must use hardcoded, explicit axis limits and divisions (e.g., `set_xlim(0, 90)`) for visual repeatability.
* **Enclosure Line Styles & Trendlines:** **`<Rotating Cage>` = Dashed (`--`)**, **`<Fixed Cage>` = Solid (`-`)**. Trendlines must use standard linear regression ($y = mx + c$) computed via least-squares `np.polyfit`. All plots with trendlines must include the footnote/subtitle explaining how trendlines are calculated: `Trendlines calculated via linear regression (y = mx + c)` or similar.
* **Data Origin Labeling:**
  * **Summary Outputs:** Every summary plot and comparative table must show the dynamic data configuration/origin in the bottom-right corner (or as a subtitle). It must explicitly declare the exact counts: `Comparison of Xx Rotating Cage and Yx Fixed Cage flights` (e.g., `Comparison of 67x Rotating Cage and 72x Fixed Cage flights`). The numbers must be dynamically computed from the active filtered dataset.
  * **Individual Outputs:** Figures showing single flights/passes must print the source bag filename or flight log ID in the bottom-right corner.
* **Comparative Tables (Table of Averages):**
  * Grouped by impact angle bins and cage condition.
  * Bins must range from 30° to 60° in 10-degree intervals, while the 60-90° range must be collapsed into a single row/bin (e.g., bins `30-40°`, `40-50°`, `50-60°`, and `60-90°`).
  * Display a mission outcomes overview grid of 6 pie charts above the Table of Averages, detailing impact vs. no-impact distributions and achieved angle ranges for the rotating and fixed cages overall and split by 45° and 75° mission profiles.
  * Sorted so that Fixed Cage and Rotating Cage alternate (oscillate) under each angle bin (e.g., bin 30-40° Rotating, bin 30-40° Fixed, bin 40-50° Rotating, etc.).
  * Must include an explicit `N-Flights` column showing the exact number of passes/flights the average was made of for each bin.
  * Color-coded formatting: Alternating background colors for the first 4 columns to oscillate between Rotating Cage (blue-tinted: `#d2e4f6` background, `#0d233a` text) and Fixed Cage (red-tinted: `#f7d2d2` background, `#3d0b0b` text). For the remaining comparative metrics columns, apply a ratio-based gradient heatmap using varying shades of HSL green (better/lower value) and HSL red (worse/higher value) matching the ratio between sibling conditions.
  * To avoid dependency issues (e.g., missing `jinja2` for Pandas `.style`), implement styling using raw inline HTML table rendering.
* **Timeline Event Alignment:** Timestamped plots must feature three synchronized vertical lines:
  1. `Exp. Start-point` (dotted purple/grey)
  2. `💥 Impact` (dash-dotted crimson). Label must be placed on the **left** to avoid obscuring recovery curves.
  3. `Exp. End-point` (dotted purple/grey)
* **Impact Window Shading:** Apply a synchronized light red shaded region (`t_impact - 0.05s` to `t_impact + 0.35s`) across all subplots.
* **Battery Bins:** Always use a 0-100 percentage scale: Red [0-40%], Orange (40-60%], Yellow-Green (60-80%], Green (80-100%].

## 5. Specific Plot References
1. **Top-Down 2D Spatial Trajectory (`kin_plot_trajectory.py`)**
   * **Mandatory 1:1 Aspect Ratio** and normalized 0.5m grid to prevent spatial distortion.
   * Axes rotated (`-y, x`). Y=Green, X=Red. Includes laboratory rotation disclaimer.
   * Labels use borderless white backgrounds. Pointers connect labels to targets.
   * **Base Reuse:** All 2D Path Overlays (like Heatmaps) must inherit this exact layout, limit `[-1.6, 1.6]/[-0.5, 1.0]`, and CAD mechanism.
2. **Consolidated Flight Kinetic Profile**
   * Vertically stacked (Speed, Accel, Frame Rate) sharing the X-axis time window.
   * Applies uniform grid resampling & spline interpolation to eliminate packet-loss spikes.
3. **Physical IMU Collision Dynamics**
   * Dual Y-axes (Linear Accel vs Gyro Surge). Left axis locked to `-1.0 to 20.0 m/s²`.
   * Integrates shock impulse and calculates settling times (ringing decay).
4. **RAW IMU X/Y/Z Components**
   * Three subplots (Lateral/X, Longitudinal/Y, Vertical/Z) with equal physical scaling (`height_ratios=[26,26,20]`).
   * Explicit limits: X [-20,6], Y [-6,20], Z [-20,0].
5. **Comparative Overlays (Deviation vs Angle)**
   * Plots map actual impact angle (X) vs Max/Avg deviation (Y), using Battery color bins.
   * Overlays mathematical trendlines (Rotating=Dashed, Fixed=Solid) with slopes documented in custom monospace legends.
6. **IMU Peak Acceleration Z vs Commanded Motor Speed (RPM) (Plot A)**
   * Single plot mapping estimated RPM (X, calculated as `2000 + 10000 * motor_max_after`) vs IMU Peak Accel Z (Y, `imu_peak_accel_z`) for all impacts.
   * Differentiate configurations using marker shapes: Rotating Cage = Squares (`s`), Fixed Cage = Circles (`o`).
   * Color-coded gradient mapped to impact angle (range 30° to 90°) using the `plt.cm.coolwarm` colormap.
   * No trendlines. Standard dynamic data origin label in the bottom-right corner.

## 6. Workspace Organization & Tool Cleanup
* **Directory Hygiene:** Maintain strict separation of concerns. 
  * `dev_logs/analysis/` is exclusively for the core, production-grade analysis pipelines and dashboards (e.g., `db_pipeline.py`, `experiments_analysis_summary.ipynb`). Do not clutter this space.
  * `dev_logs/scratch/` is the designated area for all one-off scripts, prototyping tools, temporary data checks, and experimental code.
* **Routine Tool Cleanup:** Regularly audit and delete one-time use tools, experimental scratchpads, and obsolete helper scripts once their specific task is completed. Do not leave dead code in the repository.
