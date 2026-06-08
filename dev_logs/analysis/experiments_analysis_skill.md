# Experiments Analysis Pipeline - Skill & Cookbook

This Single Source of Truth (SSoT) defines the directory architecture, coding standards, data processing, and visual formatting for the thesis experiments (Comparing `<Rotating Cage>` vs `<Fixed Cage>` collision dynamics and IMU/MoCap correlation).

## 0. Walkthrough Document — Planning, Execution & Verification Protocol

This project uses a **single shared roadmap document** for all telemetry analysis tasks. Everything flows through it.

### 0.1 The Walkthrough Document

| Property | Value |
|---|---|
| **Location** | `dev_logs/analysis/walkthrough_experiments.md` |
| **Role** | SSoT for what to implement, what's pending, what's done |
| **Structure** | Mirrors the notebook cell sequence: `experiments_analysis.ipynb` → `experiments_analysis_summary.ipynb` |
| **Format** | Checkboxes `- [ ]` for pending, `- [x]` for done, with subsections per topic |

> **When the user asks "where is that in the walkthrough?"** — answer with the exact section heading (e.g., "Under `### Advanced Thesis Highlights`"). Always anchor discussions in the walkthrough structure.

### 0.2 The Skill File (This Document)

| Property | Value |
|---|---|
| **Location** | `dev_logs/analysis/experiments_analysis_skill.md` |
| **Role** | Coding standards, architecture rules, visual formatting SSoT |
| **Ordering** | Sections 1–8 use the **same order** as `walkthrough_experiments.md` sections for quick look-up |
| **Relation** | Skill file = *how* to implement; Walkthrough = *what* to implement next |

### 0.3 Execution Protocol — Italic Execution Notes

When the AI implements a task from the walkthrough, it **updates this skill file** at the relevant subsection with an **italic block** describing:

```
*[YYYY-MM-DD] **What was done:** <one-line summary of the implementation>
**How to verify:** <exact steps to check the change works — run which cell, look for which output>
**Where:** <file path and cell/section reference>*
```

This creates a persistent execution log that the user can scan later to pick up context without re-reading the full conversation.

#### Example (after implementing task in `### Advanced Thesis Highlights`):

```
*[2026-06-08] **What was done:** Added comparative performance improvement printouts after the Recovery Area boxplot and Plot B scatter plot. Prints % reduction of Rotating Cage vs Fixed Cage for recovery_area and max_dev_after.
**How to verify:** Run cells in Step 5 → look for "✅ Rotating Cage reduces recovery area by XX.X%" printed below the boxplot.
**Where:** experiments_analysis_summary.ipynb, new code cells after the recovery boxplot cell and after Plot B cell.*
```

### 0.4 Quick-Reference: Section Mapping

| Walkthrough Section → | Skill File Section → | Notebook |
|---|---|---|
| 1. Individual Flight Ingestion & Analysis | 1. Modular Workspace & Cleanliness Rules + 2. Core Data Processing | `experiments_analysis.ipynb` |
| Telemetry Ingestion & Database | 3. SQLite Database + (pipeline details) | `db_pipeline.py` |
| Individual Pass Visualizations | 5. Specific Plot References (items 1–4) | `experiments_analysis.ipynb` |
| 2. Aggregate Comparative Dashboard | 4. Universal Plotting Standards + 5. Specific Plot References (items 5–6) | `experiments_analysis_summary.ipynb` |
| Battery, Deceleration & Structural Dynamics | 3. SQLite Database (battery metrics) + (Step 11–12) | `experiments_analysis_summary.ipynb` |
| Advanced Thesis Highlights | (freeform, per-task) | `experiments_analysis_summary.ipynb` |
| Statistical Aggregate Performance | 5. Specific Plot References (Plot 17–18) | `experiments_analysis_summary.ipynb` |

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
* **Y-Axis Truncation (Outlier Suppression):** When a high outlier compresses the meaningful data range, truncate the Y-axis to 6. Applies to:
  - **Recovery area boxplot:** `ax.set_ylim(0, 6)` with ticks every 1 cm². One outlier >6 cm².
  - **Deceleration vs Battery plots (both split & global):** `ax.set_ylim(0, 6)` with ticks every 1 m/s². One outlier at ~9.8 m/s². Truncating at 6 stretches the visible range so the Rotating vs Fixed trendline separation is clearly visible.
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

## 7. AI Behavior & Roadmap Management
* **Roadmap/Walkthrough Modification Constraint:** Under no circumstances should the AI assistant delete tasks, mark tasks as complete/done (`[x]`), or modify checklists in roadmaps (such as `walkthrough_experiments.md`) unless the user explicitly and deliberately instructs to do so. The AI assistant must always explicitly ask the user for confirmation and obtain clear approval before updating roadmap tasks.

## 8. Retired & Attempted Graphics/Metrics (Changelog / Hall of Shame)
To prevent repeating past visualization attempts that yielded low-value, redundant, or noise-dominated data, the following assets have been retired:
* **PID Rate Controller Tracking (Plot 2)**: Individual pass plots mapping rate setpoint vs. angular velocity. Retired as it was redundant with aggregate rate tracking error RMS metrics in the database and cluttered individual flight directories.
* **Motor Imbalance Profile (Plot F)**: Individual/aggregate post-impact motor signal variance. Found to be highly dependent on transient ground-effect interactions rather than cage-induced structural dynamics.
* **Sweep Velocity vs. Impact Speed Scatter (`sweep_vs_impact_speed.png`)**: Aggregate plot showing commanded transit sweep speed vs. actual velocity at impact. Found to be redundant due to the high consistency of the feedforward guidance controller.

## 9. Execution Log — Italic Notes (Most Recent First)

### `### Battery, Deceleration & Structural Dynamics` — Deceleration Y-Axis Truncation

*[2026-06-08] **What was done:** Added `ax.set_ylim(0, 6)` and `ax.set_yticks(np.arange(0, 7, 1))` to both deceleration plots — Cell 23 (split Rotating vs Fixed side-by-side, shared Y-axis via `ax1`) and Cell 24 (global comparison single `ax`). Suppresses the ~9.8 m/s² outlier so the 0–6 range is stretched and trendline separation is visible.
**How to verify:** Run Cells 23 and 24 → both deceleration plots now show Y-axis 0–6 with ticks every 1 m/s². The outlier at ~9.8 is clipped off-screen.
**Where:** `dev_logs/analysis/experiments_analysis_summary.ipynb` Cells 23 and 24.*

Each entry records a completed implementation from the walkthrough, in italic, cross-referencing where in the walkthrough it lives.

### `### Global Thesis Visualizations` — Recovery Area Boxplot Y-Axis Truncation

*[2026-06-08] **What was done:** Added `ax.set_ylim(0, 6)` and `ax.set_yticks(np.arange(0, 7, 1))` to the recovery area boxplot (Cell 12) to truncate a high outlier (>6 cm²) and stretch the vertical range so the blue (Rotating) vs orange (Fixed) box distributions are clearly separated.
**How to verify:** Run the recovery area boxplot cell → Y-axis now spans exactly 0–6 cm² with tick marks every 1 cm². The outlier is clipped off-screen, making the box separation visible.
**Where:** `dev_logs/analysis/experiments_analysis_summary.ipynb` Cell 12 (before `plt.tight_layout()`). New rule added to Section 4 of this skill file.*

### `### Advanced Thesis Highlights` — Comparative Performance Improvement Printouts

*[2026-06-08] **What was done:** Added three code cells that dynamically print the % improvement of Rotating Cage over Fixed Cage for: (1) recovery area (after the boxplot), (2) max deviation (after Plot B scatter), (3) impact deceleration (after the deceleration vs battery plot). Also polished Plot 12: removed the retired 3rd Settling Times panel → now 2 panels, Y-axis label now includes [rad] units, and added a markdown cell below with the LaTeX formula and plain-English explanation.
**How to verify:** Open `dev_logs/analysis/experiments_analysis_summary.ipynb`, run cells from Step 5 onward. Look for: "✅ Rotating Cage reduces recovery area by XX.X%" below the recovery boxplot, same for max deviation below Plot B, and for deceleration after the battery plots. Plot 12 shows 2 panels (not 3) and Rotational Energy Y-axis reads "Integrated Rotational Energy [rad]". Math markdown renders below the plot.
**Where:** `dev_logs/analysis/experiments_analysis_summary.ipynb` — code cells after the recovery area boxplot (cell ~13), after Plot B (cell ~33), after the deceleration cell (cell ~25). Plot 12 cell (~28) modified. New markdown cell (~29) with math.*
