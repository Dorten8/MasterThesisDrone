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

### `### MoCap Frame Rate & Velocity Filtering` — Fixed Cage Velocity Profile Kinks — Root Cause & Attempted Fix — FAILED & REVERTED

*[2026-06-08] **What was done:** Full diagnostic analysis and an attempted fix for persistent velocity/acceleration profile kinks in Fixed Cage flights. The attempted fix (position pre-filtering) made results worse and was reverted.*

**Attempt 1 — Position Pre-Filtering (2026-06-08) — FAILED:**
Applied a 12 Hz Butterworth low-pass to positions before SG differentiation, and relaxed the velocity Butterworth from 4 Hz → 20 Hz. **Result: way worse than original.**

**Why it failed:**
1. The 12 Hz position cutoff is too close to the ~10 Hz dropout kink frequency — barely any attenuation
2. The 4 Hz velocity Butterworth was doing ALL the smoothing work. Relaxing it to 20 Hz removed the only effective filter
3. The 12 Hz position filter doesn't compensate for the lost 4 Hz velocity filter — fundamentally different filtering stages

**What remains on disk:**
- `prefilter_position_fc=None` parameter on `compute_velocity()` (harmless, defaults off)
- Position pre-filter block commented out with failure explanation
- `_prefilter_fc` column stored in dataframe (0.0 when off)
- Notebook interactive cell reverted to original Rotating Cage example
- Filter info box reverted to original display

**Updated understanding of the problem:**
The pipeline has TWO stages that matter for Fixed Cage:
1. Linear interpolation → 100 Hz grid (creates kinks at segment boundaries)
2. SG deriv=1 → velocity (amplifies kinks)
3. 4 Hz Butterworth on velocity (blurs everything to hide the kinks)

The 4 Hz filter on velocity is a crude but effective hammer. The real challenge is: **how to smooth the kinks without smearing the collision dynamics.** The kinks are at ~10 Hz (from 100ms dropout segments), the collision is at ~4 Hz (250ms deceleration). These are barely separated in frequency. No linear filter can cleanly separate them — that's the fundamental DSP limitation.

**Potential next approaches (NOT implemented):**
- **Time-domain dropout detection:** Detect dropout segments, compute velocity within good segments using raw finite differences, bridge gaps with physically-plausible velocity (constant or linearly changing)
- **Kalman smoother:** Use a constant-acceleration motion model with process noise adapted to dropout probability — inherently band-limits velocity to physically realizable drone dynamics
- **Total Variation Regularization:** Formulate as optimization problem — find velocity signal that fits position derivatives while penalizing total variation (L1 regularization on acceleration changes), which preserves sharp collision edges while removing dropout kinks

**How to verify future attempts:** Run the interactive kinematics viewer cell in `experiments_analysis.ipynb` with a Fixed Cage flight. Compare velocity smoothness and collision edge sharpness against the original pipeline output.

**Root Cause:**
The velocity pipeline works as follows:

```
Raw MoCap /poses (irregular ~10-120Hz, median ~120Hz Rotating, as low as ~4Hz Fixed)
  → 1. Linear interpolation to 100Hz uniform grid     ← CREATES piecewise-linear kinks
  → 2. SG filter (w=19, p=3, deriv=1) on position     ← AMPLIFIES kinks into velocity spikes
  → 3. 4Hz Butterworth on velocity                    ← BLURS real collision dynamics
  → 4. Surgical ringing removal at dropout boundaries ← CREATES new kinks at boundaries
```

**Step 1 is the root cause.** Linear interpolation between irregularly-spaced MoCap samples produces a piecewise-linear position signal. Every raw data point becomes a slope discontinuity (C⁰ continuous, not C¹). When the Fixed Cage rate drops to ~10 Hz, segments are 100ms straight lines — the position looks like a connect-the-dots polygon.

**Step 2 faithfully amplifies those kinks.** The SG differentiator is designed to preserve signal features — it correctly amplifies the artificial slope discontinuities into velocity spikes. This is not an SG bug, it's working as intended.

**Steps 3-4 fight symptoms, not the cause.** A 4 Hz Butterworth on velocity is so aggressive it smears real collision deceleration dynamics (a 250ms impact spans the filter's entire passband). The surgical ringing removal at dropout boundaries creates abrupt transitions between "doctored" and "clean" segments, producing new visible kinks.

**Why previous attempts failed:**
1. The June 1 session journal proposed cubic spline interpolation instead of linear — this would give C¹ continuity at data points. But the current code comment reads *"Simple linear interpolation for all columns (preserves raw data feel, no PCHIP jitter)"* — suggesting cubic/PCHIP interpolation was tested but produced overshoot ringing at gap boundaries (Runge phenomenon with sparse data).
2. Tuning the SG window and polynomial order (w=19, p=3) only trades off noise suppression against temporal resolution — it cannot fix the fundamental issue that the position signal fed to the differentiator contains artificial high-frequency content.
3. The adaptive jitter logic (low-margin for high-dropout Fixed Cage flights) correctly identifies the problem as packet-loss related but attacks it from the wrong direction (masking ringing regions instead of preventing ringing from being generated).

**Proposed Solution — Position Pre-Filtering:**
The key physical insight: **a 1.5 kg rigid-body drone's position trajectory has negligible power above ~12 Hz.** It cannot teleport or change direction instantaneously — its dynamics are fundamentally band-limited by motor thrust (~0.5s rise time) and inertia.

The correct fix is to **low-pass filter the uniformly-resampled position signal before SG differentiation.** This removes the artificial high-frequency content from linear interpolation while preserving ALL real flight dynamics:

```python
# NEW PATH: Position Pre-Filtering (band-limit before differentiation)
# 12 Hz cutoff — band-limits position to physically realizable drone dynamics
# Prevents linear interpolation kinks from being amplified by SG deriv=1
from scipy.signal import butter, filtfilt
b_pre, a_pre = butter(2, 12.0, fs=100.0, btype='low')
for col in ['x', 'y', 'z']:
    df_mocap[col] = filtfilt(b_pre, a_pre, df_mocap[col])
```

Then SG differentiation operates on already-smooth positions → inherently smooth velocity. The existing 4 Hz velocity Butterworth can be relaxed to 15-20 Hz (light cleanup only, not primary smoothing), preserving real collision dynamics at 250ms resolution.

**Mathematical justification:**
- Drone mass ≈ 1.5 kg, max thrust ≈ 40 N → max acceleration ≈ 26.7 m/s²
- At max acceleration, the velocity changes by at most ~13 m/s in 0.5s
- For comparison: 12 Hz position filtering attenuates features below ~83 ms
- A 250 ms impact deceleration (4 Hz) is in the passband — preserved
- Artifacts from 100 ms linear interpolation segments (10 Hz) are in the stopband — removed

**Implementation Plan (route through, keep old path intact):**
1. Add a new parameter `prefilter_position_fc=None` (default = None = off) to `compute_velocity()` in `kin_calculator.py`
2. After the resampling + interpolation step, if `prefilter_position_fc` is set, apply Butterworth low-pass to x, y, z columns before SG differentiation
3. Relax the existing 4 Hz velocity filter to 20 Hz when position pre-filtering is active
4. Comment the old paths with markers, keep them intact
5. The interactive notebook cell passes `prefilter_position_fc=12.0` to route through the new path
6. Default behavior unchanged — only new calls that opt in get the new filter

**How to verify:** Run the interactive kinematics viewer cell in `experiments_analysis.ipynb` with a Fixed Cage flight. The velocity and acceleration profiles should show smooth curves with no visible kinks, while preserving real collision deceleration events. Compare side-by-side with a Rotating Cage flight — both should now look similarly smooth.

**Where:** `dev_logs/analysis/kinematics/kin_calculator.py` — `compute_velocity()` function. Notebook: `dev_logs/analysis/experiments_analysis.ipynb` — interactive cell. Plot rendering: `dev_logs/analysis/kinematics/kin_plot_kinematics.py` — filter info box.*

### `### EKF Velocity & Battery Truncation` — EKF Velocity Integration + Battery Window Fix

*[2026-06-09] **What was done:** Two changes integrated into the analysis pipeline.*

**Change 1 — EKF Velocity Integration:**
- Added `compute_ekf_kinematics(df_odom, df_mocap)` to `kin_calculator.py` — resamples PX4 EKF velocity from `vehicle_odometry` to 100 Hz, applies NED→ENU coordinate alignment, computes speed + SG-differentiated acceleration.
- Modified `calculate_metrics()` with optional `df_ekf_kin=None` parameter. When provided, velocity/acceleration columns in the MoCap DataFrame are overwritten with EKF-interpolated values before metric computation.
- Wired `df_odom → compute_ekf_kinematics → calculate_metrics()` in both pipeline sections of `db_pipeline.py`.
- Old MoCap velocity derivation path is commented out with `=== RETIRED: MoCap Velocity ===` markers.

**Change 2 — Battery Window Truncation:**
- Changed `active_flight_time_sec` and `df_bat_active` window from `arming_time → disarming_time` to `takeoff_time → disarming_time` in `db_pipeline.py` (both pipeline sections).
- Old arming_time-based code is commented out with `=== RETIRED: arming_time window ===` markers.
- Fixes artificially low drain rates caused by including ground idle time in the denominator.

**How to verify:** Run any flight through the pipeline — EKF kinematics DataFrame is printed with sample count. Compare `impact_accel` values between MoCap and EKF paths (EKF should show lower peaks, especially for Fixed Cage). For battery: pick a flight with long ground idle and check that `voltage_drop_rate` increased vs the old value.
**Where:** `kin_calculator.py` (new `compute_ekf_kinematics()` function, modified `calculate_metrics()`), `db_pipeline.py` (EKF wiring at lines ~474-482, battery truncation at lines ~585-615, secondary pipeline at lines ~1060-1090).*

Each entry records a completed implementation from the walkthrough, in italic, cross-referencing where in the walkthrough it lives.

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
