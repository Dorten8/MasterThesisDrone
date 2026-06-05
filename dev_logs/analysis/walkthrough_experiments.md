# 🏁 Master Thesis Walkthrough: Experimental Collision Sweep Refinements

This document serves as our shared, checkable roadmap to refine, fix, and polish the experimental telemetry analysis summary dashboard (`experiments_analysis_summary.ipynb`) and the telemetry ingestion pipelines. You can strike through items or check off boxes (`- [x]`) as we proceed.

---

## 🛠️ Phase 1: Database & Pipeline Hotfixes

- [x] **Fix `NameError: name 'glob' is not defined`**
  * **Location:** `dev_logs/analysis/database/db_pipeline.py:644`
  * **Symptom:** Running the automated telemetry ingestion pipeline throws a `NameError` because the standard library `glob` module is used but not imported.
  * **Fix:** Added `import glob` at the top of `db_pipeline.py`.

- [x] **Adjust IMU Collision Dynamics Time Alignment**
  * **Issue:** The coordinate-based closest approach detection uses MoCap position data, which exhibits ~100 ms of latency relative to the high-frequency IMU sensor shock. This makes the vertical "Impact Line" in the plots appear slightly before/after the actual shock.
  * **Fix:** Aligned the collision timeline dynamically by scanning the IMU data in the contact window and setting $t = 0.0$ to the exact peak gradient of the accelerometer vector magnitude ($a_{\text{deviation}}$).
  * **Benefit:** Ensures physical events align perfectly across all flights.

---

## 📊 Phase 2: Plot Enhancements & Cleaning

- [x] **Resolve Kinetic Profile Plot Red Dots (`/poses` Publish Rate)**
  * **Issue:** Red dots are plotted on the kinetic curves to highlight drops in the `/poses` MoCap update rate below 30 Hz. While useful for raw debug logs, they clutter the thesis-ready summary plot.
  * **Fix:** Disabled/removed the red scatter dots in the clean non-RAW plotting function, while retaining standard interpolation to bridge any brief data gaps.

- [x] **Fix Empty Plots in Summary Notebook**
  * **Empty Figures:**
    1. *Post-Impact Deviation vs. Impact Angle (Rotating Cage)*
    2. *Post-Impact Deviation vs. Impact Angle (Fixed Cage)*
    3. *Stabilization Overlay: Rotating vs. Fixed Cage Dynamics*
  * **Root Cause:** The database queries for these plots were using categorical filters `'Rotating Cage'` and `'Fixed Cage'` that mismatched the string labels stored in the database (`'Rotating'` and `'Fixed'`).
  * **Fix:** Standardized the query filters in the plotting cells to match the database schema exactly and ensure data points are successfully loaded and plotted.

- [x] **Retire Plot 14 (Actuator Saturation Phase Portrait) & Plot 15**
  * **Action:** Removed these two inconclusive figures to clean up the dashboard.

- [ ] **Fix Plot 16 Y-Axis Explanation**
  * **Proposed Fix:** Rename the Y-axis to clearly indicate what tracking metric is being displayed (e.g., "Normalized Attitude Error [$^\circ$]") and add a brief description of how it is calculated.

- [ ] **Polish Integrated Rotational Energy (Plot 12)**
  * **Action 1:** Remove the third panel (Plot 12c: Linear & Attitudinal Settling Times) entirely. Layout will become a 2-panel figure:
    1. *Peak Acceleration (g)*
    2. *Integrated Rotational Energy (rad)*
  * **Action 2:** Add units `[rad]` to the Y-axis of the Rotational Energy plot.
  * **Action 3:** Add an explanatory printout below the plot showing the math:
    $$\text{Rotational Energy} = \int_{t_{\text{impact}} - 50\text{ms}}^{t_{\text{impact}} + 350\text{ms}} \|\vec{\omega}(t)\|_2 \, dt \quad [\text{rad}]$$
    *Explanation:* This represents the integrated shock impulse of angular velocity magnitude (rad/s) over a 400 ms contact window, representing the total angular displacement transferred into high-frequency cage rotations.

---

## 📐 Phase 3: Nominal vs. Actual Impact Geometry Visualization

- [ ] **Nominal vs. Actual Impact Geometry Visualization**
  * **Issue:** Need to visualize the actual impact angles and how they distribute relative to the nominal sweep directions.
  * **Proposed Visualization:**
    * Draw the column to-scale ($R = 4.5\text{ cm}$).
    * Draw the drone silhouette/circumference ($R = 17.9\text{ cm}$).
    * Draw nominal sweep vectors (45° and 75°).
    * For each flight, plot a line segment from the drone perimeter to the column representing the actual impact angle.
    * Overlay shaded polar wedges (histograms) whose radii represent the percentage distribution of impacts.
    * Add a summary table below this plot.

---

## 📈 Phase 4: Statistical Rigor & Metric Quantification

- [x] **Robust Trendline for Deceleration vs. Battery Plot**
  * **Action:** Created a duplicate plot that drops the heatmap colors for angles to show the global battery dependency clearly, and used a **Huber Regressor** or **Theil-Sen estimator** (from `scikit-learn`) instead of Ordinary Least Squares (OLS) to fit the trendlines.
  * **Why:** Huber loss treats residuals quadratically for small errors and linearly for large errors (outliers), preventing anomalous battery drops/sensor spikes from pulling the trendline away from the true mean.

- [x] **Investigate Command Speed Outliers at 2000 RPM**
  * **Issue:** The plot of *Peak Acceleration Z vs. Commanded Motor Speed* has outliers at 2000 RPM (minimum idle speed).
  * **Fix:** Split into two side-by-side plots (Rotating vs. Fixed Cage) and filter out timestamps where the drone speed was below threshold or state was not in active flight. Fit robust trendlines to both.

- [ ] **Quantify Trajectory Path Spread**
  * **Proposed Metric:** Calculate the **Standard Deviation of Lateral Displacement (SDLD)** along the path.
    1. Resample all trajectories to a uniform grid along the travel axis ($y$).
    2. Compute the standard deviation of the lateral coordinate ($x$) at each step.
    3. Calculate the mean SDLD over the sweep zone. A smaller SDLD indicates highly repeatable trajectories.
  * **Output:** Display this spread value directly on the plot/table.

- [ ] **Add Recovery Area Distribution Improvement Printout**
  * **Proposed Fix:** Print the percentage improvement (reduction in recovery area deviation) of the Rotating Cage vs. Fixed Cage dynamically under the plot.

- [ ] **Implement Pearson & Spearman Correlations on Plot 13 (Phase Portrait)**
  * **Proposed Fix:** Compute and display:
    1. **Pearson's $r$:** Measures the linear correlation between shock acceleration and attitude deviation.
    2. **Spearman's $\rho$:** Measures the monotonic relationship (resilient to non-linearities and outliers).

- [x] **Build Master Comparison Metrics Table**
  * **Proposed Table:** A comprehensive markdown table at the end of the notebook summarizing all metrics:
    | Metric | Rotating Cage (Mean ± SD) | Fixed Cage (Mean ± SD) | Absolute Improvement | % Improvement |
    | :--- | :--- | :--- | :--- | :--- |
    | Peak Accel (g) | | | | |
    | Rotational Energy (rad) | | | | |
    | Recovery Area (mm·m) | | | | |
    | Path Spread (mm) | | | | |

---

## 🛠️ Phase 5: ULog Matching Review & Ulog Topics Integration

- [ ] **User Review Action: Resolve Unmatched Flights**
  * **Action:** Review unmatched flights list from `ulog_matching_log.txt` (located at `dev_logs/analysis/database/ulog_matching_log.txt`) and copy the missing ones again down to the document. 
  * **Missing flights:**
    1. `flight_20260526-0922_75°_column_collision_loop_rotating_cage`
    2. `flight_20260528-1230_75°_column_collision_loop_fixed_cage`
    3. `flight_20260529-1041_45°_column_collision_loop_fixed_cage`
    4. `flight_20260530-1724_45°_column_collision_loop_fixed_cage`
    5. `flight_20260530-1727_45°_column_collision_loop_fixed_cage`
    6. `flight_20260531-1102_45°_column_collision_loop_fixed_cage`
    7. `flight_20260601-1756_45°_column_collision_loop_rotating_cage`

- [ ] **Incorporate ULog Topics**
  * **Action:** Add and utilize the following topics from `.ulg` files to make various plots:
    * `/fmu/out/actuator_outputs` (raw PWM/ESC commands)
    * `/fmu/out/vehicle_status` (flight state and arming status)
    * `/fmu/in/actuator_motors` (normalized motor control values)
    * `/tf_static` (static TF transforms)

---

## ⚙️ Phase 6: Advanced Motor Analysis & Visualizations

- [ ] **Plot 1: Control Allocator Saturation (Individual Pass Plot)**
  * **Inputs:**
    * `control_allocator_status` (`timestamp`, `actuator_saturation[0..3]`, `torque_setpoint_achieved`, `thrust_setpoint_achieved`, `unallocated_torque[0..2]`, `unallocated_thrust[0..2]`).
    * `actuator_motors` (`timestamp`, `control[0..3]`).
    * `wp_events` (for pass alignment/time window bounds).
  * **Outputs:**
    * Standalone PNG plot: `<pass_id>_control_allocator_saturation.png` in the pass directory.
    * SQLite database columns populated: `allocator_saturation_duration_sec`, `max_unallocated_torque`, `thrust_setpoint_achieved_pct`.
  * **Visual Description:**
    * Subpanel 1: Motor Command Input (`actuator_motors/control[0..3]`) to show the desired inputs (0.0 to 1.0).
    * Subpanel 2: Saturation State (`actuator_saturation[0..3]`) step plot showing `-2` (lower saturation), `0` (normal), or `2` (upper saturation). Shading will highlight the saturation duration.
    * Subpanel 3: Control Allocation Deficiency. Plot the norm of the `unallocated_torque[0..2]` vector along with binary indicators for `torque_setpoint_achieved` and `thrust_setpoint_achieved`.

- [ ] **Plot 2: PID Rate Controller Tracking (Individual Pass Plot)**
  * **Inputs:**
    * `vehicle_rates_setpoint` (`timestamp`, `roll`, `pitch`, `yaw`).
    * `vehicle_angular_velocity` (`timestamp`, `xyz[0..2]` for rollspeed, pitchspeed, yawspeed).
    * `wp_events` (for pass alignment/time window bounds).
  * **Outputs:**
    * Standalone PNG plot: `<pass_id>_pid_rate_tracking.png` in the pass directory.
    * SQLite database columns populated: `roll_rate_error_rms`, `pitch_rate_error_rms`, `yaw_rate_error_rms`.
  * **Visual Description:**
    * A 3-panel vertical layout (Roll, Pitch, Yaw rate tracking).
    * In each panel: Commanded rate (dashed line) vs. Actual rate (solid line) during the recovery window, with the absolute tracking error (|command - actual|) plotted as a shaded band at the bottom. The impact point (`WP2`) will be marked by a vertical dashed red line.

- [ ] **Plot 17: Aggregated Control Allocator Saturation**
  * **Inputs:** SQLite database columns: `condition`, `allocator_saturation_duration_sec`, `max_unallocated_torque`, `thrust_setpoint_achieved_pct`.
  * **Outputs:**
    * Saved graphic: `dev_logs/analysis/graphics/plot_17_allocator_saturation_comparison.png`.
    * Integrated cell in `experiments_analysis_summary.ipynb` showing this comparative analysis.
  * **Visual Description:**
    * A 3-panel horizontal subplot figure.
    * Panel A: Boxplot comparison of control allocator saturation duration (seconds) for Rotating vs. Fixed Cage.
    * Panel B: Boxplot comparison of maximum unallocated torque (N-m) for both configurations.
    * Panel C: Boxplot comparison of thrust setpoint achieved percentage (%) for both configurations.

- [ ] **Plot 18: Aggregated PID Rate Tracking Error (Roll, Pitch, Yaw RMS)**
  * **Inputs:** SQLite database columns: `condition`, `roll_rate_error_rms`, `pitch_rate_error_rms`, `yaw_rate_error_rms`.
  * **Outputs:**
    * Saved graphic: `dev_logs/analysis/graphics/plot_18_pid_tracking_comparison.png`.
    * Integrated cell in `experiments_analysis_summary.ipynb` showing this comparative analysis.
  * **Visual Description:**
    * A 3-panel horizontal boxplot figure (Roll, Pitch, Yaw tracking errors).
    * Each panel contains side-by-side boxplots comparing the RMS tracking error (rad/s) for Rotating vs. Fixed Cage.

- [ ] **Spatial Thrust Vector Polar Diagrams**
  * **Action:** Visualize the direction and magnitude of thrust adjustments.

- [ ] **Differential Motor Effort Heatmaps**
  * **Action:** Show which motors work hardest under specific impact angles.

---

## 💡 Phase 7: Control Theory & Hardware Explanations

- [ ] **Control Allocator Saturation Plots Explanation**
  * **Explanation to Add:**
    During aggressive recovery post-collision, the PID attitude controller commands moments that force motor mixers to their limits (minimum or maximum RPM). When saturation occurs, the drone loses the capacity to apply further control inputs in that axis.
    Comparing the saturation duration between the two enclosures shows whether the Rotating Cage allows the drone to recover using less aggressive corrections, thereby avoiding actuator saturation limits.

- [ ] **MoCap Latency Impact on PID Tracking Error**
  * **Explanation to Add:**
    The PID rate controller tracking error is artificially inflated by packet dropouts and latency in the `/poses` topic. When MoCap drops packets, the EKF state estimate lags, producing sudden step changes in the estimated orientation. The controller sees this lag as a massive tracking error and applies sharp corrections, even though the physical drone may be stable.
    Thus, the tracking error plot reflects a combination of control instability and EOC/MoCap network latency.

---

## 📐 Phase 8: Statistical Analysis & Notebook Migration

- [ ] **Statistical Aggregate Performance Analysis**
  * **Action:** Interface the statistical aggregation code (e.g., `compare_all_angles` in kinematics module) correctly with the current SQLite database state in the Jupyter environment (`dev_logs/analysis/experiments_analysis.ipynb`).

- [ ] **Automatic LaTeX PGF/TikZ Vector Graphics Generator**
  * **Action:** Refactor the PGF/TikZ graphics generator out of the main pipeline execution loop to a standalone helper script (`dev_logs/analysis/graphics/tikz_generator.py`). Freeze this generator script.

---

## 🗺️ Phase 9: Submodule Pinning & Event-Driven Telelying

- [ ] **Submodule Pinning & Event-Driven Telemetry Decoupling**
  * Re-validate that submodules (e.g. `src/mocap_px4_bridge`) are locked to their stable commits.
  * Ensure the flight director `/flight_director/active_waypoint` status is used as the primary event segmentation mechanism.
