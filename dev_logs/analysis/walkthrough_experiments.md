# 🏁 Master Thesis Walkthrough: Experimental Collision Sweep Refinements

This document serves as our shared, checkable roadmap to refine, fix, and polish the experimental telemetry analysis dashboard. Its structure directly mirrors the sequence of cells in our two notebooks:
1. `experiments_analysis.ipynb` (individual flights and telemetry ingestion)
2. `experiments_analysis_summary.ipynb` (aggregate comparative analytics)

Feel free to check off boxes (`- [x]`) or add comments/complaints directly under the respective notebook sections below.

---

## 📐 1. Individual Flight Ingestion & Analysis (`experiments_analysis.ipynb`)

This section tracks tasks and pipeline hotfixes relating to raw telemetry processing, ULog matching, database generation, and individual flight pass visuals.

<details>
<summary><b>Telemetry Ingestion & Database Generation (`db_pipeline.py`)</b></summary>

<details>
<summary><code>- [x] **Fix `NameError: name 'glob' is not defined`**</code></summary>

  * **Location:** `dev_logs/analysis/database/db_pipeline.py:644`
  * **Fix:** Added `import glob` at the top of `db_pipeline.py`.
</details>

<details>
<summary><code>- [x] **Resolve Unmatched Flights**</code></summary>

DONE
</details>

<details>
<summary><code>- [x] **Incorporate ULog Topics & Flight Time/Efficiency Metrics**</code></summary>

  * **Location:** Telemetry Pipeline (`db_pipeline.py`)
  * **Action:** Extract and utilize additional `.ulg` fields in the pipeline to run comparative analysis. Currently, the database table has NULL/unpopulated values for:
    * `active_flight_time_sec` (not populated)
    * `voltage_drop_rate_v_per_min` (not populated)
    * `capacity_drain_rate_pct_per_min` (not populated)
    * `max_actuator_output` (not populated)
  * *_Review Note (Implemented & Investigating Trash Data):_*
    * *I updated `db_pipeline.py` to fix the `max_actuator_output` bug and implemented a robust global battery parser using `pyulog`.*
    * *Trash Data Investigation (Root Cause Found): For flights like `20260528-1511`, the ULog contains multiple arming sessions (e.g., a brief 6-second pre-flight test arming, followed by the actual flight arming). The initial `_get_global_battery` parser only captured the first arm/disarm pair it encountered, which was the 6.6-second test arm where no battery drain occurred. The actual flight occurred in a later arming session.*
    * *Proposed Session-Matching Fix:*
      1. *Modify `_get_global_battery` to identify all arming sessions in the ULog.*
      2. *Use the pass's PX4-aligned timestamp (`timestamp_PX4` already calculated in `get_ulog_motor_metrics`) to map the pass to the exact arming session during which it occurred.*
      3. *If no session matches the timestamp, fall back to the longest arming session (the main flight) rather than the first one.*
      4. *To avoid full DB repopulation, we can also check for `capacity_drain_rate_pct_per_min <= 0.0` or `active_flight_time_sec < 15.0` during notebook/dashboard loading and discard/filter out those specific data points dynamically, or apply the pipeline fix to only update the mismatched rows.*
</details>

<details>
<summary><code>- [x] **Filter Out Below 30 Hz Data Points for Velocity and Tangential Accel**</code></summary>

  * **Location:** `kin_calculator.py` → `compute_velocity` + `resample_and_interpolate_mocap`
  * **Action:** Eliminate derivative spikes caused by MoCap dropouts in the splined kinematic profiles while keeping curves fully connected.
  * *_Log of Attempts:_*
    * *Attempt 1 — Linear Repair (Reverted): Linearly connected positions across gaps → SG filter produced severe ringing spikes at corners.*
    * *Attempt 2 — PCHIP + NaN Mask (9pt): Shape-preserving PCHIP spline + mask 9 points each side. Reduced spike from ~12→3.7 m/s² but left boundary bumps visible.*
    * *Attempt 3 — PCHIP + NaN Mask (27pt): Expanded mask to 27 points. Flattened spikes completely but cut too much of the blue/red curves — user rejected.*
    * *Attempt 4 — PCHIP + No Mask: Removed NaN masking entirely. Lines were connected but spikes returned at full force (~9.9 m/s²). Root cause confirmed: SG `deriv=1` on positions amplifies C² curvature discontinuity at PCHIP gap boundaries.*
    * *Attempt 5 — PCHIP Analytical Derivative + SG Smoother:* Used PCHIP analytical 1st derivative for velocity, then applied SG as a smoother (deriv=0, w=31).
      * *What worked:* Successfully reduced the massive 9.9 m/s² dropout spike at t≈6.7s down to 1.6 m/s² (below the real impact peak of 2.4 m/s²).
      * *What did not work:* Spreading the smoothing window (w=31) across the C¹ boundary kinks of PCHIP created a new, visible bump at t=5.1-5.2s (a region with smaller 35ms dropouts that previously looked fine).
    * *Attempt 5b — Hermite Gap Reconstruction (Tested & Rejected):* Diagnostic tested using `CubicHermiteSpline` constrained by (P₁, V₁, P₂, V₂) at gap boundaries. Result: nearly identical to PCHIP (9.8 vs 9.9 m/s²) because PCHIP internally IS a Hermite spline. The spikes come from the curvature inside the gap, not from boundary velocity mismatch.*
    * *Attempt 6 — Targeted Gaussian Blur + SG Smooth (Rejected):* Applied a Gaussian blur (σ=8) only to the PCHIP velocity in dropout gaps. Reduced spikes nicely (1.8 m/s²) but user rejected it because PCHIP analytical derivatives introduced noticeable high-frequency jitter across the entire signal compared to raw SG filtering.
    * *Attempt 7 — Surgical Ringing Removal (ACTIVE for Rotating Cage):* Reverted to raw linear interpolation of positions and standard SG(deriv=1) to obtain clean raw velocity. We mathematically cut out the SG "ringing" spikes (gap width + `filter_half_window` margin on each side) and replace them with a straight line.
      * *Diagnostic results on Rotating Cage (Pass-02):* Spike reduced from 9.618 → 3.652 m/s²; impact peak (3.082 m/s²) perfectly preserved.
    * *Attempt 8 — Adaptive Butterworth Low-Pass Filter (ACTIVE for Fixed Cage / Jittery Flights):*
      * *Concept:* If a flight has systemic tracking degradation (>20 drops below 30Hz, typical of Fixed Cage flights), surgical masking becomes impossible without masking the entire flight. Instead, we use a 100ms threshold for large dropouts, and apply a 2nd-order 4Hz Butterworth Low-Pass Filter to the computed velocity.
      * *Results:* The Butterworth filter completely eliminates high-frequency noise from micro-gaps, revealing the true underlying physical trajectory without introducing C1/C2 corners.
    * **⚠️ STATUS: SUCCESSFULLY IMPLEMENTED**
      * Both paths are dynamically resolved based on MoCap signal quality (low jitter vs high jitter).
      * Universal titles updated to "Filtered Velocity" and filter descriptions added to the Tangential Acceleration subplots.

</details>

</details>

<details>
<summary><b>Individual Pass Visualizations (Step 2: UNIFIED SWEEP COLLISION EXPERIMENTS)</b></summary>

<details>
<summary><code>- [ ] **Plot 1: Control Allocator Saturation (Individual Pass Plot)**</code></summary>

  * **Inputs:** `control_allocator_status`, `actuator_motors`, `wp_events`.
  * **Outputs:** Standalone PNG plot: `<pass_id>_control_allocator_saturation.png` in the pass directory. SQLite database columns: `allocator_saturation_duration_sec`, `max_unallocated_torque`, `thrust_setpoint_achieved_pct`.
</details>

<details>
<summary><code>- [x] **Retire Plot 2: PID Rate Controller Tracking (Individual Pass Plot)**</code></summary>

  * **Status:** Retired. This was previously implemented and has been deleted to focus resources on aggregate performance comparisons.

</details>

---

</details>

## 📈 2. Aggregate Comparative Dashboard (`experiments_analysis_summary.ipynb`)

This section mirrors the aggregate notebook cells, starting from theoretical guidance derivations down to the final master thesis metrics table.

<details>
<summary><b>Guidance Framework, Waypoint Acceptance, and Impact Geometry</b></summary>

- **Theoretical Sub-sections:**
  * 1. PX4 Guidance Dynamics: The "Spring-Loaded Slider" on a Virtual Track
  * 2. Waypoint Acceptance and S-Curve Transitions
  * 3. Mathematical Justification of 2D Horizontal Plane Simplifications
  * 4. Trigonometric Derivation of Collision Impact Angles
<details>
<summary><code>- [ ] **Revert IMU Collision Dynamics Time Alignment**</code></summary>

  * **Location:** Telemetry Pipeline (`db_cache_imu.py` / `db_pipeline.py`) and aggregate plots.
  * **Issue:** Aligning the collision timeline to the peak gradient of $a_{\text{deviation}}$ places the alignment point $t = 0.0$ *after* the physical impact has already commenced. The original closest-approach detection based on the MoCap `/poses` topic correctly registered the moment of impact a split second *before* it becomes visible in the Tangential Acceleration plots.
  * **Action:** Revert the time-alignment logic back to the original MoCap `/poses`-based closest approach detection.
  * **Label Correction:** In the aggregate plots, rename the vertical alignment reference line. It is currently labeled `"WP2"` which is incorrect; rename it to `"Impact"`.

</details>

</details>

<details>
<summary><b>Data Loading, Filtering & Pre-processing (Steps 1 to 4)</b></summary>

- **Step 1: Base Impact-Only Loaders**
- **Step 2: Segmented Geometry & Cage Loaders (IMPACTS ONLY)**
- **Step 3: Segmented Impact Angle Ranges (IMPACTS ONLY)**
- **Step 4: Angle Ranges Sub-Split by Cage State (Rotating vs. Fixed)**

</details>

<details>
<summary><b>Global Thesis Visualizations (Step 5 to 10)</b></summary>

<details>
<summary><code>- [ ] **Verify Derivative Calculations on Kinetic Profile Plots (Red Dots Removal)**</code></summary>

  * **Location:** `experiments_analysis_summary.ipynb` -> Section: `Kinetic Profile Plots` / `kin_plot_kinematics.py`
  * **Action:** We removed the red update rate dropout scatter dots from the plots. Now, we must verify that when `/poses` update rate drops occur, the computed velocity and acceleration curves in the other two kinetic profile subplots are properly interpolated/filtered so that MoCap dropouts do not introduce derivative noise, spikes, or calculation errors.
  * *_Review Note (Pending Fixes - Analysis of Splined Spikes):_*
    * *The straight-line repair code was written under `if not resample`, meaning it only ran for the raw MoCap profile. The splined profile (`<Rotating Cage (Splined)>`) skipped it and used the default `np.interp` (linear interpolation).*
    * *Why linear interpolation still causes derivative spikes: Even though the positions across the dropout are a straight line, the boundaries where the real flight path meets the straight line form "corners" (discontinuities in the first derivative/velocity). The Savitzky-Golay filter (window size 19, or ~190ms) fits a polynomial to this corner, causing it to "ring" and overshoot. This ringing leaks into the healthy data up to 10 samples (~100ms) before and after the gap.*
    * *Proposed Implementation:*
      1. *Apply the repair to BOTH paths (inside `compute_velocity` and `resample_and_interpolate_mocap`).*
      2. *Instead of linear interpolation (`np.interp`), we will use a shape-preserving Hermite spline (PCHIP) or Cubic Spline to interpolate the dropout gaps. This guarantees that the velocity is smooth ($C^1$ continuous) at the boundary points, preventing the Savitzky-Golay filter from ringing.*
      3. *When masking the dropout region with NaNs, we will expand the mask window by 10 points (half the Savitzky-Golay window size) on each side to completely hide any numerical edge artifacts from the plots.*

</details>

<details>
<summary><code>- [x] **Retire Outdated Figures**</code></summary>

  * **Location:** `experiments_analysis_summary.ipynb`
  * **Action:** Removed the legacy *Actuator Saturation Phase Portrait (Plot 14)* and *Plot 15* to clean up the dashboard.
</details>

<details>
<summary><code>- [ ] **Polish Integrated Rotational Energy (originally Plot 12)**</code></summary>

  * **Location:** `experiments_analysis_summary.ipynb` -> `Peak Acceleration (g) / Integrated Rotational Energy / Linear & Attitudinal Settling Times` (Line 1752)
  * **Action 1:** Remove the third panel (*Linear & Attitudinal Settling Times*) entirely. The layout will become a 2-panel figure:
    1. `Peak Acceleration (g)`
    2. `Integrated Rotational Energy`
  * **Action 2:** Add units `[rad]` to the Y-axis of the Rotational Energy plot.
  * **Action 3:** Add an explanatory printout below the plot showing the math:
    $$\text{Rotational Energy} = \int_{t_{\text{impact}} - 50\text{ms}}^{t_{\text{impact}} + 350\text{ms}} \|\vec{\omega}(t)\|_2 \, dt \quad [\text{rad}]$$
    *Explanation:* This represents the integrated shock impulse of angular velocity magnitude (rad/s) over a 400 ms contact window, representing the total angular displacement transferred into high-frequency cage rotations.
- **Step 7 & 8: Deviation vs. Impact Angle color-coded by Battery State** (Rotating vs. Fixed Cage)
- **Step 9: Comparative Stabilization Overlay (Rotating vs. Fixed Cage)**
- **Step 10: Battery & Flight Efficiency Comparative Analysis**
  * Outputs `### 📊 Enclosure Comparative Energy Metrics Summary`

</details>

</details>

<details>
<summary><b>Battery, Deceleration & Structural Dynamics (Steps 11 to 12)</b></summary>

<details>
<summary><code>- [x] **Robust Trendline for Deceleration vs. Battery Plot**</code></summary>

  * **Location:** `experiments_analysis_summary.ipynb` -> `Maximum Impact Deceleration vs. Start Battery Percentage` and `Global Comparison: Deceleration vs. Battery State (Angle-Free)`
  * **Fix:** Used a **Huber Regressor** or **Theil-Sen estimator** instead of OLS to fit the trendlines.
</details>

<details>
<summary><code>- [ ] **Unify Axis Scales and Overlay for Motor Speed vs Acceleration**</code></summary>

  * **Location:** `experiments_analysis_summary.ipynb` -> `IMU Peak Acceleration Z vs Commanded Motor Speed (RPM)` (Line 1927)
  * **Action 1:** Enforce a unified Y-axis scale across both subplots (Rotating vs. Fixed Cage).
  * **Action 2:** Set a unified X-axis scale ranging from `8600` to `10800` RPM, with divisions of `200` (ticks every 200 RPM).
  * **Action 3:** Generate an alternative plot in this section where the Rotating Cage and Fixed Cage values are overlaid on a single, unified plot, rather than separate subplots, for direct comparison.
</details>

<details>
<summary><code>- [ ] **Investigate Command Speed Outliers at 2000 RPM**</code></summary>

  * **Location:** `experiments_analysis_summary.ipynb` -> `IMU Peak Acceleration Z vs Commanded Motor Speed (RPM)` (Line 1927)
  * **Issue:** The plot has outliers at 2000 RPM (minimum idle speed).
  * **Action:** Investigate whether these occur due to a mismatch in the collision time window (capturing pre-takeoff/post-landing states) or a failed collision detection where the drone hovered without rotor engagement. Filter out timestamps where the drone speed was below threshold or state was not in active flight.
- **Step 12: Enclosure Structural Dynamics**
  * Outputs `### Enclosure Comparative Table of Averages`

</details>

</details>

<details>
<summary><b>Advanced Thesis Highlights</b></summary>

<details>
<summary><code>- [ ] **Add Recovery Area & Comparative Performance Improvement Printouts**</code></summary>

  * **Location:** `experiments_analysis_summary.ipynb` -> Section: `📐 Trajectory Recovery Area Metric Comparison` (Line 470), `Post-Impact Maximum Deviation vs. Impact Angle` (Line 2032), `Maximum Impact Deceleration vs. Start Battery Percentage` (Line 1255).
  * **Proposed Fix:** Dynamically print the percentage improvement (reduction in recovery area, deviation, or deceleration) of the Rotating Cage vs. Fixed Cage configuration directly below/under each plot where a clear comparative metric is shown.
- **Plot A: IMU Z vs Commanded Motor Speed (RPM)**
- **Plot B: Impact Angle vs. Max Deviation**
- **Plot C: Recovery Area Distribution**
- **Plot D: 2D Path Overlay & Obstacle Clearance**
<details>
<summary><code>* [ ] **Nominal vs. Actual Impact Geometry Visualization**</code></summary>

    * Draw the column to-scale ($R = 4.5\text{ cm}$).
    * Draw the drone silhouette/circumference ($R = 17.9\text{ cm}$).
    * Draw nominal sweep vectors (45° and 75°).
    * For each flight, plot a line segment from the drone perimeter to the column representing the actual impact angle.
    * Overlay shaded polar wedges (histograms) whose radii represent the percentage distribution of impacts.
    * Add a summary table below this plot.
</details>

<details>
<summary><code>* [ ] **Quantify Trajectory Path Spread (SDLD)**</code></summary>

    * **Proposed Metric:** Calculate the **Standard Deviation of Lateral Displacement (SDLD)** along the path:
      1. Resample all trajectories to a uniform grid along the travel axis ($y$).
      2. Compute the standard deviation of the lateral coordinate ($x$) at each step.
      3. Calculate the mean SDLD over the sweep zone. A smaller SDLD indicates highly repeatable trajectories.
    * **Output:** Display this spread value directly on the plot/table.
- **Plot 13: Attitude-Shock Phase Portrait**
</details>

<details>
<summary><code>* [ ] **Implement Pearson & Spearman Correlations on Phase Portrait**</code></summary>

    * **Proposed Fix:** Compute and display:
      1. **Pearson's $r$:** Measures the linear correlation between shock acceleration and attitude deviation.
      2. **Spearman's $\rho$:** Measures the monotonic relationship (resilient to non-linearities and outliers).
- **Plot 16: Post-Impact Raw IMU Oscillation Spread**
</details>

<details>
<summary><code>* [ ] **Fix `Post-Impact Y-Axis Vibration Spread` (originally Plot 16) Y-Axis & Introduce IMU Spread Metric**</code></summary>

    * **Proposed Fix:** Rename the Y-axis to clearly indicate what tracking/vibration metric is being displayed (e.g., `"Normalized Attitude Error [deg]"`) and add a brief description of how it is calculated.
    * **New Metric (IMU Acceleration Spread):** Compute and compare the typical spread (standard deviation) of high-frequency IMU acceleration in the X, Y, and Z axes within:
      1. The **Impact Window** (from $t_{\text{impact}} - 50$ ms to $t_{\text{impact}} + 350$ ms).
      2. **Regular (Steady State) Flight** (the 1.0 second window before impact).
    
    * **Visualization:** Generate boxplots or error-bar comparisons to visualize this spread difference between Rotating and Fixed Cage configurations.
    * **Notebook Integration:** Render and display the generated plot (`plot_16_imu_vibration_spread.png`) inside the `experiments_analysis_summary.ipynb` notebook.

</details>

</details>

<details>
<summary><b>Statistical Aggregate Performance Analysis (Section 3)</b></summary>

<details>
<summary><code>- [ ] **Statistical Aggregate Performance Analysis**</code></summary>

  * **Location:** `experiments_analysis_summary.ipynb` -> Section 3
  * **Action:** Incorporate statistical significance tests (e.g., T-test, Wilcoxon rank-sum) directly into comparative plots, marking significant differences with standard notation (e.g., `*`, `**`, `n.s.`).
</details>

<details>
<summary><code>- [ ] **Control Allocator Performance Comparison (originally Plot 17)**</code></summary>

  * **Outputs:** Saved graphic: `dev_logs/analysis/graphics/plot_17_allocator_saturation_comparison.png`.
</details>

<details>
<summary><code>- [ ] **PID Rate Controller Tracking Error Comparison (originally Plot 18)**</code></summary>

  * **Outputs:** Saved graphic: `dev_logs/analysis/graphics/plot_18_pid_tracking_comparison.png`.
</details>

<details>
<summary><code>- [ ] **Spatial Thrust Vector Polar Diagrams**</code></summary>

</details>

<details>
<summary><code>- [ ] **Differential Motor Effort Heatmaps**</code></summary>

</details>

</details>

<details>
<summary><b>Aggregated IMU Dynamics & Summary (Section 4)</b></summary>

<details>
<summary><code>- [ ] **Integrate Structural Dynamics Boxplots**</code></summary>

  * **Outputs:** Saved graphic: `dev_logs/analysis/graphics/structural_dynamics_boxplots.png`.
  * **Notebook Integration:** Render and display this plot (comparing peak reaction/deceleration forces between configurations) in Section 4 of `experiments_analysis_summary.ipynb`.
  * *_Review Note: I appended a new Markdown cell to the very end of `experiments_analysis_summary.ipynb` that properly embeds this PNG image for your review._*
- **Plot 19: Aggregated IMU Collision Dynamics**
  * **Requirement:** The vertical impact line must be aligned same as in the Thesis Flight Kinetic Profile (a smidgen before the acceleration starts). It should not be hard-coded at the peak, but rather represent the average of when the physical impact is registered. Update the time alignment logic in the plotting script to reflect this.
</details>

<details>
<summary><code>- [ ] **Build High-Fidelity Expanded Master Comparison Metrics Table**</code></summary>

  * **Proposed Table:** A comprehensive markdown table segmenting all key performance metrics by Enclosure Configuration and Impact Angle. Ensure all entries are populated with precise values, units, absolute differences, and percentage improvements where applicable.
  * **Notebook Integration:** Dynamically generate and print this 20-row comparison table directly within a Python cell at the end of the `experiments_analysis_summary.ipynb` notebook using the real database values. The table must be outputted as a formatted plain-text/monospace printout.
  * *_Review Note (Pending Fixes):_*
    * *The injected code cell at the end of `experiments_analysis_summary.ipynb` currently raises a `SyntaxError: unterminated string literal` on line 85 due to a malformed `print` statement. We will patch this syntax error in the notebook.*
    * **Metrics to include:**
      1. Peak Deceleration Z (g)
      2. Maximum Attitude Deviation Magnitude (m)
      3. Integrated Rotational Energy (rad)
      4. Max Actuator Output (%)
      5. Average Commanded Motor Speed (RPM)
      6. Voltage Drop Rate (V/min)
      7. Capacity Drain Rate (%/min)
      8. Active Flight Time (s)
      9. Recovery Area (mm·m)
      10. Path Spread / SDLD (mm)
      11. Post-Impact Yaw Drift (deg)
      12. Attitude Rate Roll RMS Tracking Error (rad/s)
      13. Attitude Rate Pitch RMS Tracking Error (rad/s)
      14. Attitude Rate Yaw RMS Tracking Error (rad/s)
      15. Control Allocator Saturation Duration (s)
      16. Max Unallocated Torque (N·m)
      17. Thrust Setpoint Achieved Percentage (%)
      18. IMU Acceleration X-axis Spread (g) (Impact vs. Regular)
      19. IMU Acceleration Y-axis Spread (g) (Impact vs. Regular)
      20. IMU Acceleration Z-axis Spread (g) (Impact vs. Regular)
    * **Segmentation Columns:**
      * Overall (All Angles): `Rotating Cage (Mean ± SD)` | `Fixed Cage (Mean ± SD)` | `% Improvement`
      * 45° Impact Angle Subset: `Rotating Cage (Mean ± SD)` | `Fixed Cage (Mean ± SD)` | `% Improvement`
      * 75° Impact Angle Subset: `Rotating Cage (Mean ± SD)` | `Fixed Cage (Mean ± SD)` | `% Improvement`

</details>

</details>

<details>
<summary><b>Control Theory & Hardware Explanations</b></summary>

<details>
<summary><code>- [ ] **Control Allocator Saturation Plots Explanation**</code></summary>

</details>

<details>
<summary><code>- [ ] **MoCap Latency Impact on PID Tracking Error**</code></summary>

</details>

</details>

<details>
<summary><b>Code Automation & LaTeX Migration</b></summary>

<details>
<summary><code>- [ ] **Automatic LaTeX PGF/TikZ Vector Graphics Generator**</code></summary>

  * **Location:** Standalone helper script (`dev_logs/analysis/graphics/tikz_generator.py`).
</details>


<details>
<summary><code>- [ ] **Submodule Pinning & Event-Driven Decoupling**</code></summary>

</details>

</details>

# Finishing touches to do:
<details>
<summary><code>- [ ] **Make sure no trash data in .db**</code></summary>

  
</details>

</details>
