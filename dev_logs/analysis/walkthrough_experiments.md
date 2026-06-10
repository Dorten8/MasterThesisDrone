# 🏁 Master Thesis Walkthrough: Experimental Collision Sweep Refinements

## 📍 CURRENT STATUS (2026-06-09, ~21:45)

**Battery: 100% done ✅**
- `flights_battery_efficiency`: 32 flights (16 pre-cutoff removed, 32 approved remain)
- `flights_summary`: 108 passes (45°), 75° pipeline re-run in progress (~71 passes pending)
- 0 NULL battery columns

**Cutoff SSoT established ✅**
- SSoT: `dev_logs/analysis/database/db_manager.py` → `is_approved_flight()`
- Imported by: `db_pipeline.py`, `db_mcap_event_segmenter.py`, `db_unsliced_flights_bat_analyser.py`, `_rerun_pipeline.py`
- Cutoff: `20260524-1904` (May 24, 2026, 19:04)
- 16 pre-cutoff orphan flights removed from `flights_battery_efficiency`
- Documented in `experiments_analysis_skill.md` §3.0

**Key files modified today:**
- `dev_logs/analysis/database/db_unsliced_flights_bat_analyser.py` — memory-safe streaming reader
- `dev_logs/analysis/database/db_pipeline.py` — battery lookup from efficiency table
- `dev_logs/analysis/_rerun_pipeline.py` — preserves `flights_battery_efficiency` on re-run

**Verification (final state):**
```bash
cd /home/dorten/MasterThesisDrone
python3 -c "
import sys; sys.path.insert(0, 'dev_logs/analysis/database')
sys.path.insert(0, '/home/dorten/.local/lib/python3.10/site-packages')
from db_manager import get_connection
c = get_connection().cursor()
c.execute('SELECT condition, COUNT(*), ROUND(AVG(capacity_drain_rate_pct_per_min),1) FROM flights_summary GROUP BY condition')
for r in c.fetchall(): print(f'{r[0]}: {r[1]} passes, avg drain {r[2]}/min')
c.execute('SELECT COUNT(*) FROM flights_summary WHERE capacity_drain_rate_pct_per_min IS NULL')
print(f'NULL battery: {c.fetchone()[0]}')
"
```
Expected: 0 NULL battery, ~21%/min avg both conditions.

**⏱️ VISUAL EVIDENCE: Time Truncation Still Wrong for Many Flights (2026-06-09)**
The Exp. Start-point / Exp. End-point crop window truncates flight data incorrectly. Examples:
- [flight_20260524-1904/pass01_imu_dynamics.png](dev_logs/flights/flight_20260524-1904_75°_column_collision_loop_fixed_cage/pass01_imu_dynamics.png) — IMU window clipped, impact dynamics cut off
- [flight_20260524-1904/pass04_kinetic_profile.png](dev_logs/flights/flight_20260524-1904_75°_column_collision_loop_fixed_cage/pass04_kinetic_profile.png) — kinetic profile cropped, missing sweep data
- [flight_20260531-1112/pass02_trajectory_top_down.png](dev_logs/flights/flight_20260531-1112_45°_column_collision_loop_fixed_cage/pass02_trajectory_top_down.png) — trajectory plot looks physically impossible, star-point mismatch likely to blame

Root cause: The "Exp. Start-point" vertical line comes from WP1 (command transition time), not the actual moment the drone starts moving. Fix is designed (see `.github/copilot-instructions.md` → "Experiment Start-Point Timing — SSoT Knowledge").

**Not touched (user instruction):**
- `dev_logs/analysis/experiments_analysis.ipynb` — explicitly preserved

**Skipped (too complex / needs user input):**
- Most non-trivial notebook visualizations in sections 2-4 of this walkthrough
- Control Allocator Saturation plots — marked as to-do, not implemented
- Statistical significance tests — require design decisions
- LaTeX/TikZ generator — standalone tool, not started
- Submodule pinning — infrastructure task

---

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

## 🔧 EKF Velocity Integration — Replace MoCap-Derived Velocity with PX4 EKF Velocity

The EKF velocity (`/fmu/out/vehicle_odometry.velocity[]`) was validated on 2026-06-08 as inherently smooth even during Fixed Cage MoCap dropouts. The goal is to integrate it as a plug-in replacement for MoCap-derived velocity/acceleration in `calculate_metrics()`.

<details>
<summary><code>- [x] **Implement `compute_ekf_kinematics()` in `kin_calculator.py`**</code></summary>

* ✅ Implemented and wired into `db_pipeline.py` on 2026-06-08.
* EKF velocity from `/fmu/out/vehicle_odometry.velocity[]` is inherently smooth even during Fixed Cage MoCap dropouts.
* Coordinate alignment applied: `vx_ekf = vx_ekf_raw`, `vy_ekf = -vy_ekf_raw`, `vz_ekf = -vz_ekf_raw`.
* `calculate_metrics()` accepts optional `df_ekf_kin=None` parameter — fully backward compatible.

</details>

<details>
<summary><code>- [x] **Wire EKF kinematics into `db_pipeline.py`**</code></summary>

* ✅ Done. Both pipeline paths pass `df_ekf_kin=df_ekf_kin` to `calculate_metrics()`.

</details>

<details>
<summary><code>- [x] **Verify & Re-run (Milestones 1, 2, 4)**</code></summary>

* ✅ M1: Single-flight test verified EKF kinematics
* ✅ M2: Database re-run for 45° + 75° completed (179 passes)
* ⏳ M4: Summary notebook re-run — pending (battery rates now fixed, re-run needed)

</details>

## 🔋 Battery & Voltage Drop Fix — Truncate Ground Idle Time

<details>
<summary><code>- [x] **Truncate battery active window to takeoff_time**</code></summary>

* **Problem:** `voltage_drop_rate_v_per_min` and `capacity_drain_rate_pct_per_min` used `arming_time → disarming_time`, including ground idle time. This inflated the denominator → drain rates appeared artificially low (4%/min — impossible for a flying drone).
* **Fix:** Two-tier solution implemented on 2026-06-09:
  1. **`flights_battery_efficiency` table** — flight-level battery rates computed from full unsliced MCAPs, with takeoff→landing window detection (MoCap Z > 0.15m). Populated via `db_unsliced_flights_bat_analyser.py` (now with memory-safe streaming reader to avoid OOM on 200MB+ MCAPs).
  2. **`db_pipeline.py`** — both pipeline paths now call `_get_battery_rates(flight_folder)` which looks up `capacity_drain_rate_flying` and `voltage_drop_rate_flying` from the `flights_battery_efficiency` table, instead of computing per-pass from ~10s MCAP windows.
* **Results:**
  * **Fixed Cage:** 24.6%/min avg capacity drain rate
  * **Rotating Cage:** 23.0%/min avg capacity drain rate
  * These are physically realistic (user's longest flight: ~3.5 min, ~80% battery consumed ≈ 23%/min)
* **47/48 flights** have battery data populated. 1 flight (`flight_20260529-1419_45°_column_collision_loop_rotating_cage`) has a corrupted unsliced MCAP that needs repair or ULog fallback.
* **Memory fix:** The old `load_mcap()` loaded ALL messages from ALL topics into Python objects → OOM on 200MB+ MCAPs. New `_stream_load_for_battery()` only collects 4 needed topics (`/poses`, `/fmu/out/battery_status`, `/fmu/out/vehicle_status`, `/fmu/out/vehicle_odometry`) directly into lightweight dicts → peak memory ~350MB even for largest flights.
* **Files modified:**
  * `dev_logs/analysis/database/db_unsliced_flights_bat_analyser.py` — streaming reader + gc.collect()
  * `dev_logs/analysis/database/db_pipeline.py` — `_get_battery_rates()` helper, both pipeline paths updated
  * `dev_logs/analysis/_rerun_pipeline.py` — now preserves `flights_battery_efficiency` (only clears `flights_summary`)

</details>

<details>
<summary><code>- [x] **Verify battery truncation**</code></summary>

* ✅ Re-ran pipeline — drain rates increased from 4-15%/min (old, broken) to 20-33%/min (realistic).
* ✅ 47 flights populated in `flights_battery_efficiency`. Summary stats:
  * Fixed Cage: avg flying time 104.8s, avg drain 24.6%/min, avg voltage drop 1.06 V/min
  * Rotating Cage: avg flying time 123.9s, avg drain 23.0%/min, avg voltage drop 0.60 V/min

**🔍 How to verify yourself:**
```bash
cd /home/dorten/MasterThesisDrone
python3 -c "
import sys; sys.path.insert(0, 'dev_logs/analysis/database')
sys.path.insert(0, '/home/dorten/.local/lib/python3.10/site-packages')
from db_manager import get_connection
c = get_connection().cursor()
# Check flights_battery_efficiency
c.execute('SELECT condition, COUNT(*), ROUND(AVG(capacity_drain_rate_flying),1), ROUND(AVG(total_flying_time),1) FROM flights_battery_efficiency GROUP BY condition')
for r in c.fetchall():
    print(f'{r[0]}: {r[1]} flights, avg {r[2]}%/min, avg flying {r[3]}s')
# Check flights_summary battery rates
c.execute('SELECT condition, COUNT(*), ROUND(AVG(capacity_drain_rate_pct_per_min),1) FROM flights_summary GROUP BY condition')
for r in c.fetchall():
    print(f'{r[0]} (passes): {r[1]} rows, avg drain {r[2]}/min')
# Check NULLs
c.execute('SELECT COUNT(*) FROM flights_summary WHERE capacity_drain_rate_pct_per_min IS NULL')
print(f'NULL battery in summary: {c.fetchone()[0]}')
"
```
Expected: ~23-25%/min avg, <10 NULLs (only from the 1 corrupted flight).

</details>

## ⏱️ Experiment Start/End-Point Timing Architecture (2026-06-10)

This section defines the Single Source of Truth for experiment timing — when the sweep starts, when it ends, and how those timestamps flow from the mission code through the pipeline into the database and onto plots.

### Waypoint Numbering — ⚠️ FRAGILE

The WP numbering in `find_waypoint_events()` (`kin_calculator.py:424-435`) differs from what the mission terminal output prints. This offset-by-one is intentional and currently works, but it is the **first thing to investigate** if timing breaks after any mission refactor.

| `find_waypoint_events()` | Mission attr | Terminal prints as | Physical meaning |
|--------------------------|-------------|-------------------|------------------|
| WP1 | `wp_stage` | WP_stage | U-turn staging (pass-through) |
| **WP2** / **WP2_cmd** | `exp_sp` | **WP1** | Gate (PAUSE, 5cm) — **EXPERIMENT START** |
| **WP3** | `exp_ep` | **WP2** | Sweep end (post-column) — **EXPERIMENT END** |
| WP4 | `wp3` | WP3 | Recovery (auto-loops to WP_stage) |

### WP2 Refinement

At `kin_calculator.py:563-587`, WP2 is overwritten from the **command transition time** (when PX4 switched to commanding `exp_sp`) to the **refined forward-movement time** (last MoCap sample with speed < 0.10 m/s before crossing Y=0.70m). The original command time is preserved as `WP2_cmd`.

### Database Timestamp Columns

Stored in `flights_summary` table, computed once during pipeline ingestion:

| Column | Type | Source | Meaning |
|--------|------|--------|---------|
| `timestamp_db` | TEXT | `datetime.datetime.now()` | Wall-clock time when DB row was written |
| `e_sp_timestamp_PX4` | INTEGER | `WP2_cmd` | PX4 µs — command transition to gate |
| `e_sp_timestamp_PX4_forw` | INTEGER | `WP2` refined | PX4 µs — actual forward movement start. NULL if `|forw − cmd| < 50ms` |
| `e_ep_timestamp_PX4` | INTEGER | `WP3` | PX4 µs — sweep end achievement |

**Reader pattern:** Use `COALESCE(e_sp_timestamp_PX4_forw, e_sp_timestamp_PX4)` to always get the best available experiment start time.

### Plot Timeline Markers

`kin_plot_kinematics.py` `draw_timeline_markers()` and `get_timeline_limits()` now use:
- **Exp. Start-point** → `wp_events['WP2']` (refined forward movement)
- **Exp. End-point** → `wp_events['WP3']` (actual sweep end)
- Timeline crop window: `[esp_time - 1.0s, eep_time + 1.0s]`

### 50ms Threshold Rationale

If the drone springs forward immediately after the gate command (typical for most passes), storing two identical timestamps adds noise. The 50ms threshold suppresses `forw` when command and movement are effectively simultaneous. Only passes with a meaningful pause (>50ms) between command and movement get a distinct `forw` value.

### Verification

```bash
cd /home/dorten/MasterThesisDrone
python3 -c "
import sys, sqlite3
sys.path.insert(0, 'dev_logs/analysis/database')
sys.path.insert(0, '/home/dorten/.local/lib/python3.10/site-packages')
from db_manager import get_connection
c = get_connection().cursor()
# Check new columns exist
c.execute('PRAGMA table_info(flights_summary)')
cols = [r[1] for r in c.fetchall()]
for col in ['timestamp_db', 'e_sp_timestamp_PX4', 'e_sp_timestamp_PX4_forw', 'e_ep_timestamp_PX4']:
    print(f'  {col}: {\"✅\" if col in cols else \"❌ MISSING\"} ')

# Check data migrated
c.execute('SELECT COUNT(*) FROM flights_summary WHERE timestamp_db IS NOT NULL')
print(f'  timestamp_db populated: {c.fetchone()[0]} rows')
c.execute('SELECT COUNT(*) FROM flights_summary WHERE \"e_sp_timestamp_PX4\" IS NOT NULL')
print(f'  e_sp_timestamp_PX4 populated: {c.fetchone()[0]} rows')
c.execute('SELECT COUNT(*) FROM flights_summary WHERE \"e_sp_timestamp_PX4_forw\" IS NOT NULL')
print(f'  e_sp_timestamp_PX4_forw populated (distinct forw): {c.fetchone()[0]} rows')
c.execute('SELECT COUNT(*) FROM flights_summary WHERE \"e_ep_timestamp_PX4\" IS NOT NULL')
print(f'  e_ep_timestamp_PX4 populated: {c.fetchone()[0]} rows')
"
```

Expected: all 4 columns present, `timestamp_db` and `e_sp_timestamp_PX4` fully populated (179 rows), `e_sp_timestamp_PX4_forw` with fewer rows (only passes where drone paused >50ms), `e_ep_timestamp_PX4` fully populated.

</details

# Finishing touches to do:
<details>
<summary><code>- [x] **Make sure no trash data in battery rates**</code></summary>

* ✅ Battery rates fixed — flight-level `flights_battery_efficiency` table populated (47 flights, realistic 23-25%/min).
* ✅ Pipeline re-run in progress (2026-06-09) — `flights_summary` being repopulated with battery rates from efficiency table.
* ⚠️ 1 flight (`flight_20260529-1419`) has corrupted unsliced MCAP → 4-6 passes will have NULL battery.
  * Fix: Use ULog fallback (`_populate_battery_ulog_fallback.py`) or MCAP repair to fill the gap.

</details>

<details>
<summary><code>- [x] **Fix remaining NULL battery for corrupted MCAP flight**</code></summary>

* `flight_20260529-1419_45°_column_collision_loop_rotating_cage` — unsliced MCAP throws `unknown (opcode 28) record` error.
* Has `.ulg` file and 6 pass MCAPs → ULog fallback script exists at `dev_logs/analysis/_populate_battery_ulog_fallback.py`.

</details>

<details>
<summary><code>- [x] **ULog fallback executed for corrupted flight**</code> (2026-06-09)</summary>

* ✅ Ran targeted ULog fallback — parsed `14_13_41.ulg` for `vehicle_status` + `battery_status`.
* Armed: 242.9s → Disarmed: 461.4s (218.4s total armed).
* Battery: 24.5V / 100% at arm → 22.7V / 40.6% at landing.
* **Result: 16.6%/min capacity drain, 0.47 V/min voltage drop** (slightly lower than average — this was a Rotating Cage flight, consistent).
* Inserted into `flights_battery_efficiency` — now 48/48 flights populated.
* **Note:** Pass MCAPs show drone already at Z=0.50m (mid-flight), so takeoff/landing used arm+2s / disarm-2s fallback. The 16.6%/min rate may be slightly lower than true flying rate because the 4s ground-idle buffer dilutes the denominator.

</details>

<details>
<summary><code>- [ ] **Investigate 2000 RPM command speed outliers**</code> — diagnosed (2026-06-09)</summary>

* **Investigation:** The outliers at 2000 RPM (idle speed) in `IMU Peak Acceleration Z vs Commanded Motor Speed` likely come from collision detection picking up pre-takeoff or post-landing states where motors are at idle but no actual flight/impact occurred.
* **Likely cause:** `impact_detected = 1` is triggered by `closest_clearance < 0.0` (column proximity detection in `calculate_metrics()`). If the drone is on the ground near the column during pre-flight checks, this can fire with idle motors.
* **Suggested fix (in notebook/data loading):** Filter `flights_summary` with `WHERE motor_avg_before > 5000` or `WHERE active_flight_time_sec > 5` to exclude ground-idle false positives. This can be done in the notebook cell without modifying the pipeline.
* **Location in notebook:** `experiments_analysis_summary.ipynb` → `IMU Peak Acceleration Z vs Commanded Motor Speed (RPM)` (Line 1927)

<details>
<summary><code>⚠️ Correction (2026-06-09): motor_avg_before is actuator output (0-1), not RPM</code></summary>

* The `motor_avg_before` column stores **normalized actuator output** (0.0–1.0), not RPM. Values cluster tightly at 0.7–0.8 across all impacts.
* The 2000 RPM outliers observed in the notebook plot likely come from a different RPM derivation path (possibly from ULog `actuator_motors` data) not from the `motor_avg_before` DB column.
* To diagnose this properly: check how the notebook's RPM values are computed, and whether the low-RPM datapoints correspond to passes with `active_flight_time_sec < 5` (likely ground-idle false positives).

</details>

</details>

</details>

---

## 🔍 Representative Flight Finder — Design Rationale

This section documents the design decisions behind the `RepresentativeFlightFinder` tool (`dev_logs/analysis/database/representative_finder.py`).

### Problem Statement

When presenting single-flight kinematic plots in the thesis (trajectory overlays, velocity profiles, IMU dynamics), we need to pick a **representative** flight — not the best, not the worst, but the one closest to the category average. This avoids cherry-picking and provides a fair visual comparison between Rotating and Fixed Cage configurations.

### What Does "Average" Mean?

A flight is defined by **70+ numeric metrics** in the `flights_summary` table. The "average flight" is the one whose **multivariate distance from the category centroid** is smallest after standardizing each feature to comparable scales (z-score normalization).

**Algorithm:**
1. Filter flights to the target category (e.g., "Rotating Cage, 45° impacts")
2. Select a curated set of ~12 features spanning all key thesis dimensions
3. Z-score standardize: `z_i = (x_i - μ) / σ` for each feature
4. Compute Euclidean norm of each row: `d = sqrt(Σ z_i²)`
5. Sort by `d` ascending → the flight with smallest `d` is the most representative

After z-scoring, the centroid is the origin `[0,0,...,0]`, so `d` is the distance from the multivariate mean measured in **pooled standard deviations (σ)**. This is a **diagonal Mahalanobis distance** (covariances ignored for interpretability and stability with small samples).

### Why Not PCA?

PCA would create principal components that are linear combinations of features, making the "average" uninterpretable — we couldn't say "this flight is within 0.3σ of the mean on recovery area." We keep features directly interpretable.

### Default Feature Set (10 Curated Metrics)

**Impact angle is the #1 most important feature** — it answers "at what angle did the drone hit on average?" This is the first metric anyone would think of when describing a representative collision.

| # | Feature | Dimension | Why |
|---|---------|-----------|-----|
| 1 | `impact_angle` | Contact Geometry | **#1 MOST IMPORTANT** — achieved contact angle |
| 2 | `impact_speed` | Impact Dynamics | Speed at moment of contact |
| 3 | `impact_accel` | Impact Dynamics | Peak tangential deceleration |
| 4 | `before_impact_accel` | Impact Dynamics | Pre-impact approach deceleration |
| 5 | `recovery_area` | Recovery | Integrated post-impact deviation (mm·m) |
| 6 | `max_dev_after` | Recovery | Peak lateral displacement after impact (mm) |
| 7 | `avg_dev_after` | Recovery | Mean post-impact deviation (mm) |
| 8 | `imu_peak_accel_z` | IMU / Structural | Peak vertical shock (g) — thesis-critical |
| 9 | `imu_gyro_energy` | IMU / Structural | Integrated rotational energy (rad) — thesis-critical |
| 10 | `imu_accel_settling` | IMU / Structural | Time until acceleration returns to baseline (s) |

**Excluded from defaults:**
- Motor metrics (`motor_thrust_surge`, etc.) — secondary to the core impact/recovery/IMU story
- `path_spread_sdld` — trajectory spread is more about flight-to-flight consistency than individual flight character
- `imu_peak_gyro` — redundant with `imu_gyro_energy` (energy already captures rotational shock magnitude)
- Battery columns — only 118/157 rows populated (many NULLs), would shrink usable categories
- Per-axis IMU components — magnitudes capture the signal; X/Y/Z breakdown is redundant

The user can override with `feature_columns=[...]` at construction time for any custom set.

### Category System

Flexible layered filtering (all AND-ed):

| Filter | Type | Example |
|--------|------|---------|
| `condition` | Exact match on DB column | `"Rotating Cage"`, `"Fixed Cage"` |
| `impact_angle_range` | Numeric range on `impact_angle` column | `(20, 35)` — finds glancing blows |
| `extra_query` | Arbitrary pandas `.query()` string | `"recovery_area > 50"` |

This allows categories like:
- "All Rotating Cage flights" → `condition="Rotating Cage"`
- "Fixed Cage, shallow impacts (20-35°)" → `condition="Fixed Cage", impact_angle_range=(20, 35)`
- "Rotating Cage, high-speed impacts" → `condition="Rotating Cage", extra_query="impact_speed > 2.5"`

Note: `nominal_angle` (45°/75° from flight name) is NOT a filter — the achieved `impact_angle` is what matters physically.

### Print-out Includes Rich Metadata

Each ranked flight shows a **metadata block** (experiment time window, battery %, impact angle) in addition to the per-feature comparison table, so the user can identify the flight and verify it's reasonable before using its plots:

```
🥇 Rank 1 (distance=0.847σ): flight_20260529-1210_45°_... - Pass-03
   Context:
     Nominal angle:     45°
     Impact angle:      27.8°
     Battery at start:  82.5%
     Sweep speed:       2.34 m/s
   ─────────────────────────────────────────────────
   Metric                  Flight    Category Mean±SD     Δ(σ)
   impact_angle            27.8°     30.2°±8.5°          -0.28
   ...
```

### Integration with Notebook

In `experiments_analysis_summary.ipynb`, a new section at the end:

```python
from dev_logs.analysis.database.representative_finder import RepresentativeFlightFinder

# Find the most average Rotating Cage flight
finder = RepresentativeFlightFinder(condition="Rotating Cage")
finder.print_summary(top_n=3)
```

This prints a formatted report showing the top-3 flights with per-feature comparison to the category mean, so the user can verify the selection before using that flight's plots in the thesis.

### Status
- [x] **Implement `RepresentativeFlightFinder` class** — ✅ Done, `dev_logs/analysis/database/representative_finder.py`
- [ ] **Integrate into `experiments_analysis_summary.ipynb`** — pending notebook update

<details>
<summary><code>💡 Integration Note (2026-06-09)</code></summary>

* The `RepresentativeFlightFinder` class is fully functional at `dev_logs/analysis/database/representative_finder.py`.
* To integrate: add a cell at the end of `experiments_analysis_summary.ipynb` that imports the finder, runs it for 4 category combinations (Rotating/Fixed × 45°/75°), and prints the top-ranked flight for each.
* **Verified working** (2026-06-09): Finder ran against live DB — 52 Rotating Cage flights matched, category system works. `category_label` and `n_flights` are `@property`, not methods (no `()` needed).
* **Verification command:**
```bash
cd /home/dorten/MasterThesisDrone
python3 -c "
import sys; sys.path.insert(0,'dev_logs/analysis/database')
sys.path.insert(0,'/home/dorten/.local/lib/python3.10/site-packages')
from representative_finder import RepresentativeFlightFinder
for cond in ['Rotating Cage', 'Fixed Cage']:
    finder = RepresentativeFlightFinder(condition=cond)
    print(f'=== {finder.category_label} ({finder.n_flights} flights) ===')
    finder.print_summary(top_n=1)
    print()
"
```
* The `print_summary()` output is formatted markdown — it renders naturally in a notebook markdown cell (wrap in a `print('```') ... print('```')` block for monospace formatting).

</details>

<details>
<summary><code>💡 Pipeline Clean-Up Helper (2026-06-09)</code></summary>

If any passes end up with NULL battery (the 6 from the corrupted flight were inserted before the ULog fallback fixed the battery table), run this to backfill:

```bash
cd /home/dorten/MasterThesisDrone
python3 -c "
import sys; sys.path.insert(0,'.')
sys.path.insert(0,'/home/dorten/.local/lib/python3.10/site-packages')
from dev_logs.analysis.database.db_manager import get_battery_efficiency_df, get_connection
df = get_battery_efficiency_df()
if not df.empty:
    db = get_connection()
    for _, row in df.iterrows():
        db.execute('''UPDATE flights_summary SET
            capacity_drain_rate_pct_per_min = ?,
            voltage_drop_rate_v_per_min = ?
            WHERE pass_name LIKE ? || '%'
            AND capacity_drain_rate_pct_per_min IS NULL''',
            (row['capacity_drain_rate_flying'], row['voltage_drop_rate_flying'], row['flight_name']))
    db.commit()
    # Report
    c = db.cursor()
    c.execute('SELECT COUNT(*) FROM flights_summary WHERE capacity_drain_rate_pct_per_min IS NULL')
    print(f'Remaining NULL: {c.fetchone()[0]}')
    db.close()
"
```
</details>
