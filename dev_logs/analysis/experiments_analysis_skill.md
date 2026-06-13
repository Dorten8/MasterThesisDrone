# Experiments Analysis Pipeline — Skill & Cookbook

SSoT for thesis experiments directory architecture, coding standards, data processing, and visual formatting (Rotating Cage vs Fixed Cage collision dynamics / IMU-MoCap correlation).

## 0. Planning, Execution & Verification Protocol

### 0.1 Walkthrough Document
- **Location:** `dev_logs/analysis/walkthrough_experiments.md`
- **Role:** SSoT for what to implement, pending/done tracking
- **Structure:** Mirrors notebook cell sequence: `experiments_analysis.ipynb` → `experiments_analysis_summary.ipynb`
- **Format:** Checkboxes `- [ ]` pending, `- [x]` done, subsections per topic
- **Usage anchor:** When asked "where is that in the walkthrough?", cite exact section heading

### 0.2 Skill File (This Document)
- **Location:** This file — `dev_logs/analysis/experiments_analysis_skill.md`
- **Role:** Coding standards, architecture rules, visualization SSoT
- **Ordering:** Sections 1–8 follow `walkthrough_experiments.md` section order
- **Relation:** Skill = *how* to implement; Walkthrough = *what* to implement next

### 0.3 Execution Protocol — Italic Notes
When implementing a walkthrough task, update this file's relevant subsection with an italic block:
```
*[YYYY-MM-DD] **What was done:** <summary>
**How to verify:** <steps>
**Where:** <file+cell/section>*
```
This creates a persistent log for context recovery.

### 0.4 Section Mapping Quick-Reference
| Walkthrough Section → | Skill Section → | Notebook |
|---|---|---|
| 1. Individual Flight Ingestion & Analysis | §1 + §2 | `experiments_analysis.ipynb` |
| Telemetry Ingestion & Database | §3 + `db_pipeline.py` | — |
| Individual Pass Visualizations | §5 items 1–7 | `experiments_analysis.ipynb` |
| 2. Aggregate Comparative Dashboard | §4 + §5 items 8–13 | `experiments_analysis_summary.ipynb` |
| Battery, Deceleration & Structural Dynamics | §3 (battery) + Steps 11–12 | `experiments_analysis_summary.ipynb` |
| Advanced Thesis Highlights | freeform | `experiments_analysis_summary.ipynb` |
| Statistical Aggregate Performance | §5 items 12–13 (Plots 17–18) | `experiments_analysis_summary.ipynb` |

## 1. Modular Workspace & Cleanliness Rules
- `database/`: Ingestion, MCAP slicing, SQLite managers
- `kinematics/`: Physics calculations, plot generators
- `dev_logs/scratch/`: **Mandatory** for all temporary/playground scripts (`scratch_*.py`), cell editors, one-off helpers
- **Policy:** `analysis/` contains ONLY permanent scripts, active DBs, graphics. Helpers → `dev_logs/scratch/`, deleted/archived after use
- **API Access:** Expose shared functions in package `__init__.py`

## 2. Core Data Processing & Flight Control
- **Mission SSoT:** Nominal waypoints from active mission class (e.g., `ExpCollision75Deg`), NOT from `.mcap` interpolated setpoints
- **Coordinate Alignment:** EKF-to-MoCap offset via `pd.merge_asof` (median diff); setpoints restored to ENU frame
- **MCAP Segmenter:** `db_mcap_event_segmenter.py` slices raw flights into `-passXX.mcap` by waypoint events
- **Velocity Feedforward:** Autonomous sweeps use `hb.velocity = True` for smooth transit, avoid jerky position-only hunting

## 3. SQLite Database (`experiments_summary.db`)

### 3.0 Approved Flight Cutoff — SSoT
**Only flights >= `20260524-1904` (May 24, 2026, 19:04 UTC+2) are approved.** Earlier flights had poor MoCap tracking (Fixed Cage ~10 Hz dropouts, Rotating ~30 Hz).

**Source:** `db_manager.py`
```python
APPROVED_CUTOFF = "20260524-1904"
def is_approved_flight(folder_name): ...
```
**All processing scripts import:** `from db_manager import is_approved_flight, APPROVED_CUTOFF`

**Enforced by:** `db_pipeline.py`, `db_mcap_event_segmenter.py`, `db_unsliced_flights_bat_analyser.py`, `database/rerun_pipeline.py`

**Approved flights (as of 2026-06-09):** 2026-05-24 (1, from 19:04+), 05-25 (6), 05-26 (2), 05-27 (2), 05-28 (5), 05-29 (6), 05-30 (3), 05-31 (4), 06-01 (1) = 30 flights. **Excluded:** All 16 flights on 2026-05-23 and 2026-05-24 before 19:04 (no pass-sliced MCAPs).

- **SSHFS Safety:** 30s timeout, `PRAGMA busy_timeout`, never open `.db` externally
- **Querying:** Load via `get_database_df()`, filter client-side (e.g., `.query("impact_detected == 1")`)

### 3.1 Table: `flights_summary` — Per-Pass Metrics
PK: `flight_name` (`flight_YYYYMMDD-HHMM_XXdeg_column_collision_loop_<condition>/passNN`). 179 rows (108×45° + 71×75°).

#### Identification & Core
| Column | Type | Meaning |
|---|---|---|
| `flight_name` | TEXT PK | Pass identifier |
| `condition` | TEXT | `'Rotating Cage'` / `'Fixed Cage'` |
| `sweep_speed` | REAL | Transit speed (m/s) |
| `battery_at_start` | REAL | Battery % at WP2 refined |

#### Impact Dynamics
| Column | Type | Meaning |
|---|---|---|
| `impact_speed` | REAL | Speed at contact (m/s) |
| `impact_accel` | REAL | Peak tangential deceleration at impact (m/s²) |
| `before_impact_accel` | REAL | Pre-impact approach deceleration (m/s²) |
| `impact_angle` | REAL | Achieved contact normal angle (0–90°) |
| `impact_detected` | INTEGER | 1 if closest_clearance < 0 |
| `closest_clearance` | REAL | Min drone–column distance (cm, negative=penetration) |

#### Recovery & Deviation
| Column | Type | Meaning |
|---|---|---|
| `avg_dev_after` | REAL | Mean lateral post-impact deviation (mm) |
| `max_dev_after` | REAL | Peak lateral post-impact displacement (mm) |
| `recovery_area` | REAL | Integrated spatial error envelope (mm·m) |

#### Waypoints — Nominal vs Actual
| Column | Type | Meaning |
|---|---|---|
| `nom_sp_x/y/z` | REAL | Nominal start point (exp_sp) |
| `nom_ep_x/y/z` | REAL | Nominal end point (exp_ep) |
| `act_sp_x/y/z` | REAL | MoCap position at exp start (WP2 refined) |
| `act_ep_x/y/z` | REAL | MoCap position at exp end (WP3) |

#### IMU — Peak Shock
| Column | Type | Meaning |
|---|---|---|
| `imu_peak_accel` | REAL | Peak 3D accel magnitude (g) |
| `imu_peak_accel_x/y/z` | REAL | Per-axis peak accel (g) |
| `imu_peak_gyro` | REAL | Peak 3D angular velocity magnitude (rad/s) |
| `imu_peak_gyro_x/y/z` | REAL | Per-axis peak angular velocity (rad/s) |

#### IMU — Integrated Energy (400ms impact window)
| Column | Type | Meaning |
|---|---|---|
| `imu_accel_energy` | REAL | Integrated linear accel magnitude over 400ms (g·s) |
| `imu_accel_energy_x/y/z` | REAL | Per-axis integrated accel (g·s) |
| `imu_gyro_energy` | REAL | Integrated angular velocity magnitude — "Rotational Energy" (rad) |
| `imu_gyro_energy_x/y/z` | REAL | Per-axis integrated angular velocity (rad) |

#### IMU — Settling & Vibration
| Column | Type | Meaning |
|---|---|---|
| `imu_accel_settling` | REAL | Time to return to accel baseline after impact (s) |
| `imu_gyro_settling` | REAL | Time to return to gyro baseline after impact (s) |
| `imu_vib_ax/ay/az` | REAL | Pre-impact accel std dev (g) |
| `imu_vib_gx/gy/gz` | REAL | Pre-impact angular velocity std dev (rad/s) |

#### IMU — Acceleration Spread
| Column | Type | Meaning |
|---|---|---|
| `imu_ax/ay/az_spread_impact` | REAL | Accel std dev during impact window [t-50ms, t+350ms] (g) |
| `imu_ax/ay/az_spread_regular` | REAL | Accel std dev during 1.0s pre-impact steady flight (g) |

#### Motor & Actuator
| Column | Type | Meaning |
|---|---|---|
| `motor_avg_before` | REAL | Mean normalized actuator before impact (0–1) |
| `motor_max_before` | REAL | Max actuator before impact |
| `motor_avg_after` | REAL | Mean actuator after impact |
| `motor_max_after` | REAL | Max actuator after impact |
| `motor_thrust_surge` | REAL | `max_after − avg_before` |
| `motor_imbalance_after` | REAL | Post-impact motor variance (retired from pipeline, kept in notebook for interactive debugging) |
| `motor_m1/m2/m3/m4_avg_after` | REAL | Per-motor avg output after impact |

#### Control Allocator & Rate Tracking
| Column | Type | Meaning |
|---|---|---|
| `allocator_saturation_duration_sec` | REAL | Seconds at 0 or 1 actuator limit in 1s post-impact |
| `max_unallocated_torque` | REAL | Peak unsatisfied torque demand (N·m) |
| `thrust_setpoint_achieved_pct` | REAL | % commanded thrust delivered during saturation |
| `roll_rate_error_rms` | REAL | RMS roll rate tracking error (rad/s) |
| `pitch_rate_error_rms` | REAL | RMS pitch rate tracking error (rad/s) |
| `yaw_rate_error_rms` | REAL | RMS yaw rate tracking error (rad/s) |
| `max_actuator_output` | REAL | Peak actuator output 0–100% from ULog |

#### Battery & Flight Efficiency
| Column | Type | Meaning |
|---|---|---|
| `active_flight_time_sec` | REAL | Takeoff→disarm duration (s) |
| `voltage_drop_rate_v_per_min` | REAL | Flight-level V/min |
| `capacity_drain_rate_pct_per_min` | REAL | Flight-level %/min |
| `path_spread_sdld` | REAL | Std dev of lateral displacement (mm) |

#### Experiment Timing (2026-06-10)
| Column | Type | Source | Meaning |
|---|---|---|---|
| `timestamp_db` | TEXT | `datetime.now()` | Write timestamp |
| `e_sp_timestamp_PX4` | INTEGER | WP2_cmd | PX4 µs — command transition to exp_sp |
| `e_sp_timestamp_PX4_forw` | INTEGER | WP2 refined | PX4 µs — forward movement start (NULL if <50ms from cmd) |
| `e_ep_timestamp_PX4` | INTEGER | WP3 | PX4 µs — sweep end |
| `e_impact_timestamp_PX4` | INTEGER | Column Impact | PX4 µs — actual collision instant |

**Reader pattern:** `COALESCE(e_sp_timestamp_PX4_forw, e_sp_timestamp_PX4)` for best start time.

### 3.2 Table: `flights_battery_efficiency` — Flight-Level Battery
PK: `flight_name` (`flight_YYYYMMDD-HHMM_XXdeg_column_collision_loop_<condition>`). 48 rows. Populated by `db_unsliced_flights_bat_analyser.py`, used by `db_pipeline.py` for per-pass lookup.

| Column | Type | Meaning |
|---|---|---|
| `flight_name` | TEXT PK | Flight folder ID |
| `condition` | TEXT | `'Rotating Cage'` / `'Fixed Cage'` |
| `log_duration` | REAL | MCAP log duration (s) |
| `total_armed_time` | REAL | Total armed time (s) |
| `total_flying_time` | REAL | Takeoff→landing duration (Z > 0.15m) |
| `voltage_at_arm` / `remaining_at_arm` | REAL | At arm |
| `voltage_at_takeoff` / `remaining_at_takeoff` | REAL | At takeoff |
| `voltage_at_landing` / `remaining_at_landing` | REAL | At landing |
| `voltage_at_disarm` / `remaining_at_disarm` | REAL | At disarm |
| `min_voltage_during_flight` | REAL | Lowest voltage observed (V) |
| `avg_voltage_during_flight` | REAL | Mean voltage (V) |
| `total_voltage_dropped` | REAL | Full-flight voltage drop (V) |
| `total_capacity_consumed_pct` | REAL | Full-flight % consumed |
| `voltage_drop_rate_armed` | REAL | V/min over full armed window |
| `capacity_drain_rate_armed` | REAL | %/min over full armed window |
| `voltage_drop_rate_flying` | REAL | V/min over takeoff→landing (copied to flights_summary) |
| `capacity_drain_rate_flying` | REAL | %/min over takeoff→landing (copied to flights_summary) |
| `timestamp_db` | TEXT | Write timestamp |

### 3.3 Experiment Timing Architecture
See [`walkthrough_experiments.md`](walkthrough_experiments.md#-experiment-startend-point-timing-architecture-2026-06-10). TL;DR: `find_waypoint_events()` WP numbering differs from mission terminal. WP2 = exp_sp, WP3 = exp_ep. WP2 refined from command→actual forward movement at `kin_calculator.py:563-587`. Stored as `e_sp_timestamp_PX4` (cmd), `e_sp_timestamp_PX4_forw` (forward, NULL if <50ms gap), `e_ep_timestamp_PX4` (end).

## 4. Universal Plotting Standards
- **100% pure Matplotlib** (no Seaborn)
- **Filenames:** Use `deg` not `°`
- **Data truncation:** Dynamically crop to `[WP2−1s, WP3+1s]`. **Combined side-by-side plots** use `[WP2 refined − 0.5s, Column Center Passed + 2.5s]` (see §Combined Plots).
- **Strict axis limits:** Hardcoded explicit ranges for repeatability
- **Y-axis truncation for outliers:** Truncate to 6 when outlier compresses range:
  - Recovery area boxplot: `set_ylim(0, 6)`, ticks every 1 cm²
  - Deceleration vs Battery (split & global): `set_ylim(0, 6)`, ticks every 1 m/s²
- **Enclosure styles:** `<Rotating Cage>` = dashed `--`, `<Fixed Cage>` = solid `-`
- **Trendlines:** Linear regression (`np.polyfit`), `y = mx + c`, footnote in all plots
- **Data origin labeling — BELOW x-axis ticks:**
  All source/filename labels are placed at the bottom of the figure, below the x-axis ticks and axis labels (never inside the axes / data area).
  - Implementation: `fig.text(0.98, 0.01, flight_name, transform=fig.transFigure, ha='right', va='bottom', fontsize=7.5, ...)`
  - Summary plots: Dynamic `Comparison of Xx Rotating Cage and Yx Fixed Cage flights` in subtitle or figure-bottom
  - Individual plots: Flight ID / bag name at figure-bottom in the lower-right corner (below all x-axis labels)
  - For pie/sunburst plots with two panels: data tables at both bottom-left (beneath Rotating) and bottom-right (beneath Fixed) using `fig.text(0.02, 0.01, ...)` and `fig.text(0.98, 0.01, ..., ha='right')`
- **Bold titles with `<Condition>`:** Every plot title uses `fontweight='bold'` and wraps condition in `< >` — applies to `fig.suptitle()` and `ax.set_title()`
- **Fixed Y-axis limits (2026-06-10) — no dynamic scaling:**

| Plot | Variable | Limits | Ticks |
|------|----------|--------|-------|
| All velocity panels (MoCap+EKF) | Speed | `-0.05` – `0.85` | every `0.2` |
| Velocity panels | Tangential accel | `-8.0` – `8.0` | every `2.0` |
| EKF debug (3×2 + dual) | Per-axis velocity | `-0.4` – `0.6` | auto |
| IMU Dynamics | Accel left axis | `-1.0` – `20.0` | every `2.0` |
| IMU Dynamics | Gyro right axis | `0` – `10` | every `1` |
| IMU XYZ | Accel X/Y/Z | `(-20,6)` / `(-6,20)` / `(-20,0)` | every `2.0` |
| IMU XYZ | Gyro X/Y | `-1.5` – `1.5` | every `0.5` |
| IMU XYZ | Gyro Z | `-1.0` – `5.0` | every `1.0` |
| MoCap Rate | Rate | `-10` – `480` Hz | auto |

- **Comparative tables (Table of Averages):**
  - Grouped by impact angle bins (30–40°, 40–50°, 50–60°, 60–90°) × cage condition
  - 6-pie-chart mission outcomes grid above table
  - Oscillating row order: Rotating→Fixed per bin
  - Explicit `N-Flights` column
  - Color coding: Rotating blue (`#d2e4f6` bg, `#0d233a` text), Fixed red (`#f7d2d2` bg, `#3d0b0b` text). Ratio heatmap for metrics columns (HSL green=better, red=worse)
  - Raw inline HTML (no `jinja2` dependency)
- **Timeline events:** Three vertical lines: `Exp. Start-point` (dotted purple/grey), `💥 Impact` (dash-dotted crimson, label on LEFT), `Exp. End-point` (dotted purple/grey)
- **Impact window shading:** Light red `[t−0.05s, t+0.35s]` across all subplots
- **Battery bins:** Red [0–40%], Orange (40–60%], Yellow-Green (60–80%], Green (80–100%]

- **Impact Angle colormap (RdYlGn — applies to all angle-colored plots):**
  Use `plt.cm.RdYlGn` with `norm = plt.Normalize(0, 90)`, where **0° (direct/orthogonal) = red** and **90° (glancing/tangential) = green**.
  - Scatter plots: `c=df['impact_angle'], cmap=cmap, norm=norm` with a colorbar
  - Categorical bins (e.g. sunburst wedges, stacked bars): map bin midpoint → cmap color
    - `<30°` → midpoint 15° → reddish
    - `30-40°` → midpoint 35° → orange–yellow
    - `40-50°` → midpoint 45° → yellow–green
    - `50-60°` → midpoint 55° → light green
    - `60-90°` → midpoint 75° → green
  - `No Impact` category always `#B0B0B0` (neutral grey)

- **Sunburst / Nested Pie Charts (2026-06-13 standard):**
  - One panel per cage condition (Rotating left, Fixed right)
  - Outermost ring: Impact Detected vs No Impact in condition colour (Rotating = `#1F77B4` blue, Fixed = `#D62728` red) — this matches the standard enclosure convention
  - Inner ring(s): Per-mission angle distribution coloured by the RdYlGn angle colormap; grey for No Impact
  - Percent labels inside wedges (`autopct`), omitting segments <3% for legibility
  - Centre text: condition name + total N
  - Figure-bottom annotation: data tables printed in both bottom-left and bottom-right corners showing the exact aggregated figures

- **Data frame annotation (summary plots):**
  Two compact tables at figure-bottom showing the exact aggregated figures used:
  - Left-bottom under Rotating Cage panel: mission breakdown table
  - Right-bottom under Fixed Cage panel: mission breakdown table
  - Format: plain monospace text via `fig.text()` or `ax.text()`

## 5. Specific Plot References

### Notebook "Interactive Analysis" Cells (`experiments_analysis.ipynb`)
These are called on-the-fly from the notebook with representative flight data. They do NOT generate pipeline PNGs. The notebook header reads "Interactive Analysis — Representative Flights" (covers both 45° and 75° despite a prior stale heading).

1. **Top-Down 2D Trajectory (`kin_plot_trajectory.py`):** 1:1 aspect, 0.5m grid, rotated axes (−y,x), white label backgrounds, limits `[-1.6,1.6]/[-0.5,1.0]`. Base layout for heatmaps.

2. **MoCap Kinetic Profile — Raw (`plot_velocity_profile_from(data, raw=True)`):** 2-panel (Speed + Accel), no MoCap rate panel. Retained in notebook for interactive debugging (retired from automated pipeline PNG generation). Same crop/markers as §5.4.

3. **MoCap Kinetic Profile — Splined (`plot_velocity_profile_from(data)`):** 3-panel (Speed, Accel, MoCap Rate). Shared X, uniform grid resampling + spline interpolation. Retained as debugging view showing MoCap rate diagnosis.

4. **EKF Kinetic Profile (Raw, no MoCap rate):** 2-panel (velocity + tangential accel), PX4 `vehicle_odometry` on MoCap grid, `np.gradient` for accel, same timeline crop/markers as MoCap profile. Title bold `<Condition>`. Pipeline output saved as `passNN_ekf_kinetic_profile.png`. Notebook wrapper `plot_ekf_kinetic_from(data)` in `flight_loader.py`.

5. **EKF Velocity Viewer (`plot_ekf_velocity_from(data)`):** Side-by-side comparison of PX4 onboard EKF velocity vs MoCap-derived velocity for a single flight. EKF shown as smooth solid line, MoCap-derived as dashed. Y-axis limits per §4 table (EKF debug row). Standard bottom-right flight-name label.

6. **EKF Dual Comparison (`plot_ekf_dual_comparison(rot_data, fix_data)`):** Two-panel figure — Rotating Cage on left, Fixed Cage on right — each with EKF velocity + tangential acceleration. Shared Y-limits per §4. Used specifically to contrast velocity smoothness across cage types.

7. **Battery Voltage Sag (`plot_battery_sag_from(data)`):** Single-axis plot showing battery voltage over time, vertical markers at waypoints. Not generated by the pipeline (retired from automated PNG output); kept in notebook for interactive per-flight diagnostics.

8. **Physical IMU Collision Dynamics (`plot_imu_dynamics_from(data)`):** Dual Y (Linear Accel vs Gyro), left axis `[-1.0, 20.0]`, shock impulse integration, settling times. Pipeline output `passNN_imu_dynamics.png`.

9. **RAW IMU X/Y/Z Components (`plot_imu_xyz_from(data)`):** 3 subplots (Lateral/X, Longitudinal/Y, Vertical/Z), height ratios `[26,26,20]`, explicit limits per axis. Pipeline output `passNN_imu_xyz_components.png`.

10. **Actuator Motor Commands (`plot_actuators_from(data)`):** 2-panel (vehicle status panel removed 2026-06-12 — showed nothing useful). Panel 1: motor commands, ylim `0.4–1.0`, ylabel `Motor Cmd (normalized)`. Panel 2: DShot output values to ESCs, ylim `750–2000`, ylabel `Motor Output (DShot value → RPM)`, xlabel `Time (s)`. Pipeline output `passNN_actuator_motor_commands.png`.

11. **Control Allocator Saturation (`plot_allocator_from(data)`):** **Retired from notebook 2026-06-12** — showed nothing beyond the actuator commands plot (same data, same saturation events). Pipeline output still generated as `passNN_control_allocator_saturation.png` for completeness.

12. **PID Rate Controller Tracking — Per-Flight (`plot_pid_tracking_from(data)`):** 3-panel (roll, pitch, yaw rate setpoint vs actual). Retained in notebook as per-flight diagnostic. **The aggregate PID tracking plot (Plot 2) in the summary notebook was retired** — it was redundant with RMS error metrics stored in DB. The per-flight PID view is NOT retired; they are different plots.

### Summary Notebook Plots (`experiments_analysis_summary.ipynb`)

13. **Comparative Overlays (Deviation vs Angle):** Actual impact angle (X) vs Max/Avg deviation (Y), battery color bins, trendlines (Rotating=dashed, Fixed=solid), monospace legend with slopes.

14. **IMU Peak Accel Z vs Commanded Motor Speed (Plot A):** X=RPM (`2000+10000*motor_max_after`), Y=`imu_peak_accel_z`, Rotating=squares, Fixed=circles, color by impact angle (`coolwarm`), no trendlines.

15. **IMU Rotational Energy (Plot 12):** 2 panels (not 3 — Settling Times panel retired). Rotational Energy Y-axis labeled `[rad]`. LaTeX formula in markdown below.

16. **Allocator Saturation (Plot 17):** Aggregate bar/boxplot of `allocator_saturation_duration_sec` grouped by condition.

17. **PID Rate Error RMS (Plot 18):** Aggregate bar/boxplot of `roll/pitch/yaw_rate_error_rms` grouped by condition.

18. **Recovery Area Boxplot:** `set_ylim(0,6)`, ticks every 1 cm². Performance improvement printout below.

19. **Deceleration vs Battery (split & global):** `set_ylim(0,6)`, ticks every 1 m/s². Trendline separation visible after truncation.

### Combined Side-by-Side Plots (`experiments_analysis.ipynb`)

20. **Combined EKF Kinetic Profile (`plot_ekf_kinetic_combined`):** 2×2 grid, figsize=(16,10). Rotating left, Fixed right. Timeline: `[WP2 − 0.5s, Column Center Passed + 2.5s]`. Legend only left column (upper left). No Exp. End-point label. Source labels: left-bottom for Rotating, right-bottom for Fixed. Output: `Combined EKF Kinetic Profile.png`.

21. **Combined IMU Dynamics (`plot_imu_dynamics_combined`):** 1×2 panels, figsize=(14,7), twin Y per panel (accel left, gyro right). Legend only left panel (upper left). No Exp. End-point label. Per-column source labels. Output: `Combined IMU Collision Dynamics.png`.

22. **Combined IMU XYZ (`plot_imu_xyz_combined`):** 3×2 grid, figsize=(16,12). Twin axes per subpanel. Accel Y-axis labels only on left column; gyro Y-axis labels only on right column. Single stacked fig-level legend in right upper corner. No Exp. End-point label. Per-column source labels. Output: `Combined IMU XYZ Components.png`.

23. **Combined Actuator Commands (`plot_actuators_and_status_combined`):** 2×2 grid, figsize=(16,10). Y-axis labels only on left column; legends only on right column. Per-column source labels. Output: `Combined Actuator Commands.png`.

24. **Combined PID Tracking (`plot_pid_rate_tracking_combined`):** 3×2 grid, figsize=(16,12). Legend only left column (upper left). Per-column source labels. Output: `Combined PID Tracking.png`.

All combined plots use `get_combined_timeline_limits()` (+2.5s after Column Center Passed). Excluded: trajectory (no time axis) and battery sag. Impact line is THIN (lw=1.2) everywhere. Exp. Start-point label vertically centred on dotted line. Bottom margin reserved (7% of figure height via `tight_layout(rect=[0, 0.07, 1, 0.96])`) — source labels sit at y=0.025, clear of X-axis descriptions.

## 6. Workspace Organization & Tool Cleanup
- `dev_logs/analysis/`: Only core production pipelines (`db_pipeline.py`, notebooks)
- `dev_logs/scratch/`: One-off scripts, prototyping, temp checks
- **Routine cleanup:** Delete completed one-off tools and scratchpads — no dead code

## 7. AI Behavior & Roadmap Management
**Do not** delete tasks, mark `[x]`, or modify checklists in `walkthrough_experiments.md` without explicit user confirmation.

## 8. Retired Graphics/Metrics — Changelog
To prevent repeat attempts of low-value visualizations:

**Pipeline-retired vs notebook-kept:** The pipeline (`db_pipeline.py`) generates `passNN_*.png` files into flight directories. Some plots are "retired" from that automated PNG generation but remain callable in the interactive notebook for manual debugging. The notebook retirement policy is separate — it only applies when the function call itself is removed from the notebook cells.

| Asset | Pipeline | Notebook | Reason |
|-------|----------|----------|--------|
| PID Rate Tracking (Plot 2, aggregate) | Retired | Retired | Redundant with RMS error metrics in DB |
| PID Rate Tracking (per-flight) | Retired | **Kept** | Diagnostic value for per-pass inspection |
| Motor Imbalance Profile (Plot F) | Retired | Retired | Dominated by ground effect, not cage dynamics |
| Sweep Velocity vs Impact Speed | Retired | Retired | Redundant (feedforward is highly consistent) |
| Raw MoCap Kinetic Profile | Retired | **Kept** | Debugging view — shows MoCap rate quality |
| Battery Sag (`battery_sag.png`) | Retired | **Kept** | Interactive per-flight voltage inspection |
| Splined MoCap Kinetic Profile | Retired | **Kept** | Detailed 3-panel rate diagnosis (replaced by EKF as standard) |
| EKF Kinetic Profile | **Active** | **Active** | New standard — clean, dropout-free velocity |
| Control Allocator Saturation | **Active** | **Retired** (2026-06-12) | Redundant with actuator commands plot — same data, same events |
| Vehicle Status Panel (in actuator plot) | **Active** | **Retired** (2026-06-12) | Showed nothing useful — removed from actuator plot |

## 9. Execution Log — Italic Notes (Most Recent First)

### Combined plots Round 2 — bottom margin, per-column labels, source below x-axis
*[2026-06-13] **Combined plots + trajectory refinements:**
(a) Halved bottom margins on all 5 combined plots: `tight_layout(rect=[0, 0.035, 1, 0.96])`, source labels at `y=0.015` — eliminates X-axis description overlap with source labels.
(b) EKF kinetic: right-column Y-labels suppressed (`labelleft=False`).
(c) IMU dynamics: per-column twin-Y — left gets accel Y-label, right gets gyro Y-label.
(d) IMU XYZ: plots pushed down under stacked legend (right upper corner), suptitle centred in whitespace (`y=0.94`), legend at `bbox_to_anchor=(0.98, 0.965)`, `rect=[0, 0.035, 1, 0.925]` — full (16×12) height preserved.
(e) Actuator combined: right-column Y-ticks hidden.
(f) Trajectory plots (both): `tight_layout(rect=[0, 0.03, 1, 0.97])` — tighter bottom margin.
(g) Flight name labels moved from axis-space (`transAxes`) to figure-bottom (`transFigure`) in trajectory, actuators, and aggregated IMU plots.
**How to verify:** Run notebook → 6 combined/trajectory PNGs regenerate with new layouts.
**Where:** `kin_plot_kinematics.py`, `kin_plot_actuators.py`, `kin_plot_trajectory.py`, `plot_aggregated_imu_dynamics.py`, `experiments_analysis_skill.md`.*

### Comprehensive code documentation pass — full pipeline
*[2026-06-13] Added detailed design-rationale docstrings and inline comments across every major analysis module. Covers: (1) `kin_calculator.py` — resampling strategies (100 Hz rationale, linear vs PCHIP, adaptive jitter logic for Rotating vs Fixed Cage), SG velocity pipeline, EKF coordinate NED→ENU alignment, waypoint detection tiers, column impact/collision geometry; (2) `db_pipeline.py` — two-tier clock sync (timesync_status vs EKF velocity correlation fallback), battery rate caching rationale (flight-level vs per-pass), motor metrics extraction, PID tracking error; (3) `eda_angle_prediction.py` — Huber IRLS regressor with full derivation (MAD scale, weight function, convergence); (4) `eda_impact_to_end.py` — normal MLE fitting (ddof=1), PX4 µs→s conversion; (5) `rf_angle_prediction.py` — dual Pearson+Spearman filter purpose, redundancy filter logic; (6) `kin_plot_imu_spread.py` — complete docstring with impact-vs-regular window definitions; (7) `kin_plot_statistics.py` — metric definitions, boxplot construction style.
**Also:** §4 updated with new data-origin labeling rule (below x-axis, figure-bottom), impact angle RdYlGn colormap spec (0°=red, 90°=green), combined plot timeline note. Aggregated IMU plot alpha 0.03→0.06 for better visibility. `thesis/references.bib` +1 ref.
**Where:** All files listed in git diff. Full documentation pass across 21 source files.*

### Kinetic y-limits truncated + All plots routed to thesis/plots/ + Actuator plot fixed + Allocator retired
*[2026-06-12] **Velocity y-limits** truncated from 0–1.25 to 0–0.85 across all four kinetic variants (MoCap raw/splined, EKF kinetic, EKF viewer/dual). **Tangential accel** from ±12 to ±8, ticks every 2. Applied consistently to `plot_velocity_profile`, `plot_ekf_kinetic_profile`, `plot_ekf_velocity_from`, `plot_ekf_dual_comparison`, and their `draw_timeline_markers` bounds.
**Output routing:** Added `output_path` forwarding to all wrappers (except battery sag — kept interactive). Added `savefig` to the two functions that owned their own plt code (`plot_ekf_velocity_from`, `plot_ekf_dual_comparison`). All notebook cells now export to `thesis/plots/` with descriptive filenames.
**Actuator plot:** Upper panel ylim 0.4–1.0, ylabel shortened. Center panel ylim 750–2000, ylabel explains DShot→RPM, added xlabel. Lower panel (vehicle status) removed — showed nothing useful. Unused `vehicle_status` ULog extraction removed.
**Control Allocator Saturation:** Removed from notebook — showed nothing beyond actuator commands (same data, same saturation events). Pipeline output still generated for completeness.
**§4 table** updated: velocity 0–0.85, accel ±8. **§5 item 10** updated. **§5 item 11** marked retired in notebook. **§8 table** updated.*

### EKF Pipeline Integration Verified & Complete
*[2026-06-11] Audit confirmed EKF fully wired: `db_pipeline.py` → `compute_ekf_kinematics()` → `calculate_metrics()` with column override. Database repopulated 2026-06-10. DB columns `impact_speed`, `impact_accel`, `before_impact_accel` are EKF-derived. Status flag in copilot-instructions.md updated from "not yet in pipeline" to "fully integrated ✅".
**Copilot-instructions.md:** stale "not yet in pipeline" text replaced.
**Skill file §4, §5.4:** Already correct — EKF listed as active standard.
**Walkthrough:** M3-M5 marked done.
**Where:** `.github/copilot-instructions.md`, `experiments_analysis_skill.md`, `walkthrough_experiments.md`.*

### EKF Kinetic Profile — Impact line fixes
*[2026-06-10] Fixed impact line bug in actuator/allocator/PID plots (used WP2 instead of Column Impact). Added `e_impact_timestamp_PX4` column to `flights_summary` (CREATE, migration, INSERT, pipeline, cache checks). Fixed `db_cache_imu.py` to align to Column Impact. Fixed `plot_aggregated_imu_dynamics.py` label from "IMPACT (WP2)" to "Column Impact".
**Verify:** Pipeline → `SELECT e_impact_timestamp_PX4` has values. Delete `imu_cache.pkl` + re-run → IMU plot impact line at t=0.
**Where:** `db_pipeline.py`, `db_manager.py`, `db_cache_imu.py`, `plot_aggregated_imu_dynamics.py`.*

*[2026-06-10] Added `plot_ekf_kinetic_profile()` to `kin_plot_kinematics.py` + `plot_ekf_kinetic_from()` to `flight_loader.py`. New notebook cells 4+5 in `experiments_analysis.ipynb`. Bold `<Condition>` titles. Fixed axis limits (Speed 0–1.2, Gyro 0–10, IMU XYZ per-axis, EKF debug −0.4 to 0.6). Wired into both pipeline sections. Replaced raw MoCap in A3 PDF dumper.
**Verify:** Pipeline → `ekf_kinetic_profile.png` per-pass. Y-ticks match §4 table.
**Where:** `kin_plot_kinematics.py`, `flight_loader.py`, `db_pipeline.py`, `plot_to_a3_pdf.py`, §4 table, §5.4, §8, §9.*

### A3 PDF Per-Pass Dumper
*[2026-06-10] Built `plot_to_a3_pdf.py` — scans flight dirs, collects 6 `passNN_*.png` per pass, arranges as A3 portrait 2×3 grid (1 pass/page). Drops `battery_sag`. Output: `all_passes_a3.pdf` (~226 MB, 141 pages). No new dependencies (matplotlib `PdfPages` + PIL). CLI flags: `--dry-run`, `--output`, `--dpi`.
**Design:** A3 portrait for legibility. Canonical order: trajectory → control_allocator → imu_dynamics → imu_xyz → kinetic_profile_raw → kinetic_profile. Missing plots → blank cells.
**Verify:** Run script → check PDF exists at ~226 MB, 141 pages, 2×3 grid.
**Where:** `dev_logs/scratch/plot_to_a3_pdf.py`. Referenced in `.github/copilot-instructions.md`.*

*[2026-06-10] Retired MoCap-based `kinetic_profile.png` and `kinetic_profile_raw.png` from pipeline (both `process_flights()` and standalone `__main__`). Also retired `battery_sag.png`. Deleted 365 + 170 PNGs from flight dirs. EKF kinetic profiles and notebook inline display untouched. Metrics already use EKF velocity/accel since 2026-06-09.
**Verify:** Pipeline → no `kinetic_profile.png`/`kinetic_profile_raw.png` in flight dirs.
**Where:** `db_pipeline.py`.*

### MoCap Frame Rate & Velocity Filtering — FAILED & REVERTED
*[2026-06-08] Full diagnostic + attempted fix for Fixed Cage velocity kinks. Position pre-filtering (12 Hz Butterworth + 20 Hz velocity) made results worse and was reverted.
**Why it failed:** 12 Hz position cutoff too close to 10 Hz dropout kink frequency. 4 Hz velocity Butterworth was doing all smoothing — relaxing it removed the only effective filter. No linear filter can separate the ~10 Hz dropout kinks from ~4 Hz collision dynamics.
**Pipeline stages:** Raw MoCap (irregular 10–120 Hz) → linear interpolation to 100 Hz (creates C⁰ kinks) → SG deriv=1 (amplifies kinks) → 4 Hz Butterworth (blurs collision) → surgical ringing removal (creates new boundary kinks).
**Remaining on disk:** `prefilter_position_fc=None` parameter (harmless, defaults off), commented-out pre-filter block, `_prefilter_fc` column (=0.0), notebook reverted to Rotating example, filter info box reverted.
**Unimplemented approaches:** Time-domain dropout detection, Kalman smoother with constant-acceleration model, Total Variation Regularization (L1 on acceleration changes).
**Where:** `kin_calculator.py` `compute_velocity()`, interactive notebook cell, `kin_plot_kinematics.py` filter info box.*

### EKF Velocity & Battery Truncation
*[2026-06-09] Two changes:*
**Change 1 — EKF Velocity:** Added `compute_ekf_kinematics(df_odom, df_mocap)` to `kin_calculator.py` (100 Hz resample, NED→ENU, speed + SG accel). Modified `calculate_metrics()` with optional `df_ekf_kin` param. Wired into both pipeline sections. Old MoCap path commented with `=== RETIRED: MoCap Velocity ===`.
**Change 2 — Battery Window:** Changed window from `arming→disarming` to `takeoff→disarming` in both pipeline sections. Old code commented with `=== RETIRED: arming_time window ===`. Fixes artificially low drain rates from ground idle.
**Verify:** Pipeline output shows EKF kinematics sample count. EKF `impact_accel` lower for Fixed Cage. Battery drain rates higher for flights with long ground idle.
**Where:** `kin_calculator.py` (new `compute_ekf_kinematics()`, modified `calculate_metrics()`), `db_pipeline.py`.*

### Battery, Deceleration & Structural Dynamics
*[2026-06-08] Added `set_ylim(0,6)` / `set_yticks(range(0,7))` to both deceleration plots — Cell 23 (split, shared Y) and Cell 24 (global). Suppresses ~9.8 outlier.
**Verify:** Both plots Y-axis 0–6, ticks every 1 m/s².
**Where:** `experiments_analysis_summary.ipynb` Cells 23, 24.*

### Global Thesis Visualizations — Recovery Area
*[2026-06-08] Added `set_ylim(0,6)` / `yticks(0,7,1)` to recovery area boxplot (Cell 12) to truncate outlier >6 cm².
**Verify:** Boxplot Y-axis 0–6, ticks every 1 cm², box separation visible.
**Where:** `experiments_analysis_summary.ipynb` Cell 12. Rule added to §4.*

### Advanced Thesis Highlights — Performance Printouts
*[2026-06-08] Added dynamic % improvement printouts for: (1) recovery area after boxplot, (2) max deviation after Plot B scatter, (3) impact deceleration after battery plot. Polished Plot 12: removed 3rd Settling Times panel (now 2 panels), Y label includes `[rad]`, added LaTeX markdown explanation.
**Verify:** Notebook shows "✅ Rotating Cage reduces recovery area by XX.X%" etc. Plot 12 has 2 panels, Rotational Energy in `[rad]`. Math markdown renders below.
**Where:** `experiments_analysis_summary.ipynb` cells after boxplot (~13), after Plot B (~33), after deceleration (~25). Plot 12 cell (~28). New markdown cell (~29).*

### Impact Timestamp Column + Pipeline Plot Retirement — 2026-06-10
*[2026-06-10] **Three changes:**

**1. Impact timestamp in DB.** Added `e_impact_timestamp_PX4` INTEGER column to `flights_summary` (schema, migration, pipeline computation, cache checks). Impact time now persisted from `wp_events.get('Column Impact')` so consumers don't re-run `find_waypoint_events()`.

**2. IMU cache & aggregated plot fixed.** `db_cache_imu.py` was aligning traces to `WP2` (gate arrival, ~2.5s early) instead of `Column Impact` (actual collision). Fixed to `wp_events.get('Column Impact') or wp_events.get('WP2')`. Aggregated plot label corrected from "IMPACT (WP2)" to "Column Impact". IMU cache rebuilt: 137 traces aligned to true impact.

**3. Pipeline plot cleanup.** Retired MoCAP kinetic plots (`kinetic_profile.png`, `kinetic_profile_raw.png`) and battery sag (`battery_sag.png`) from the pipeline — only `ekf_kinetic_profile.png` is generated for velocity/accel. Deleted 365 legacy MoCAP kinetic + 170 battery sag PNGs from flight directories. EKF kinetic, trajectory, IMU dynamics, IMU XYZ, actuators, control allocator, and PID plots remain.

**Full pipeline re-run:** `python3 -m dev_logs.analysis.database.db_pipeline --today --force-plot` — 179/179 passes, 0 errors, 1,253 PNGs regenerated (7 types × 179 passes) with corrected Column Impact alignment.

**Where:** `db_pipeline.py`, `db_manager.py`, `db_cache_imu.py`, `plot_aggregated_imu_dynamics.py`, `experiments_analysis_skill.md`.*

## 10. Calculation Techniques Reference

This section documents every mathematical technique, algorithm, and signal-processing method used across the analysis pipeline, organized by domain.  Each entry states **what** is computed, **how** it is computed, and **where** it lives.

### 10.1 Velocity & Acceleration Derivation

#### 10.1.1 MoCap Position Resampling (`kin_calculator.py:6–71`)
**What:** Raw MoCap pose data arrives at irregular 10–120 Hz.  We resample onto a uniform 100 Hz grid for compatible differentiation.

**How:**
- **Linear interpolation** (`np.interp`) — preserves raw data character (no PCHIP overshoot)
- **Dropout detection:** `np.diff(raw_t) > 1/30` flags gaps.  If >20 gaps → "high-jitter" (Fixed Cage), only repair gaps >100 ms with tight 2-sample margin.  If ≤20 gaps → "low-jitter" (Rotating Cage), mark gaps + 9-sample SG-window margin.
- **Ringing mask:** Boolean mask marking gap + margin regions.  Used downstream to surgically excise SG boundary artifacts (ringing at discontinuity edges).

#### 10.1.2 Savitzky-Golay Velocity Differentiation (`kin_calculator.py:73–202`)
**What:** Derive velocity and acceleration from resampled position on a uniform 100 Hz grid.

**How:**
- **SG filter** with `window_length=19` (≈190 ms), `polyorder=3` (cubic), `deriv=1`
- **Scaling:** `savgol_filter(..., deriv=1) / median_dt` — converts from sample-derivative to physical m/s
- **Adaptive window:** Shrunk to `n_points // 2 * 2 - 1` if data shorter than window, clamped to ≥3
- **Fixed Cage Butterworth:** 2nd-order, 4 Hz cutoff, zero-phase (`filtfilt`).  Applied after SG diff on velocity arrays to suppress dropout kink amplification (~10 Hz kinks vs. ~4 Hz collision dynamics).
- **Disabled approach (2026-06-08):** 12 Hz Butterworth pre-filter on position *before* SG diff.  Failed because 12 Hz cutoff too close to 10 Hz kink frequency — couldn't separate noise from signal.
- **Surgical ringing removal:** After SG diff, linearly interpolate over ringing mask regions (replaces SG boundary spikes with clean straight-line bridges).

#### 10.1.3 EKF Velocity (PX4 Onboard Odometry) (`kin_calculator.py:204–313`)
**What:** Use PX4 Extended Kalman Filter velocity estimates instead of MoCap-derived velocity.  The EKF fuses MoCap position (10–120 Hz) + IMU acceleration (250 Hz) internally, producing a velocity signal that is inherently smooth even during MoCap dropouts.

**How:**
- **Coordinate alignment:** NED→ENU: `vx = vx_raw`, `vy = −vy_raw`, `vz = −vz_raw` (matches `build_dataframes` convention)
- **Resample to 100 Hz** via `interp1d(kind='linear')` onto MoCap-aligned time grid
- **Speed:** `sqrt(vx² + vy² + vz²)`
- **Acceleration:** SG `deriv=1` on velocity (same window=19, polyorder=3 as MoCap pipeline), fallback `np.gradient`
- **Override in metrics:** When `df_ekf_kin` is provided to `calculate_metrics()`, all velocity/accel columns in `df_mocap` are replaced with EKF values via `np.interp` onto MoCap time grid before metric extraction

### 10.2 Waypoint Detection & Column Impact Timing

#### 10.2.1 Three-Tier Waypoint Detection (`kin_calculator.py:373–618`)
**What:** Locate the 4 mission waypoints (WP1–WP4) and the column impact instant for each flight pass.

**How — Priority 1 (Mission SSoT):**
- Import mission class via `detect_mission_class()` (45°/75° from label + setpoint coordinate heuristics)
- Simulate `mission.on_start()` to extract nominal `exp_sp` (WP2) and `exp_ep` (WP3) coordinates
- Build `wp_sequence` list from mission attributes

**How — Priority 2 (Dynamic MCAP waypoints):**
- Accept `dynamic_waypoints` from `build_dataframes()` only if exactly 4 were found (>4 = PX4 trajectory interpolation noise, not real WPs)

**How — Priority 3 (Hardcoded fallback):**
- 45°: lane x=0.248, 75°: lane x=0.186.  WPs: (x, 1.200), (x, 0.950), (x, −1.200), (0, 0.300)

**Sequential command-transition search:**
- When the flight director transitions from commanding WP1 → WP2, WP1 is "reached"
- Detection: `sqrt((x_cmd − next_wx)² + (y_cmd − next_wy)²) < 0.05 m` on `df_setpoint`
- Search pointer advances 0.1 s after each match to prevent self-triggering
- Failed passes: pointer jumps 5 s forward to next WP1→WP2 transition

**MoCap proximity fallback:**
- For truncated/segmented passes with no setpoint data: find `argmin` distance to WP2 and WP3 nominal coordinates

#### 10.2.2 WP2 Refinement (`kin_calculator.py:563–588`)
**What:** The WP2 command timestamp marks when PX4 *accepts* the exp_sp waypoint — but the drone may still be stationary.  Refine to the moment forward motion actually begins.

**How:**
1. Find when drone crosses Y = 0.70 m (toward WP3 at Y = −1.2)
2. In the pre-cross window, find the **last** timestamp where speed < 0.10 m/s
3. Falls back to minimum-speed point in window, then to end-of-window

#### 10.2.3 Column Passed & Column Impact (`kin_calculator.py:590–613`)
**How — Column Passed:**
- In sweep window [WP2, WP3]: `argmin |y − column_y|` — instant of closest Y to column

**How — Column Impact (collision instant):**
- **Proximity gate:** Minimum 2D distance from drone to column center must be ≤ 0.38 m (≈cage radius + column radius + margin)
- **Speed peak search:** In window `[t_closest − 0.25 s, t_closest + 0.1 s]`, find `argmax speed` — the instant of peak deceleration onset
- Falls back to geometric closest-approach time if no speed data

### 10.3 Clearance, Collision & Path Tracking

#### 10.3.1 Closest Clearance (`kin_calculator.py:761–765`)
**What:** Minimum surface-to-surface distance between drone cage and column.

**How:**
- `dist_to_col_center = sqrt((x − column_x)² + (y − column_y)²)`
- `closest_clearance = min(dist) − column_radius − cage_radius`
- Negative value → physical penetration (collision detected)

#### 10.3.2 Perpendicular Path Tracking Error (`kin_calculator.py:695–702`)
**What:** Lateral deviation from the ideal straight line WP2→WP3.

**How:**
- Point-to-line distance formula: `|(y₂−y₁)x₀ − (x₂−x₁)y₀ + x₂y₁ − y₂x₁| / √((y₂−y₁)² + (x₂−x₁)²)`
- Applied to every MoCap sample in the sweep window
- **Metrics:** Mean error (accuracy), Max error (worst excursion), Std Dev × 1000 = `path_spread_sdld` (mm)

#### 10.3.3 Recovery Area (`kin_calculator.py:846–875`)
**What:** Integrated spatial error envelope after collision, measuring how much the drone deviates from the nominal path.

**How:**
- **Path projection:** Each post-impact point is projected onto the nominal WP2→WP3 line via dot product with unit direction vector: `s = (p − wp2) · û`
- **Sort by s** (travel distance along path) to ensure monotonic integration
- **Trapezoidal integration:** `np.trapz(d_sorted, s_sorted) × 1000` → mm·m (area under deviation curve)

#### 10.3.4 Impact Angle (`kin_calculator.py:818–836`)
**What:** Angle between the drone's velocity vector and the collision normal (vector from drone to column center).

**How:**
- Collision normal: `r = column_center − drone_position`
- Velocity vector: `v = (vx, vy)` at impact instant
- `cos(θ) = (r · v) / (|r| · |v|)`, clamped to [−1, 1]
- `θ = arccos(cos(θ))` in degrees
- If θ > 90°, reflect: `θ = 180 − θ`

### 10.4 IMU Shock & Structural Dynamics

#### 10.4.1 IMU Signal Preprocessing (`db_loader.py:244–247`)
**How:**
- **Acceleration magnitude:** `a_mag = sqrt(ax² + ay² + az²)`
- **Deviation from gravity:** `a_deviation = |a_mag − 9.81|` — isolates collision forces from steady 1g
- **Angular velocity magnitude:** `g_mag = sqrt(gx² + gy² + gz²)`

#### 10.4.2 Peak Shock Extraction (`kin_calculator.py:888–921`)
**What:** Maximum linear and rotational shock during collision.

**How:**
- **Contact window:** [t_impact − 0.05 s, t_impact + 0.35 s] — 400 ms total
- **Peak accel:** `max(a_deviation)` over contact window, per-axis `max(|ax|)`, Z-axis `max(|az + 9.81|)` (restores absolute including gravity)
- **Peak gyro:** `max(g_mag)`, per-axis `max(|gx|)`, etc.

#### 10.4.3 Integrated Shock Energy (`kin_calculator.py:923–943`)
**What:** Time-integrated IMU magnitude over the 400 ms contact window — measures total shock "dose."

**How:**
- **Trapezoidal integration:** `np.trapz(signal, t_vals)` — numerically integrates the area under the signal curve
- Applied to `a_deviation`, per-axis `|ax|`, `|az+9.81|`, `g_mag`, per-axis `|gx|`, etc.
- Units: accel energy [g·s], gyro energy [rad]

#### 10.4.4 Settling Time (`kin_calculator.py:945–958`)
**What:** Time from impact until shock amplitude returns to baseline.

**How:**
- **Accel settling:** Last timestamp where `a_deviation ≥ 1.5 m/s²`
- **Gyro settling:** Last timestamp where `g_mag ≥ 0.5 rad/s`
- If no samples above threshold → 0.0 (immediate recovery)

#### 10.4.5 Vibration & Spread (`kin_calculator.py:912–920, 960–976`)
**What — Vibration (post-impact):** Std dev over stabilization window [t+0.2 s, t+3.0 s] (skip first 0.2 s to exclude initial shock spike).  Measures post-collision oscillation amplitude.

**What — Spread (impact vs. regular):**
- Impact spread: `std(ax)` over [t−50 ms, t+350 ms] (400 ms), in g
- Regular spread: `std(ax)` over [t−1050 ms, t−50 ms] (1.0 s pre-impact), in g
- Captures high-frequency "buzz" vs. normal flight vibration

#### 10.4.6 Aggregated IMU Display (`db_cache_imu.py` → `plot_aggregated_imu_dynamics.py`)
**How — Cache:**
- For each impact flight: load IMU from MCAP, align to Column Impact time, extract `[t_rel −1.0, +2.0]` window
- Store `t_rel`, `a_deviation`, `g_mag` arrays in pickle
- **Why:** Avoids re-loading 179 MCAPs (200MB+ each) for every aggregated plot regeneration

**How — Plot:**
- Group traces by condition, compute per-timestep mean ± SEM across all flights
- Display as time-aligned overlay with shaded confidence bands

### 10.5 Motor & Actuator Analysis

#### 10.5.1 MCAP-ULog Clock Synchronization (`db_pipeline.py:70–145`)
**What:** Align MCAP bag-relative timestamps with PX4 ULog microsecond timestamps.

**How — Primary (timesync_status):**
- Read `/fmu/out/timesync_status` from MCAP → `observed_offset` (ns)
- `offset_sec = −(observed_offset × 1e−6)`
- Converts ULog µs to MCAP-relative seconds: `(t_us × 1e−6) + offset_sec − bag_start_ns/1e9`

**How — Fallback (EKF velocity correlation):**
- Pick one `vehicle_odometry` message from MCAP (ROS timestamp + velocity)
- Scan all `vehicle_local_position` messages in ULog for closest velocity vector (minimum squared error)
- Compute offset from matching timestamps

#### 10.5.2 Motor Command Metrics (`db_pipeline.py:148–245`)
**What:** Extract motor response to collision from ULog `actuator_motors`.

**How:**
- **Average signal:** `(control[0] + control[1] + control[2] + control[3]) / 4.0`
- **Before window:** [t_impact − 1.0, t_impact) → `avg_before`, `max_before`
- **After window:** [t_impact, t_impact + 1.0] → `avg_after`, `max_after`
- **Thrust surge:** `max_after − avg_before` — how much harder motors work after impact
- **Imbalance (0.4 s):** Mean of per-timestamp `std(control[0:4])` over [t_impact, t_impact+0.4s] — measures asymmetric motor response
- **Max actuator output:** Scan `actuator_outputs` for peak PWM, convert: `(PWM − 1000) / 10.0 → %`

#### 10.5.3 Control Allocator Saturation (`db_pipeline.py:271–335`)
**What:** Time spent at actuator saturation limits in 1 s post-impact.

**How — 3-tier fallback:**
1. **actuator_motors (10 Hz):** Saturation if any `control[i] ≥ 0.999` or `≤ 0.001`.  Sum `dt` of saturated samples.
2. **actuator_outputs (50 Hz):** Saturation if any output ≥ 1999 (or 2000) or ≤ 115 (or 1000).  Auto-detects PWM range.
3. **control_allocator_status:** Saturation if any `actuator_saturation[i] != 0`.
- **Time-weighted:** Sum of `dt` values for saturated samples, not sample count
- **Unallocated torque norm:** `sqrt(tx² + ty² + tz²)` from control_allocator_status
- **Thrust achieved %:** `mean(thrust_setpoint_achieved) × 100`

#### 10.5.4 PID Rate Tracking Error (`db_pipeline.py:354–375`)
**What:** RMS error between commanded angular rate setpoint and actual angular velocity.

**How:**
- Interpolate `vehicle_rates_setpoint` timestamps onto `vehicle_angular_velocity` timestamps via `np.interp`
- **Error:** `roll_err = actual_xyz[0] − interpolated_roll_sp`
- **RMS:** `sqrt(mean(err²))` for roll, pitch, yaw over [t_impact, t_impact+1.0s]

### 10.6 Battery Efficiency

#### 10.6.1 Flight-Level Battery Analysis (`db_unsliced_flights_bat_analyser.py`)
**What:** Battery voltage and capacity drain rates computed from full-flight (unsliced) MCAPs.

**How:**
- **Memory-safe streaming:** Only 4 topics loaded from MCAP (poses, battery_status, vehicle_status, vehicle_odometry) — avoids OOM on 200MB+ files
- **Milestones:** Arm, Takeoff (Z > 0.15 m), Landing (last Z > 0.15 m), Disarm
- **Voltage drop rate:** `(V_takeoff − V_landing) / flying_min` [V/min]
- **Capacity drain rate:** `(SOC_takeoff − SOC_landing) / flying_min` [%/min]
- **Negative prevention:** `max(0.0, x)` on all drop values
- **Per-pass lookup:** `db_pipeline.py` reads flight-level rates from `flights_battery_efficiency` table (per-pass MCAP windows are ~10 s — too short for meaningful battery trend)

#### 10.6.2 Nearest-Timestamp Battery Query (`kin_calculator.py:620–626` → `db_unsliced_flights_bat_analyser.py:115–121`)
**How:** Binary search via `np.searchsorted(t_array, t_query)`, clamped to [0, len−1], returns nearest value.

### 10.7 Statistical & ML Analysis

#### 10.7.1 Trendlines & Regression
**How — Simple linear:** `np.polyfit(x, y, 1)` → `y = mx + c` (used in summary notebook overlay plots)

**How — Robust Huber regressor (`eda_angle_prediction.py:157–190`):**
- **Huber loss:** Quadratic for |u| ≤ δ, linear for |u| > δ — down-weights outlier influence
- **IRLS algorithm:**
  1. Initial OLS fit via normal equations (`np.linalg.lstsq`)
  2. Compute residuals, estimate scale: `σ̂ = MAD(residuals) / 0.6745` (robust σ under Gaussian)
  3. Compute weights: `w_i = 1.0` if `|u_i| ≤ δ`, else `δ / |u_i|`
  4. Weighted least squares: `β = (XᵀWX)⁻¹XᵀWy`
  5. Iterate until `||β_new − β_old|| < tol`
- **δ = 1.345:** Standard value for 95% asymptotic efficiency under normality
- **Pseudo-R²:** `1 − SS_res / SS_tot` (same formula as OLS R² but with Huber fit)

#### 10.7.2 Correlation Analysis
**How — Pearson r (`scipy.stats.pearsonr`):**
- Measures linear correlation strength and direction
- Significance: `*** p<0.001`, `** p<0.01`, `* p<0.05`

**How — Spearman ρ (`scipy.stats.spearmanr`):**
- Rank-based correlation — captures monotonic (not just linear) relationships
- Used alongside Pearson in RF feature selection to catch non-linear signals

#### 10.7.3 Statistical Tests (Between-Condition Comparison)
**How — Welch's t-test (`scipy.stats.ttest_ind, equal_var=False`):**
- Tests for difference in means without assuming equal variance
- Parametric (assumes approximate normality)

**How — Mann-Whitney U (`scipy.stats.mannwhitneyu`):**
- Non-parametric rank-sum test — no distribution assumption
- Used alongside Welch's for robustness

**How — Cohen's d effect size:**
- `d = (μ₁ − μ₂) / √((σ₁² + σ₂²) / 2)` — pooled standard deviation
- Interpret: |d| < 0.2 = negligible, 0.2–0.5 = small, 0.5–0.8 = medium, >0.8 = large

#### 10.7.4 Normal Distribution Fit (`eda_impact_to_end.py:161–165`)
**How:** `scipy.stats.norm.pdf(x, μ, σ)` with MLE parameters: μ = sample mean, σ = sample std (ddof=1).  Overlaid on density histogram for visual normality check.

#### 10.7.5 Random Forest Angle Prediction (`models/rf_angle_prediction.py`)
**What:** Predict impact angle from IMU features using ensemble learning.

**Feature selection:**
- Broad filter: |Pearson r| or |Spearman ρ| > 0.3
- Redundancy removal: compute pairwise |r| matrix (upper triangle), drop weaker of each pair with |r| > 0.85

**Model:**
- `RandomForestRegressor(n_estimators=200, max_features='sqrt')` — ensemble of 200 decorrelated trees
- **Stratified 5-fold CV:** `StratifiedKFold` with angle quartile bins — ensures each fold has representative angle distribution
- **Nested grid search:** Outer CV for evaluation, inner CV for `max_depth ∈ [3,4,5,6]` × `min_samples_leaf ∈ [3,5,7]`
- **MDI importance:** Mean Decrease in Impurity — how much each feature reduces variance across all tree splits
- **Permutation importance:** `sklearn.inspection.permutation_importance` with 20 repeats — drop-column method, more reliable than MDI
- **OOB score:** Out-of-bag R² — unbiased estimate using samples not in each tree's bootstrap

**Evaluation:**
- Actual-vs-predicted scatter with ±5° band, residuals-vs-predicted check
- Learning curve: R² vs training size, 8 points from 30% to 100%
- Cross-condition transfer: train on Fixed Cage → test on Rotating Cage
- Huber baseline: top-1-feature linear model vs multi-feature RF

### 10.8 Data Storage & Caching

#### 10.8.1 SQLite Schema Migration (`db_manager.py:131–333`)
**How:** `CREATE TABLE IF NOT EXISTS` for initial schema, then `ALTER TABLE ADD COLUMN` for each new column (catches `OperationalError` if already exists) — incremental, backwards-compatible schema evolution.

#### 10.8.2 Trajectory Cache (`db_cache_trajectories.py`)
**How:** Query DB for impacted flights, load each MCAP once, extract `(x, y, z, t)` arrays + column position, pickle to disk.  179 flights × ~10 s trajectories — loading from cache is instant vs. minutes from raw MCAP.

#### 10.8.3 Pass Exclusion Config (`db_manager.py:42–121`)
**How:** JSON-based `config_db.json` maps `folder_name → {excluded: true, reason, notes, passes: [...]}`.  Human review overrides the clearance-based `impact_detected` heuristic.  `apply_exclusion_config_to_db()` syncs to SQLite `excluded` column.
