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

**Enforced by:** `db_pipeline.py`, `db_mcap_event_segmenter.py`, `db_unsliced_flights_bat_analyser.py`, `_rerun_pipeline.py`

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
- **Data truncation:** Dynamically crop to `[WP2−1s, WP3+1s]`
- **Strict axis limits:** Hardcoded explicit ranges for repeatability
- **Y-axis truncation for outliers:** Truncate to 6 when outlier compresses range:
  - Recovery area boxplot: `set_ylim(0, 6)`, ticks every 1 cm²
  - Deceleration vs Battery (split & global): `set_ylim(0, 6)`, ticks every 1 m/s²
- **Enclosure styles:** `<Rotating Cage>` = dashed `--`, `<Fixed Cage>` = solid `-`
- **Trendlines:** Linear regression (`np.polyfit`), `y = mx + c`, footnote in all plots
- **Data origin labeling:**
  - Summary plots: Dynamic `Comparison of Xx Rotating Cage and Yx Fixed Cage flights` in bottom-right or subtitle
  - Individual plots: Source bag filename/flight ID in bottom-right
- **Bold titles with `<Condition>`:** Every plot title uses `fontweight='bold'` and wraps condition in `< >` — applies to `fig.suptitle()` and `ax.set_title()`
- **Fixed Y-axis limits (2026-06-10) — no dynamic scaling:**

| Plot | Variable | Limits | Ticks |
|------|----------|--------|-------|
| All velocity panels (MoCap+EKF) | Speed | `-0.05` – `1.25` | every `0.2` |
| Velocity panels | Tangential accel | `-12.0` – `12.0` | every `4.0` |
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

10. **Actuator Motor Commands (`plot_actuators_from(data)`):** 4-motor subplot (one per motor). Normalized output 0–1 vs time. Vertical markers at waypoints. Pipeline output `passNN_actuator_motor_commands.png`.

11. **Control Allocator Saturation (`plot_allocator_from(data)`):** 2-panel: (1) actuator motor outputs with saturation highlights, (2) allocated vs unallocated torque. Requires `.ulg` alongside MCAP. Pipeline output `passNN_control_allocator_saturation.png`.

12. **PID Rate Controller Tracking — Per-Flight (`plot_pid_tracking_from(data)`):** 3-panel (roll, pitch, yaw rate setpoint vs actual). Retained in notebook as per-flight diagnostic. **The aggregate PID tracking plot (Plot 2) in the summary notebook was retired** — it was redundant with RMS error metrics stored in DB. The per-flight PID view is NOT retired; they are different plots.

### Summary Notebook Plots (`experiments_analysis_summary.ipynb`)

13. **Comparative Overlays (Deviation vs Angle):** Actual impact angle (X) vs Max/Avg deviation (Y), battery color bins, trendlines (Rotating=dashed, Fixed=solid), monospace legend with slopes.

14. **IMU Peak Accel Z vs Commanded Motor Speed (Plot A):** X=RPM (`2000+10000*motor_max_after`), Y=`imu_peak_accel_z`, Rotating=squares, Fixed=circles, color by impact angle (`coolwarm`), no trendlines.

15. **IMU Rotational Energy (Plot 12):** 2 panels (not 3 — Settling Times panel retired). Rotational Energy Y-axis labeled `[rad]`. LaTeX formula in markdown below.

16. **Allocator Saturation (Plot 17):** Aggregate bar/boxplot of `allocator_saturation_duration_sec` grouped by condition.

17. **PID Rate Error RMS (Plot 18):** Aggregate bar/boxplot of `roll/pitch/yaw_rate_error_rms` grouped by condition.

18. **Recovery Area Boxplot:** `set_ylim(0,6)`, ticks every 1 cm². Performance improvement printout below.

19. **Deceleration vs Battery (split & global):** `set_ylim(0,6)`, ticks every 1 m/s². Trendline separation visible after truncation.

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

## 9. Execution Log — Italic Notes (Most Recent First)

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
