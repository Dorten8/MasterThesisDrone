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
| `battery_at_start` | REAL | Battery % at Exp. Start-point |

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
| `act_sp_x/y/z` | REAL | MoCap position at Exp. Start-point (refined) |
| `act_ep_x/y/z` | REAL | MoCap position at Exp. End-point |

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
| `e_sp_timestamp_PX4` | INTEGER | Exp. Start-point cmd | PX4 µs — command transition to exp_sp |
| `e_sp_timestamp_PX4_forw` | INTEGER | Exp. Start-point refined | PX4 µs — actual forward movement (NULL if <50ms from cmd) |
| `e_ep_timestamp_PX4` | INTEGER | Exp. End-point | PX4 µs — sweep end |
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
See [`walkthrough_experiments.md`](walkthrough_experiments.md#-experiment-startend-point-timing-architecture-2026-06-10). TL;DR: `find_waypoint_events()` WP numbering differs from mission terminal. WP2 = Exp. Start-point, WP3 = Exp. End-point. Exp. Start-point refined from command→actual forward movement at `kin_calculator.py:563-587`. Stored as `e_sp_timestamp_PX4` (cmd), `e_sp_timestamp_PX4_forw` (forward, NULL if <50ms gap), `e_ep_timestamp_PX4` (end).

## 4. Universal Plotting Standards

### 4.0 Output Routing & Layout

#### 4.0.1 Output Routing — Dual-Save Rule

- **Every plot function called from a notebook MUST save to both:**
  1. **`dev_logs/analysis/graphics/`** — development/iteration copy (numbered filename, e.g. `plot_19_aggregated_imu_dynamics.png`)
  2. **`thesis/plots/`** — LaTeX-ready copy (descriptive filename matching `\includegraphics` references in thesis)

- **Rationale:** The notebook generates the `graphics/` version for quick preview and iteration.  The `thesis/plots/` version is the canonical source for `\includegraphics` in thesis `.tex` files, avoiding manual copy steps and stale-thesis-plot risk.

- **Filename convention for thesis/plots/:**
  - Match the thesis `\includegraphics` filename **exactly**, including spaces and punctuation.
  - For new plots not yet referenced in thesis: use a human-readable title-case name (e.g. `Mission Outcome Distribution.png`).

- **LaTeX tables** (`.tex` files generated by `plot_*_table()` functions) save to `thesis/tables/` — see §5.2 for table-specific entries.

#### 4.0.2 Layout & Panel Ordering

- **Panel order convention:**
  - **Side-by-side** (1×2, n×2): Rotating Cage on the **left**, Fixed Cage on the **right**. The conditions list in code always reads `[('Rotating Cage', ...), ('Fixed Cage', ...)]`; col 0 = left, col 1 = right.
  - **Stacked** (2×1): Rotating Cage on **top**, Fixed Cage on the **bottom**. Row 0 = Rotating, Row 1 = Fixed.
  - **Single-panel comparison:** Rotating points first in legend, Fixed second. Trendlines: Rotating = dashed `--`, Fixed = solid `-`.
- **Color convention (when color differentiates conditions):** Rotating Cage = **blue** (`#1F77B4`), Fixed Cage = **red** (`#D62728`). Applies to scatter points, bar fills, line colours, and table backgrounds.
- **Enclosure styles (trendlines/overlays):** Rotating Cage = dashed `--`, Fixed Cage = solid `-`.

### 4.1 Sourcing & Note Text — Standard Templates

Every figure MUST carry source/origin information at the figure bottom, below all x-axis labels. Use `fig.text(..., transform=fig.transFigure)` so coordinates are figure-relative.

#### A) Data Origin (bottom-right, every figure)

```python
fig.text(0.98, 0.01,
         f"flights_summary (N={len(df_rot)} Rotating, N={len(df_fix)} Fixed)",
         ha='right', va='bottom', fontsize=7.5, color='#555555')
```

- The text MUST include **the exact dataframe variable name** and **the number of rows/filtered flights** in that dataframe. Examples:
  - `flights_summary (N=60 Rotating, N=67 Fixed)` — summary notebook with both conditions
  - `flights_summary (N=127)` — single-condition plot
  - `df_rot (N=60)` — direct variable reference
- `df_rot` / `df_fix` are the standard variable names for the two conditions. Use `len(df)` to get the count.
- Position: `x=0.98` (right-aligned), `y=0.01` (below x-axis ticks).
- Font: `fontsize=7.5`, `color='#555555'`.

#### B) Trendline Footnote (bottom-left, only on plots WITH trendlines)

**Ordinary Least Squares (summary notebook overlay plots):**
```python
fig.text(0.02, 0.01,
         "Note: Trendlines computed via ordinary least squares (y = mx + c).",
         ha='left', va='bottom', fontsize=7.5, fontstyle='italic', color='#555555')
```

**Robust Huber regression (EDA scatter plots, impact plots):**
```python
fig.text(0.08, 0.02,
         "Note: Trendlines computed via robust Huber regression to mitigate the influence of outliers (y = mx + c).",
         fontsize=8.5, fontstyle='italic', color='#555555')
```

#### C) Statistical Significance (ML/angle-prediction plots)

```python
fig.text(0.02, 0.015, r'$p < 0.05$',
         ha='left', va='bottom', fontsize=7.5, fontstyle='italic', color='#555555')
fig.text(0.98, 0.015, r'$p < 0.05$',
         ha='right', va='bottom', fontsize=7.5, fontstyle='italic', color='#555555')
```

#### D) Trendlines — Rule

**Only include a trendline when correlation/regression is the subject of the plot.** Do NOT add trendlines to boxplots, bar charts, pie charts, or heatmaps. Trendlines belong ONLY on:
- Scatter plots showing a relationship (impact angle vs velocity, deceleration vs battery, deviation vs angle)
- Actual-vs-predicted plots (y=x diagonal is the reference line; do not add a separate trendline)
- Learning curves (training/validation curves are not trendlines — they are data series)

### 4.2 Axis Limits & Tick Standards

- **All axes MUST have hardcoded explicit limits** — no dynamic/auto scaling. This ensures visual comparability across conditions and reproducibility.
- **When comparing two conditions in the same figure (side-by-side or stacked), both panels MUST share identical Y-limits and X-limits.** Use `sharex=True, sharey=True` on `plt.subplots()` wherever possible.
- **Ticks must be regular, evenly spaced intervals** — set explicitly via `ax.set_xticks()` / `ax.set_yticks()`. Never rely on matplotlib's auto-tick placement for final figures.
- **Fixed limits table (established per-plot SSoT):**

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
| Recovery Area boxplot | Recovery area | `0` – `6` cm² | every `1` |
| Deceleration vs Battery (split & global) | Deceleration | `0` – `6` m/s² | every `1` (now 0–3.5, every 0.5) |

- **Y-axis truncation for outliers:** When one extreme outlier compresses the visible range, cap the axis to a reasonable max (e.g., `set_ylim(0, 6)` for recovery area, `set_ylim(0, 3.5)` for deceleration after recent revision).

### 4.3 Additional Standards

- **100% pure Matplotlib** (no Seaborn)
- **Filenames:** Use `deg` not `°`
- **Data truncation:** Dynamically crop to `[Exp. Start-point − 1s, Exp. End-point + 1s]`. **Combined side-by-side plots** use `[Column Center Passed − 1s, Column Center Passed + 1.5s]` — timeline is centered on the column, not the start-point. Exp. Start-point line and label are suppressed.
- **Bold titles:** Every plot title uses `fontweight='bold'`. Wrapping condition in `< >` is **retired** — use plain `"Rotating Cage"` / `"Fixed Cage"` in black (`color='black'`).
- **Timeline events:** Three vertical lines: `Exp. Start-point` (dotted purple/grey), `💥 Impact` (dash-dotted crimson, label on LEFT), `Exp. End-point` (dotted purple/grey)
- **Impact window shading:** Light red `[t−0.05s, t+0.35s]` across all subplots
- **Battery bins:** Red [0–40%], Orange (40–60%], Yellow-Green (60–80%], Green (80–100%]

### 4.4 Colormaps

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
- **Correlation colormap (diverging):** `plt.cm.coolwarm` with `norm = plt.Normalize(vmin=-1.0, vmax=1.0)` — -1 = deep blue, 0 = white, +1 = deep red. For the **consolidated (ranked) heatmap** where features are sorted by |r| descending (best at top) with explicit shared ylim, **reverse the array** (`vals[::-1]`) before `imshow(origin='upper')` so row 0 of the reversed array (weakest) maps to the bottom of the axes and row N−1 (strongest) maps to the top, matching the text annotations.

### 4.5 Comparative Table Standards

- **Comparative tables (Table of Averages):**
  - Grouped by impact angle bins (30–40°, 40–50°, 50–60°, 60–90°) × cage condition
  - 6-pie-chart mission outcomes grid above table
  - Oscillating row order: Rotating→Fixed per bin
  - Explicit `N-Flights` column
  - Color coding: Rotating blue (`#d2e4f6` bg, `#0d233a` text), Fixed red (`#f7d2d2` bg, `#3d0b0b` text). Ratio heatmap for metrics columns (HSL green=better, red=worse)
  - Raw inline HTML (no `jinja2` dependency)

### 4.6 Sunburst / Nested Pie Charts

- One panel per cage condition (Rotating left, Fixed right)
- Outermost ring: Impact Detected vs No Impact in condition colour (Rotating = `#1F77B4` blue, Fixed = `#D62728` red)
- Inner ring(s): Per-mission angle distribution coloured by the RdYlGn angle colormap; grey for No Impact
- Percent labels inside wedges (`autopct`), omitting segments <3% for legibility
- Centre text: condition name + total N
- Figure-bottom annotation: data tables printed in both bottom-left and bottom-right corners showing the exact aggregated figures

### 4.7 Data Frame Annotation (Summary Plots)

- Two compact tables at figure-bottom showing the exact aggregated figures used:
  - Left-bottom under Rotating Cage panel: mission breakdown table
  - Right-bottom under Fixed Cage panel: mission breakdown table
- Format: plain monospace text via `fig.text()` or `ax.text()`

## 5. Master Plot Directory — Comprehensive Table of All Plots

All plots grouped by source notebook/script. Each table row = one plot, with all descriptive detail (layout, axis limits, legend, markers, sourcing, notes) in the Notes column. Empty cells = data not yet hardcoded (pending). Status: ✅ Done, 🔧 Needs work, ⏳ Pending implementation.

### 5.1 `experiments_analysis.ipynb` — Per-Flight Interactive Plots

Called on-the-fly with representative flight data. Some produce pipeline `passNN_*.png` outputs, others are inline-only. Notebook header reads "Interactive Analysis — Representative Flights" (covers both 45° and 75°).

| # | Plot Name | Function(s) | Core File | Output File | Notes | Pipeline | Status |
|---|-----------|-------------|-----------|-------------|-------|----------|--------|
| 1 | **Top-Down 2D Trajectory** | `plot_trajectory()` / `plot_trajectory_from()` | `kin_plot_trajectory.py` / `flight_loader.py` | `passNN_trajectory_top_down.png` | 1:1 aspect, 0.5m grid, rotated axes (−y, x), white label backgrounds. Limits `[-1.6,1.6]/[-0.5,1.0]`. Base layout for trajectory heatmaps. | Active | ✅ |
| 2 | **Full-Loop Geometry** | `plot_full_loop_geometry()` / `plot_full_loop_geometry_from()` | `kin_plot_trajectory.py` / `flight_loader.py` | — (thesis export) | Full sweep-plane geometry diagram with column, fence and cage annotations. Used in thesis §3. | Export-only | 🔧 |
| 3 | **MoCap Kinetic Profile — Raw** | `plot_velocity_profile_from(data, raw=True)` | `kin_plot_kinematics.py` / `flight_loader.py` | — (inline only) | 2-panel: Speed + Accel (no MoCap rate panel). Same crop/markers as EKF profile. Retained for interactive debugging. Speed: `-0.05–0.85` e.0.2; Accel: `-8.0–8.0` e.2.0. | Retired (pipeline) | ✅ |
| 4 | **MoCap Kinetic Profile — Splined** | `plot_velocity_profile_from(data)` | `kin_plot_kinematics.py` / `flight_loader.py` | — (inline only) | 3-panel: Speed + Accel + MoCap Rate. Shared X, uniform grid resampling + spline interpolation. Debugging view for MoCap rate diagnosis. Same axis limits as #3 + Rate `-10–480` Hz. | Retired (pipeline) | ✅ |
| 5 | **EKF Kinetic Profile** | `plot_ekf_kinetic_profile()` / `plot_ekf_kinetic_from()` | `kin_plot_kinematics.py` / `flight_loader.py` | `passNN_ekf_kinetic_profile.png` | 2-panel: velocity + tangential accel. PX4 `vehicle_odometry` on MoCap grid, `np.gradient` for accel. Same timeline crop/markers as MoCap profile. Speed: `-0.05–0.85` e.0.2; Accel: `-8.0–8.0` e.2.0. Title bold. | Active | ✅ |
| 6 | **EKF Velocity Viewer** | `plot_ekf_velocity_from(data)` | `flight_loader.py` | — (inline) | Side-by-side EKF vs MoCap velocity. EKF = smooth solid line, MoCap = dashed. Per-axis velocity `-0.4–0.6`. Standard bottom-right flight-name label. | Inline | ✅ |
| 7 | **EKF Dual Comparison** | `plot_ekf_dual_comparison(rot_data, fix_data)` | `flight_loader.py` | — (inline) | Two-panel: Rotating left, Fixed right — each with EKF velocity + tangential acceleration. Shared Y-limits per §4. Used to contrast velocity smoothness across cage types. | Inline | ✅ |
| 8 | **Battery Voltage Sag** | `plot_battery_sag()` / `plot_battery_sag_from()` | `kin_plot_kinematics.py` / `flight_loader.py` | — (inline) | Single-axis, battery voltage over time, vertical markers at waypoints. Per-flight diagnostics only. | Retired (pipeline) | ✅ |
| 9 | **Physical IMU Collision Dynamics** | `plot_imu_dynamics()` / `plot_imu_dynamics_from()` | `kin_plot_kinematics.py` / `flight_loader.py` | `passNN_imu_dynamics.png` | Dual Y (Linear Accel vs Gyro). Shock impulse integration, settling times. Accel left `-1.0–20.0` e.2.0; Gyro right `0–10` e.1. | Active | ✅ |
| 10 | **RAW IMU X/Y/Z Components** | `plot_imu_xyz_components()` / `plot_imu_xyz_from()` | `kin_plot_kinematics.py` / `flight_loader.py` | `passNN_imu_xyz_components.png` | 3 subplots (Lateral/X, Longitudinal/Y, Vertical/Z), height ratios `[26,26,20]`. Accel X `(-20,6)`/Y `(-6,20)`/Z `(-20,0)` e.2; Gyro X/Y `-1.5–1.5` e.0.5; Gyro Z `-1.0–5.0` e.1.0. | Active | ✅ |
| 11 | **Actuator Motor Commands** | `plot_actuators_and_status()` / `plot_actuators_from()` | `kin_plot_actuators.py` / `flight_loader.py` | `passNN_actuator_motor_commands.png` | 2-panel (vehicle status panel removed 2026-06-12 — showed nothing). Panel 1: Motor Cmd (normalized), ylim `0.4–1.0`. Panel 2: DShot output `750–2000`, xlabel `Time (s)`. | Active | ✅ |
| 12 | **Control Allocator Saturation** | `plot_control_allocator_saturation()` / `plot_allocator_from()` | `kin_plot_actuators.py` / `flight_loader.py` | `passNN_control_allocator_saturation.png` | Shows saturation events. Retired from notebook 2026-06-12 — redundant with actuator commands plot (same data, same events). Pipeline output still generated for completeness. | Active (pipeline) / Retired (notebook) | ✅ |
| 13 | **PID Rate Tracking — Per-Flight** | `plot_pid_rate_tracking()` / `plot_pid_tracking_from()` | `kin_plot_actuators.py` / `flight_loader.py` | `passNN_pid_rate_tracking.png` | 3-panel (roll, pitch, yaw rate setpoint vs actual). Retained in notebook as per-flight diagnostic. Aggregate PID tracking (Plot 2 in summary notebook) was retired — redundant with RMS error metrics in DB. | Inline (kept) / Retired (pipeline) | ✅ |

#### Combined Side-by-Side Plots (`experiments_analysis.ipynb`)

All use `get_combined_timeline_limits()` (`[Column Center Passed − 1s, Column Center Passed + 1.5s]`). All: Impact line THIN (lw=1.2), **Exp. Start-point line and label suppressed**, no Exp. End-point label. Bottom margin: `tight_layout(rect=[0, 0.035, 1, 0.96])`, source labels at `y=0.015`. Excluded from combined plot set: trajectory (no time axis) and battery sag.

| # | Plot Name | Function(s) | Core File | Output File | Notes | Status |
|---|-----------|-------------|-----------|-------------|-------|--------|
| 14 | **Combined EKF Kinetic Profile** | `plot_ekf_kinetic_combined()` / `plot_ekf_kinetic_combined_from()` | `kin_plot_kinematics.py` / `flight_loader.py` | `Combined EKF Kinetic Profile.png` | 2×2 grid, figsize=(16,10). Rotating left, Fixed right. Legend only left column (upper left). Source labels: left-bottom Rotating, right-bottom Fixed. | ✅ |
| 15 | **Combined IMU Dynamics** | `plot_imu_dynamics_combined()` / `plot_imu_dynamics_combined_from()` | `kin_plot_kinematics.py` / `flight_loader.py` | `Combined IMU Collision Dynamics.png` | 1×2 panels, figsize=(14,7), twin-Y per panel (accel left, gyro right). Legend only left panel (upper left). Per-column source labels. | ✅ |
| 16 | **Combined IMU XYZ** | `plot_imu_xyz_combined()` / `plot_imu_xyz_combined_from()` | `kin_plot_kinematics.py` / `flight_loader.py` | `Combined IMU XYZ Components.png` | 3×2 grid, figsize=(16,12). Twin axes per subpanel. Accel Y-labels on left column; gyro Y-labels on right column. Single stacked fig-level legend in right upper corner. | ✅ |
| 17 | **Combined Actuator Commands** | `plot_actuators_and_status_combined()` / `plot_actuators_combined_from()` | `kin_plot_actuators.py` / `flight_loader.py` | `Combined Actuator Commands.png` | 2×2 grid, figsize=(16,10). Y-labels only on left column; legends only on right column. Per-column source labels. | ✅ |
| 18 | **Combined PID Tracking** | `plot_pid_rate_tracking_combined()` / `plot_pid_tracking_combined_from()` | `kin_plot_actuators.py` / `flight_loader.py` | `Combined PID Tracking.png` | 3×2 grid, figsize=(16,12). Legend only left column (upper left). Per-column source labels. | 🔧 |

### 5.2 `experiments_analysis_summary.ipynb` — Aggregate Comparative Dashboard

All plot logic is offloaded to `summary_plots.py`. Notebook cells are thin one-liner calls.

| # | Plot Name | Function(s) in `summary_plots.py` | Output File | Notes | Status |
|---|-----------|-----------------------------------|-------------|-------|--------|
| 19 | **Recovery Area Boxplot** | `plot_recovery_area_boxplot(df_impacts)` | `recovery_area_comparison.png` | `set_ylim(0,270)`, ticks e.50. Jittered scatter overlay. Performance improvement printout below. | ✅ |
| 20 | **Deceleration vs Battery (split)** | `plot_deceleration_vs_battery_split(df_impacts)` | `deceleration_vs_battery_angle.png` | Split panels by condition, colored by impact angle (RdYlGn). Trendlines via Huber regression. | ✅ |
| 21 | **Deceleration vs Battery (global)** | `plot_deceleration_vs_battery_global(df_impacts)` | `deceleration_vs_battery_global.png` | Single panel both conditions overlaid. Same Y-limits as #20 (0–3.5 e.0.5). | ✅ |
| 22 | **Comparative Overlays (Deviation vs Angle)** | `plot_deviation_vs_angle_overlay(df_rot, df_fix)` | `comparative_cage_deviation_overlay.png` | Battery color bins. Rotating=dashed, Fixed=solid trendlines. Monospace legend with slopes. | ✅ |
| 23 | **IMU Peak Accel Z vs Motor RPM (Plot A)** | `plot_imu_z_vs_motor_rpm(df_impacts)` | `plot_A_imu_z_vs_rpm.png` | X=RPM, Y=`imu_peak_accel_z`. Rotating=squares, Fixed=circles. Color by impact angle (`coolwarm`). | ✅ |
| 24 | **Peak Accel + Rotational Energy Boxplots** | `plot_peak_accel_rotational_energy(df_impacts)` | — | 1×2 boxplots: imu_peak_accel (g) + imu_gyro_energy (rad). | ✅ |
| 25 | **Allocator Saturation (Plot 17)** | `plot_allocator_saturation(df_impacts)` | `plot_17_allocator_saturation_comparison.png` | 1×3 boxplots: saturation duration, unallocated torque, thrust achieved %. | ✅ |
| 26 | **PID Tracking Error (Plot 18)** | `plot_pid_tracking_error(df_impacts)` | `plot_18_pid_tracking_comparison.png` | 1×3 boxplots: roll/pitch/yaw rate error RMS. | ✅ |
| 27 | **Sunburst / Nested Pie Charts** | `plot_sunburst_impact_distribution()` | `plot_14_sunburst_revised.png` | Hardcoded aggregated data. Outer ring: Impact/No Impact in cage colour. Inner rings: RdYlGn angle distribution. Legend + source note below. | ✅ |
| 28 | **Aggregated IMU Collision Dynamics** | `generate_aggregated_imu_plot()` in `plot_aggregated_imu_dynamics.py` | `plot_19_aggregated_imu_dynamics.png` | 1×2 side-by-side (Rotating left, Fixed right). Timeline [−0.5 s, +1.0 s] from Column Impact. Accel Y-labels on left panel, gyro Y-labels on right panel. Legend only left panel. Per-column source labels. | ✅ |
| 29 | **IMU Acceleration Spread Boxplots** | `plot_imu_spread()` in `kin_plot_imu_spread.py` | `plot_16_imu_vibration_spread.png` | Impact vs Regular flight windows. Per-axis (ax/ay/az), g units. | ✅ |
| 30 | **Motor Aggregate Comparison** | `plot_motor_aggregates()` in `plot_motor_aggregates.py` | — | Motor command metrics across conditions. | 🔧 |
| 31 | **Comparative Table of Averages** | `render_comparison_table_html(df_all)` | — | Grouped by angle bins × cage condition. Ratio heatmap (HSL green=better, red=worse). Raw inline HTML. | ✅ |
| 32 | **EKF vs MoCap Comparison Figure** | Inline cell | — | Side-by-side velocity traces. Thesis §4 evidence. | ✅ |
| — | **Battery Efficiency** | `plot_battery_efficiency_comparison(df_eff)` | `plot_10a/b/c_*.png` | 3-panel: duration boxplot, drain rate boxplot, voltage-vs-duration scatter. | ✅ |
| — | **Mission Outcome Pies** | `plot_mission_outcome_pies(df_all)` | `plot_13_mission_pies.png` | 3×2 grid: Rotating|Fixed × (Total, 45°, 75°). | ✅ |
| — | **Recovery Area Distribution** | `plot_recovery_area_distribution(df_impacts)` | `plot_15_recovery_distribution.png` | Horizontal boxplots by angle bin. | ✅ |
| — | **2D Path Overlay** | `plot_2d_path_overlay()` | `plot_16_path_heatmap.png` | All flights overlaid per cage. Uses trajectory_cache.pkl. | ✅ |
| — | **Attitude-Shock Phase Portrait** | `plot_attitude_shock_phase_portrait(df_impacts)` | `advanced_thesis_highlights.png` | 1×2: Phase portrait + Y-axis vibration spread. | ✅ |
| — | **Impact Angle vs Max Deviation (Plot B)** | `plot_impact_angle_vs_max_deviation(df_impacts)` | `plot_14_angle_vs_deviation.png` | Scatter colored by battery. OLS trendlines. | ✅ |

### 5.3 `eda/eda_angle_prediction.py` — Exploratory Feature Analysis (Impact-Angle Prediction)

| # | Plot Name | Function(s) | Output File | Notes | Status |
|---|-----------|-------------|-------------|-------|--------|
| 33 | **Correlation Heatmap** | `plot_correlation_heatmap(df, condition)` | `eda_corr_heatmap_{condition}.png` | Single condition. `coolwarm` colormap, `vmin=-1.0, vmax=1.0`. Feature × feature Pearson r matrix. | ✅ |
| 34 | **Consolidated Feature Correlation** | `plot_consolidated_feature_correlation(df_fix, df_rot)` | `eda_consolidated_feature_correlation.png` | Dual heatmap (Rotating left, Fixed right) with spline arcs between correlated pairs. `coolwarm`, `vmin=-1.0, vmax=1.0`. Spline colours via same `cmap(norm(mean_r))` so they match cell colours. Values reversed (`[::-1]`) before `imshow(origin='upper')` to align with text annotations. Features sorted by |r| descending (best at top). Spline formulas: lw = `mean_abs_r * 4.0`, alpha = `0.2 + mean_abs_r * 0.8`. Tight key-column width ratios `[0.9, 0.7, 0.4, 0.7, 0.9]` (10" total). Rank-numbered Cage keys. Data origin footnote per §4.1. Colorbar on far right. | ✅ |
| 35 | **Top-3 Feature Scatter** | `plot_top3_scatter(df, condition)` | `eda_scatter_top3_{condition}.png` | Per-feature scatter plots for top-3 correlated features + Huber robust trendline (δ=1.345, IRLS algorithm). | ✅ |
| 36 | **Parallel Coordinates** | `plot_parallel_coordinates(df, condition)` | `eda_parallel_coordinates_{condition}.png` | Single condition. Y: `(-0.05, 1.05)`. Line color by impact angle via `coolwarm`. | ✅ |
| 37 | **Consolidated Parallel Coordinates** | `plot_consolidated_parallel_coordinates(df_fix, df_rot)` | `eda_consolidated_parallel_coordinates.png` | Both conditions side-by-side. Y: `(-0.05, 1.05)`. Feature names only on bottom plot. | ✅ |
| 38 | **Consolidated Top Features** | `plot_consolidated_top_features(df_fix, df_rot)` | `eda_consolidated_top_features.png` | 3×2 scatter grid, no title spacer row. Rotating left, Fixed right. Each cage gets its own top-3 by |r|, **guaranteed** `imu_ax_spread_impact` at row 0 (inserted if not naturally top-3). **Y-labels: `#<actual_rank>` from full 26-feature IMU correlation table** — not grid position. Huber trendlines per panel. Column sub-titles via `fig.text()` above axes. §4.1 data-origin footnote, §4.1B Huber footnote (3-line), §4.1C stat key. `figsize=(12, 8.5)`, GridSpec `hspace=0.20, wspace=0.18`. Colorbar: horizontal, bottom-centred. | ✅ |

### 5.4 `eda/eda_impact_to_end.py` — Impact-to-End Distribution Analysis

| # | Plot Name | Function(s) | Output File | Notes | Status |
|---|-----------|-------------|-------------|-------|--------|
| 39 | **Impact-to-End Histograms** | `plot_histograms(df)` | `impact_to_end_histograms.png` | Distribution of time from impact to sweep end. Normal MLE overlay (μ, σ with ddof=1). | ✅ |
| 40 | **Impact Angle vs Time** | `plot_angle_vs_time(df)` | `impact_angle_vs_time.png` | Achieved impact angle over the course of the experiment session. Checks for drift/learning effects. | ✅ |

### 5.5 `models/rf_angle_prediction.py` — Random Forest Angle Prediction

| # | Plot Name | Function(s) | Output File | Notes | Status |
|---|-----------|-------------|-------------|-------|--------|
| 41 | **Feature Importance (MDI + Permutation)** | `plot_consolidated_feature_importance(results_fix, results_rot)` | `rf_feature_importance.png` | Dual panel: MDI (Mean Decrease in Impurity) left, Permutation importance (20 repeats) right. Both conditions overlaid per panel. | ✅ |
| 42 | **Actual vs Predicted** | `plot_consolidated_actual_vs_predicted(results_fix, results_rot)` | `rf_actual_vs_predicted.png` | y=x diagonal reference line. ±5° shaded band. Both conditions in separate colours. | ✅ |
| 43 | **Residuals** | `plot_consolidated_residuals(results_fix, results_rot)` | `rf_residuals.png` | Residuals vs predicted, zero line reference. Checks for heteroscedasticity. | ✅ |
| 44 | **Model Comparison (RF vs Huber baseline)** | `plot_consolidated_model_comparison(results_fix, results_rot)` | `rf_model_comparison.png` | RF (200 trees, `max_features='sqrt'`) vs top-1-feature Huber linear model. Scatter per condition. | ✅ |
| 45 | **Cross-Condition Transfer** | `plot_consolidated_cross_condition_transfer(results_fix, results_rot)` | `rf_cross_condition_transfer.png` | Train on Fixed Cage → test on Rotating Cage (and vice versa). Tests model generalisation. | 🔧 |
| 46 | **Learning Curves** | `plot_consolidated_learning_curves(results_fix, results_rot)` | `rf_learning_curves.png` | R² vs training size, 8 points from 30% to 100%. Stratified 5-fold CV. | ✅ |

### 5.6 `export_thesis_plots.py` — Thesis Export Pipeline

| # | Plot Name | Notes | Status |
|---|-----------|-------|--------|
| 47 | **Thesis Plot Exporter** | Batch-export all thesis-ready plots from both notebooks to `thesis/plots/` at 300 DPI. Routes combined plots, trajectory, actuator, IMU, and summary graphics via `run_flight_loader_plots()` and `run_geometry_plot()`. | ✅ |

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

## 9. Calculation Techniques Reference

This section documents every mathematical technique, algorithm, and signal-processing method used across the analysis pipeline, organized by domain.  Each entry states **what** is computed, **how** it is computed, and **where** it lives.

### 9.1 Velocity & Acceleration Derivation

#### 9.1.1 MoCap Position Resampling (`kin_calculator.py:6–71`)
**What:** Raw MoCap pose data arrives at irregular 10–120 Hz.  We resample onto a uniform 100 Hz grid for compatible differentiation.

**How:**
- **Linear interpolation** (`np.interp`) — preserves raw data character (no PCHIP overshoot)
- **Dropout detection:** `np.diff(raw_t) > 1/30` flags gaps.  If >20 gaps → "high-jitter" (Fixed Cage), only repair gaps >100 ms with tight 2-sample margin.  If ≤20 gaps → "low-jitter" (Rotating Cage), mark gaps + 9-sample SG-window margin.
- **Ringing mask:** Boolean mask marking gap + margin regions.  Used downstream to surgically excise SG boundary artifacts (ringing at discontinuity edges).

#### 9.1.2 Savitzky-Golay Velocity Differentiation (`kin_calculator.py:73–202`)
**What:** Derive velocity and acceleration from resampled position on a uniform 100 Hz grid.

**How:**
- **SG filter** with `window_length=19` (≈190 ms), `polyorder=3` (cubic), `deriv=1`
- **Scaling:** `savgol_filter(..., deriv=1) / median_dt` — converts from sample-derivative to physical m/s
- **Adaptive window:** Shrunk to `n_points // 2 * 2 - 1` if data shorter than window, clamped to ≥3
- **Fixed Cage Butterworth:** 2nd-order, 4 Hz cutoff, zero-phase (`filtfilt`).  Applied after SG diff on velocity arrays to suppress dropout kink amplification (~10 Hz kinks vs. ~4 Hz collision dynamics).
- **Disabled approach (2026-06-08):** 12 Hz Butterworth pre-filter on position *before* SG diff.  Failed because 12 Hz cutoff too close to 10 Hz kink frequency — couldn't separate noise from signal.
- **Surgical ringing removal:** After SG diff, linearly interpolate over ringing mask regions (replaces SG boundary spikes with clean straight-line bridges).

#### 9.1.3 EKF Velocity (PX4 Onboard Odometry) (`kin_calculator.py:204–313`)
**What:** Use PX4 Extended Kalman Filter velocity estimates instead of MoCap-derived velocity.  The EKF fuses MoCap position (10–120 Hz) + IMU acceleration (250 Hz) internally, producing a velocity signal that is inherently smooth even during MoCap dropouts.

**How:**
- **Coordinate alignment:** NED→ENU: `vx = vx_raw`, `vy = −vy_raw`, `vz = −vz_raw` (matches `build_dataframes` convention)
- **Resample to 100 Hz** via `interp1d(kind='linear')` onto MoCap-aligned time grid
- **Speed:** `sqrt(vx² + vy² + vz²)`
- **Acceleration:** SG `deriv=1` on velocity (same window=19, polyorder=3 as MoCap pipeline), fallback `np.gradient`
- **Override in metrics:** When `df_ekf_kin` is provided to `calculate_metrics()`, all velocity/accel columns in `df_mocap` are replaced with EKF values via `np.interp` onto MoCap time grid before metric extraction

### 9.2 Waypoint Detection & Column Impact Timing

#### 9.2.1 Three-Tier Waypoint Detection (`kin_calculator.py:373–618`)
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

#### 9.2.2 WP2 Refinement (`kin_calculator.py:563–588`)
**What:** The WP2 command timestamp marks when PX4 *accepts* the exp_sp waypoint — but the drone may still be stationary.  Refine to the moment forward motion actually begins.

**How:**
1. Find when drone crosses Y = 0.70 m (toward WP3 at Y = −1.2)
2. In the pre-cross window, find the **last** timestamp where speed < 0.10 m/s
3. Falls back to minimum-speed point in window, then to end-of-window

#### 9.2.3 Column Passed & Column Impact (`kin_calculator.py:590–613`)
**How — Column Passed:**
- In sweep window [WP2, WP3]: `argmin |y − column_y|` — instant of closest Y to column

**How — Column Impact (collision instant):**
- **Proximity gate:** Minimum 2D distance from drone to column center must be ≤ 0.38 m (≈cage radius + column radius + margin)
- **Speed peak search:** In window `[t_closest − 0.25 s, t_closest + 0.1 s]`, find `argmax speed` — the instant of peak deceleration onset
- Falls back to geometric closest-approach time if no speed data

### 9.3 Clearance, Collision & Path Tracking

#### 9.3.1 Closest Clearance (`kin_calculator.py:761–765`)
**What:** Minimum surface-to-surface distance between drone cage and column.

**How:**
- `dist_to_col_center = sqrt((x − column_x)² + (y − column_y)²)`
- `closest_clearance = min(dist) − column_radius − cage_radius`
- Negative value → physical penetration (collision detected)

#### 9.3.2 Perpendicular Path Tracking Error (`kin_calculator.py:695–702`)
**What:** Lateral deviation from the ideal straight line WP2→WP3.

**How:**
- Point-to-line distance formula: `|(y₂−y₁)x₀ − (x₂−x₁)y₀ + x₂y₁ − y₂x₁| / √((y₂−y₁)² + (x₂−x₁)²)`
- Applied to every MoCap sample in the sweep window
- **Metrics:** Mean error (accuracy), Max error (worst excursion), Std Dev × 1000 = `path_spread_sdld` (mm)

#### 9.3.3 Recovery Area (`kin_calculator.py:846–875`)
**What:** Integrated spatial error envelope after collision, measuring how much the drone deviates from the nominal path.

**How:**
- **Path projection:** Each post-impact point is projected onto the nominal WP2→WP3 line via dot product with unit direction vector: `s = (p − wp2) · û`
- **Sort by s** (travel distance along path) to ensure monotonic integration
- **Trapezoidal integration:** `np.trapz(d_sorted, s_sorted) × 1000` → mm·m (area under deviation curve)

#### 9.3.4 Impact Angle (`kin_calculator.py:818–836`)
**What:** Angle between the drone's velocity vector and the collision normal (vector from drone to column center).

**How:**
- Collision normal: `r = column_center − drone_position`
- Velocity vector: `v = (vx, vy)` at impact instant
- `cos(θ) = (r · v) / (|r| · |v|)`, clamped to [−1, 1]
- `θ = arccos(cos(θ))` in degrees
- If θ > 90°, reflect: `θ = 180 − θ`

### 9.4 IMU Shock & Structural Dynamics

#### 9.4.1 IMU Signal Preprocessing (`db_loader.py:244–247`)
**How:**
- **Acceleration magnitude:** `a_mag = sqrt(ax² + ay² + az²)`
- **Deviation from gravity:** `a_deviation = |a_mag − 9.81|` — isolates collision forces from steady 1g
- **Angular velocity magnitude:** `g_mag = sqrt(gx² + gy² + gz²)`

#### 9.4.2 Peak Shock Extraction (`kin_calculator.py:888–921`)
**What:** Maximum linear and rotational shock during collision.

**How:**
- **Contact window:** [t_impact − 0.05 s, t_impact + 0.35 s] — 400 ms total
- **Peak accel:** `max(a_deviation)` over contact window, per-axis `max(|ax|)`, Z-axis `max(|az + 9.81|)` (restores absolute including gravity)
- **Peak gyro:** `max(g_mag)`, per-axis `max(|gx|)`, etc.

#### 9.4.3 Integrated Shock Energy (`kin_calculator.py:923–943`)
**What:** Time-integrated IMU magnitude over the 400 ms contact window — measures total shock "dose."

**How:**
- **Trapezoidal integration:** `np.trapz(signal, t_vals)` — numerically integrates the area under the signal curve
- Applied to `a_deviation`, per-axis `|ax|`, `|az+9.81|`, `g_mag`, per-axis `|gx|`, etc.
- Units: accel energy [g·s], gyro energy [rad]

#### 9.4.4 Settling Time (`kin_calculator.py:945–958`)
**What:** Time from impact until shock amplitude returns to baseline.

**How:**
- **Accel settling:** Last timestamp where `a_deviation ≥ 1.5 m/s²`
- **Gyro settling:** Last timestamp where `g_mag ≥ 0.5 rad/s`
- If no samples above threshold → 0.0 (immediate recovery)

#### 9.4.5 Vibration & Spread (`kin_calculator.py:912–920, 960–976`)
**What — Vibration (post-impact):** Std dev over stabilization window [t+0.2 s, t+3.0 s] (skip first 0.2 s to exclude initial shock spike).  Measures post-collision oscillation amplitude.

**What — Spread (impact vs. regular):**
- Impact spread: `std(ax)` over [t−50 ms, t+350 ms] (400 ms), in g
- Regular spread: `std(ax)` over [t−1050 ms, t−50 ms] (1.0 s pre-impact), in g
- Captures high-frequency "buzz" vs. normal flight vibration

#### 9.4.6 Aggregated IMU Display (`db_cache_imu.py` → `plot_aggregated_imu_dynamics.py`)
**How — Cache:**
- For each impact flight: load IMU from MCAP, align to Column Impact time, extract `[t_rel −1.0, +2.0]` window
- Store `t_rel`, `a_deviation`, `g_mag` arrays in pickle
- **Why:** Avoids re-loading 179 MCAPs (200MB+ each) for every aggregated plot regeneration

**How — Plot:**
- Group traces by condition, compute per-timestep mean ± SEM across all flights
- Display as time-aligned overlay with shaded confidence bands

### 9.5 Motor & Actuator Analysis

#### 9.5.1 MCAP-ULog Clock Synchronization (`db_pipeline.py:70–145`)
**What:** Align MCAP bag-relative timestamps with PX4 ULog microsecond timestamps.

**How — Primary (timesync_status):**
- Read `/fmu/out/timesync_status` from MCAP → `observed_offset` (ns)
- `offset_sec = −(observed_offset × 1e−6)`
- Converts ULog µs to MCAP-relative seconds: `(t_us × 1e−6) + offset_sec − bag_start_ns/1e9`

**How — Fallback (EKF velocity correlation):**
- Pick one `vehicle_odometry` message from MCAP (ROS timestamp + velocity)
- Scan all `vehicle_local_position` messages in ULog for closest velocity vector (minimum squared error)
- Compute offset from matching timestamps

#### 9.5.2 Motor Command Metrics (`db_pipeline.py:148–245`)
**What:** Extract motor response to collision from ULog `actuator_motors`.

**How:**
- **Average signal:** `(control[0] + control[1] + control[2] + control[3]) / 4.0`
- **Before window:** [t_impact − 1.0, t_impact) → `avg_before`, `max_before`
- **After window:** [t_impact, t_impact + 1.0] → `avg_after`, `max_after`
- **Thrust surge:** `max_after − avg_before` — how much harder motors work after impact
- **Imbalance (0.4 s):** Mean of per-timestamp `std(control[0:4])` over [t_impact, t_impact+0.4s] — measures asymmetric motor response
- **Max actuator output:** Scan `actuator_outputs` for peak PWM, convert: `(PWM − 1000) / 10.0 → %`

#### 9.5.3 Control Allocator Saturation (`db_pipeline.py:271–335`)
**What:** Time spent at actuator saturation limits in 1 s post-impact.

**How — 3-tier fallback:**
1. **actuator_motors (10 Hz):** Saturation if any `control[i] ≥ 0.999` or `≤ 0.001`.  Sum `dt` of saturated samples.
2. **actuator_outputs (50 Hz):** Saturation if any output ≥ 1999 (or 2000) or ≤ 115 (or 1000).  Auto-detects PWM range.
3. **control_allocator_status:** Saturation if any `actuator_saturation[i] != 0`.
- **Time-weighted:** Sum of `dt` values for saturated samples, not sample count
- **Unallocated torque norm:** `sqrt(tx² + ty² + tz²)` from control_allocator_status
- **Thrust achieved %:** `mean(thrust_setpoint_achieved) × 100`

#### 9.5.4 PID Rate Tracking Error (`db_pipeline.py:354–375`)
**What:** RMS error between commanded angular rate setpoint and actual angular velocity.

**How:**
- Interpolate `vehicle_rates_setpoint` timestamps onto `vehicle_angular_velocity` timestamps via `np.interp`
- **Error:** `roll_err = actual_xyz[0] − interpolated_roll_sp`
- **RMS:** `sqrt(mean(err²))` for roll, pitch, yaw over [t_impact, t_impact+1.0s]

### 9.6 Battery Efficiency

#### 9.6.1 Flight-Level Battery Analysis (`db_unsliced_flights_bat_analyser.py`)
**What:** Battery voltage and capacity drain rates computed from full-flight (unsliced) MCAPs.

**How:**
- **Memory-safe streaming:** Only 4 topics loaded from MCAP (poses, battery_status, vehicle_status, vehicle_odometry) — avoids OOM on 200MB+ files
- **Milestones:** Arm, Takeoff (Z > 0.15 m), Landing (last Z > 0.15 m), Disarm
- **Voltage drop rate:** `(V_takeoff − V_landing) / flying_min` [V/min]
- **Capacity drain rate:** `(SOC_takeoff − SOC_landing) / flying_min` [%/min]
- **Negative prevention:** `max(0.0, x)` on all drop values
- **Per-pass lookup:** `db_pipeline.py` reads flight-level rates from `flights_battery_efficiency` table (per-pass MCAP windows are ~10 s — too short for meaningful battery trend)

#### 9.6.2 Nearest-Timestamp Battery Query (`kin_calculator.py:620–626` → `db_unsliced_flights_bat_analyser.py:115–121`)
**How:** Binary search via `np.searchsorted(t_array, t_query)`, clamped to [0, len−1], returns nearest value.

### 9.7 Statistical & ML Analysis

#### 9.7.1 Trendlines & Regression
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

#### 9.7.2 Correlation Analysis
**How — Pearson r (`scipy.stats.pearsonr`):**
- Measures linear correlation strength and direction
- Significance: `*** p<0.001`, `** p<0.01`, `* p<0.05`

**How — Spearman ρ (`scipy.stats.spearmanr`):**
- Rank-based correlation — captures monotonic (not just linear) relationships
- Used alongside Pearson in RF feature selection to catch non-linear signals

#### 9.7.3 Statistical Tests (Between-Condition Comparison)
**How — Welch's t-test (`scipy.stats.ttest_ind, equal_var=False`):**
- Tests for difference in means without assuming equal variance
- Parametric (assumes approximate normality)

**How — Mann-Whitney U (`scipy.stats.mannwhitneyu`):**
- Non-parametric rank-sum test — no distribution assumption
- Used alongside Welch's for robustness

**How — Cohen's d effect size:**
- `d = (μ₁ − μ₂) / √((σ₁² + σ₂²) / 2)` — pooled standard deviation
- Interpret: |d| < 0.2 = negligible, 0.2–0.5 = small, 0.5–0.8 = medium, >0.8 = large

#### 9.7.4 Normal Distribution Fit (`eda_impact_to_end.py:161–165`)
**How:** `scipy.stats.norm.pdf(x, μ, σ)` with MLE parameters: μ = sample mean, σ = sample std (ddof=1).  Overlaid on density histogram for visual normality check.

#### 9.7.5 Random Forest Angle Prediction (`models/rf_angle_prediction.py`)
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

### 9.8 Data Storage & Caching

#### 9.8.1 SQLite Schema Migration (`db_manager.py:131–333`)
**How:** `CREATE TABLE IF NOT EXISTS` for initial schema, then `ALTER TABLE ADD COLUMN` for each new column (catches `OperationalError` if already exists) — incremental, backwards-compatible schema evolution.

#### 9.8.2 Trajectory Cache (`db_cache_trajectories.py`)
**How:** Query DB for impacted flights, load each MCAP once, extract `(x, y, z, t)` arrays + column position, pickle to disk.  179 flights × ~10 s trajectories — loading from cache is instant vs. minutes from raw MCAP.

#### 9.8.3 Pass Exclusion Config (`db_manager.py:42–121`)
**How:** JSON-based `config_db.json` maps `folder_name → {excluded: true, reason, notes, passes: [...]}`.  Human review overrides the clearance-based `impact_detected` heuristic.  `apply_exclusion_config_to_db()` syncs to SQLite `excluded` column.

## 10. Execution Log — Italic Notes (Most Recent First)

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
