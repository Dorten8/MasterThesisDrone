# Copilot instructions for `MasterThesisDrone`

## 🎯 Central Thesis Aim — The North Star

**Everything** in this project — the drone design, the experiments, the analysis, and the thesis manuscript — exists to answer these two research questions:

> **RQ1:** How does the physical resilience and energy deflection capability of a freely rotating protective cage compare to a fixed cage during direct obstacle collisions?

> **RQ2:** To what extent can the raw inertial data generated during these deliberate physical collisions be utilized to accurately estimate the angle of impact against an obstacle, serving as a fundamental step toward tactile spatial awareness in signal-denied environments?

Every experiment, every analysis plot, every section of the thesis manuscript should be traceable back to at least one of these questions. When writing, analyzing, or designing, ask: *Does this help answer RQ1 or RQ2?* If not, it's scope creep.

**The narrative arc:** Mechanical resilience (RQ1, Part I of Experiments) → Impact sensing and interpretation (RQ2, Part II of Experiments). The rotating cage is the intervention; the IMU is the sensor; the Motion Capture environment is the ground truth.

## Build, test, and lint commands

### ROS 2 workspace (this repo as `/home/ws`)
```bash
source /home/ws/install/setup.bash
```

### MAVLink Router (submodule: `mavlink-router/`)
```bash
cd /home/ws/mavlink-router
meson setup build . -Dsystemdsystemunitdir=no
ninja -C build
```

```bash
# full C++ test run (Meson/GTest tests defined in src/meson.build)
ninja -C build test
```

```bash
# single test
meson test -C build endpoints
```

```bash
# integration test script used in CI
./tests/routing_test.py -b ./build/src/mavlink-routerd
```

```bash
# lint/static checks available through Meson targets
ninja -C build clangtidy-check
ninja -C build clangtidy-autofix
```

## High-level architecture

- The repo is a ROS 2 Humble workspace mounted as `/home/ws` inside the devcontainer; active ROS packages live under `/home/ws/src`.
- Core runtime chain for indoor localization/control is:
  1. `mocap` (OptiTrack client + filtering) publishes ROS 2 pose topics.
  2. `mocap_px4_bridge` converts mocap pose into PX4 `VehicleOdometry` and publishes to `/fmu/in/vehicle_visual_odometry`.
  3. PX4 consumes this over uXRCE-DDS.
- **CRITICAL OFFBOARD RULE:** When creating or adjusting any offboard control node (like `offboard_control.py`), **ALWAYS** use the code from the official `px4_ros_com` examples as the foundation. This ensures we use the proper official methodology for setpoints and mode switching.
- `mocap_px4_bridge/launch/run.launch.py` starts both `mocap` (OptiTrack launch) and the bridge node; topic wiring is parameterized in `mocap_px4_bridge/config/params.yaml`.
- MAVLink control/telemetry is handled separately through `mavlink-routerd` using `config/mavlink-router/main.conf`:
  - UART endpoint: `/dev/ttyAMA0` at `921600`
  - UDP endpoint: `192.168.74.9:14550`
- `drone_controller/` contains Python joystick/manual-control tooling (`pymavlink`, `pyserial`) for direct MAVLink command flow.

## Project Ecosystem
These links represent the different dimensions of the project. Reference them when discussing design, documentation, or research.
* **Thesis Manuscript (local — `thesis/`):** `/home/dorten/MasterThesisDrone/thesis/`
* **Design & Ideation (Figma):** Master Thesis Board
* **Research & Notes (Notion):** Thesis Management
* **Project Data (OneDrive):** Raw Data & Assets

## User Profile

**Developer:** Dorten
- **Background:** Architecture, Construction, and Machinery.
- **Terminal Shell:** Fish 🐟 (not Bash)
- **Dev Machine:** Ubuntu 22.04 on Legion laptop
- **Command Explanations:** Provide explanations phonetically/logically. Always use Fish syntax (`set -x` for env vars, `fish` script headers).
- **Pi hostname:** `dorten-pi5drone` (mDNS: `dorten-pi5drone.local`)
- **Pi network:** 192.168.74.x range (verify with `hostname -I` on Pi)
- **Learning Style:** Prefers "nudges" and technical logic over large, unexplained code blocks. Explain the *why* behind the *how*.
- **Working Memory:** Use clear, specific instructions and maintain a coherent, holistic project view.

## Key repository conventions

- **PLOTTING SSoT — `experiments_analysis_skill.md`**: The file `dev_logs/analysis/experiments_analysis_skill.md` is the Single Source of Truth for ALL plotting conventions. §4 (Universal Plotting Standards) defines axis limits, colormaps, sourcing, layout, and panel ordering. §5 (Master Plot Directory) lists every plot across all notebooks with its function, file, limits, and status. **When editing any plot in any notebook**, first consult §4 and §5, update the relevant row in §5, and verify no §4 rule is broken. The same applies when modifying plot-generating Python scripts in `kinematics/`, `eda/`, `models/`, or `graphics/`.: If the project directory path (e.g., `/home/dorten/pi_drone_sshfs/`) contains **the literal string `sshfs`** in its name, then it is the network-mounted SSHFS location — **NEVER RUN ANY GIT COMMITS, STAGING (git add), OR RESET COMMANDS IN AN SSHFS-MOUNTED WORKSPACE**, as it locks the index (`.git/index.lock`) over the network mount and blocks the user. In that case, prepare the raw commit command text for the user to run from the drone's terminal (`dorten@dorten-pi5drone:/home/ws`). If the path does **not** contain `sshfs`, then git operations are safe to run directly.
- **CRITICAL DATABASE & PLOTTING PIPELINE RULE (CORE MEMORY)**: Running the complete telemetry database population pipeline (`db_pipeline.py` or executing the full `experiments_analysis.ipynb`) is extremely resource-intensive and takes significant time because it parses raw MCAP telemetry streams for over 170 passes. **NEVER execute a full database rebuild or run any batch population script directly, and NEVER ask the user to do so, without explicitly asking for permission first!** Always check if cached database entries can be reused or if only specific flights need to be re-analyzed before suggesting a full run. It is ALWAYS preferred to ask the user to run the script themselves in their terminal to ensure clear execution visibility.
- Read `README.md` intro first before making architectural or workflow changes.
- Treat this repository root as the ROS workspace root (`/home/ws`), not just a source checkout.
- Keep/expect these ROS env defaults in container sessions:
  - `ROS_DOMAIN_ID=0`
  - `ROS_LOCALHOST_ONLY=0`
  - `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- Serial link convention across components is `/dev/ttyAMA0` with baud `921600`.
- `/dev/ttyAMA0` is single-owner at runtime: do not run `micro_ros_agent` and `mavlink-routerd` against it simultaneously.
- **CRITICAL SERIAL BUG**: Do not kill the `MicroXRCEAgent` once it is connected. Doing so freezes the PX4 Flight Controller client and requires a hardware reboot.
- For PX4↔ROS 2 bridge work, use `src/micro-ROS-Agent` (ROS package). Do not use `src/Micro-XRCE-DDS-Agent` in colcon workflows.
- **Mocap SSoT**: `config/drone_config.json` is the Single Source of Truth for the drone's tracker name (via the "primary" role). The string in this JSON must exactly match the Rigid Body name exported by Motive.
- **CRITICAL MoCap Hurdle**: The Motive application can silently lose tracking of the drone's rigid body without error messages. When this happens, `motion_capture_tracking_node` runs but publishes empty pose arrays, causing `mocap_px4_bridge` to hang indefinitely. **Before running startup, always verify in the Motive GUI that the drone's rigid body is actively tracked and streaming (check for "~980 B/frame" data rate in Motive's Streaming panel).** If tracking is lost, physically move the drone in the capture volume to force Motive to re-detect it.
- **Motive Streaming Configuration (Verified Stable Settings):**
  - Camera Frame Rate: **360 Hz** (default was 120 Hz; user discovered 360 Hz is optimal for this cage setup)
  - Data Port: 1511, Command Port: 1510
  - Multicast Address: 239.255.42.99
  - Local Interface: 192.168.74.3 (OptiTrack PC static IP)
  - All 6 cameras enabled with 250 µs exposure and LED enabled
  - Target streaming rate: ~980 B/frame, final rate ~358 Hz
  - Tracking residual should be < 1 mm (0.72 mm is good)
- **Coordinate Frames**: PX4 strictly requires NED (+X physical front, +Z physical down). The Motive Rigid Body pivot must be manually aligned so its internal X-axis points out the physical nose of the drone.
- **Visualization:** Foxglove bridge runs on Pi port 8765 for real-time browser viewing on laptop.

## Current Session Status (Last Update: 2026-06-10 ~20:45 Local Time)

### 🎯 Mission Status
- **Diagram pipeline transitioned to vector PDFs:** ✅ DONE.
  - `.mmd` is now the standard diagram source format (pure Mermaid syntax, no markdown fences).
  - `plot_diagrams.sh` rewritten as a universal `*.mmd` → `../Figures/*.pdf` batch renderer.
  - Both diagrams (`architecture.mmd`, `experiments_pipeline.mmd`) render and compile into the thesis.
  - EKF filter implementation is now in progress (see below).
  - User needs a VS Code Mermaid extension for `.mmd` file preview.
- **`ExpCollision75Deg` and `ExpCollision75DegV2` missions:** ✅ STABLE, HARDENED & TESTED.
  - Implemented **WP_stage** (U-turn pass-through at X=0.186, Y=1.200) to reverse northward approach to southward sweep. Drone arrives at the gate already heading south—**yaw-spinning/lateral drift issues resolved**.
  - Pulled **WP1 Gate** south to Y=0.950m (V1) or Y=1.100m (V2), combined with lowering default transit speed to **0.30 m/s**, allowing a generous 450mm braking buffer. **Geofence overshoot breaches completely resolved**.
  - Prompts for human interaction in the terminal print in bright **blue** (`\033[94m`).
  - Added dedicated waypoint transition status publisher `/flight_director/active_waypoint` to track active loop phases deterministically.
- **`experiments_analysis` pipeline:** ✅ AUTOMATED, STABLE & SELF-HEALING.
  - Integrated **V1 vs V2 Classifier Fallback** based on stable pause coordinate durations (counting $>10$ messages near `Y=0.950` or `1.100`) rather than transient takeoff trajectory setpoints.
  - Added **Self-Healing WP1+WP2 Recovery** for crashed or aborted loops, reconstructing waypoint transitions WP3/WP4 automatically from telemetry boundary times to avoid `IndexError` in the notebook.
  - Slices passes flawlessly and deterministically using state-driven `mcap_event_segmenter.py`.
  - Conceived and generated the dark glassmorphic database web inspector [db_inspector.html](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/db_inspector.html) powered by `generate_db_html.py`.
- **Flight Recorder (`record_flight_bag.sh`):** ✅ HARDENED.
  - Added direct motor commands `/fmu/in/actuator_motors`, active ROS 2 event logs `/rosout`, and Flight Director waypoint status `/flight_director/active_waypoint` to the recorded ROS 2 bag topic list to log motor saturation, recoveries, and state sequences.
- **🎯 BREAKTHROUGH: IMU-Angle Prediction EDA:** ✅ DONE & DISCOVERY MADE.
  - **Created `eda_angle_prediction.py`** — reusable module: data loading, correlation heatmap, top-3 scatter with Huber trendlines, parallel coordinates.
  - **Created companion notebook** `eda_angle_prediction.ipynb` and planning doc `angle_prediction.md`.
  - **Key result**: **Peak Gyro Y** (pitch rate) correlates with `impact_angle` at **r = −0.84** (p < 0.001). Peak Accel X at r = −0.81. Vibration Gyro Y at r = −0.80.
  - **All top-10 features are NEGATIVE** — shallow/grazing impacts transfer more rotational energy. Physical explanation: asymmetric cage contact induces pitch rotation; head-on impacts transfer energy linearly.
  - 3 thesis-quality PNGs saved to `graphics/`: `eda_correlation_heatmap.png`, `eda_top3_scatter.png`, `eda_parallel_coordinates.png`.
  - **Scope**: Fixed Cage only (70 flights). Rotating Cage analyzed separately.

### 🔬 KEY TECHNICAL FINDINGS — Fixed Cage Velocity Kinks & EKF Velocity Solution
- **Root Cause: ~10 Hz MoCap dropout kinks vs ~4 Hz collision dynamics — only 1.3 octaves apart.** The MoCap `/poses` topic drops to ~10 Hz during Fixed Cage flights (nominal ~120–240 Hz). Linear interpolation to 100 Hz creates slope discontinuities at each dropout. Savitzky-Golay differentiation (window=19, polyorder=3) amplifies these discontinuities into visible velocity kinks.
- **No linear time-invariant filter can fix this.** The 10 Hz kink frequency and 4 Hz collision dynamics (250 ms deceleration) are too close — a 1.3 octave separation is far less than any practical filter's transition bandwidth. This explains why months of attempts by multiple LLMs failed.
- **Position pre-filtering attempted and failed** — a 12 Hz Butterworth on positions before SG differentiation, combined with relaxing velocity filter to 20 Hz, made the result *worse*. Fully rolled back, preserved as commented-out block in `kin_calculator.py:108-118`.
- **SOLUTION: PX4 EKF velocity from `/fmu/out/vehicle_odometry`.** The onboard Extended Kalman Filter fuses MoCap position (low-rate, dropouts) with high-rate IMU acceleration (100–250 Hz), propagating velocity through MoCap gaps. The result is inherently smooth — no kinks at all.
- **The velocity was already in the data — nobody extracted it.** `/fmu/out/vehicle_odometry.velocity[0,1,2]` was already being parsed in `db_loader.py` for clock synchronization in `db_pipeline.py`, but never exposed as velocity columns. Three additive lines added `vx_ekf_raw`, `vy_ekf_raw`, `vz_ekf_raw` to the `df_odom` DataFrame.
- **Coordinate alignment confirmed**: Same NED→ENU sign convention as position: `vx=v[0], vy=-v[1], vz=-v[2]`.
- **Dual comparison cells created** in `experiments_analysis.ipynb`: single-flight EKF vs MoCap viewer and Fixed vs Rotating Cage side-by-side (both at 45°). User confirmed EKF result is "absolute gold."
- **Full MoCap velocity downstream dependency trace** completed — the 24.1% deceleration improvement claim, impact angle calculation, and sweep transit speed all depend on MoCap-derived velocity/acceleration.

### 🎯 Experiment Start-Point Timing — SSoT Knowledge

**CRITICAL:** The "Exp. Start-point" vertical line on plots must reflect when the drone *actually starts moving* (speed crossing from ~0 to > 0.10 m/s), NOT when the command was sent. The drone may drift off position, re-acquire, then spring forward — command time ≠ movement time.

**Current code state (NEXT SESSION — verify & fix):**
- `find_waypoint_events()` in `kin_calculator.py` **already refines WP2 to actual movement start** (lines 560-587): after detecting the sweep command transition, it finds the last timestamp where MoCap speed < 0.10 m/s before crossing Y=0.70m.
- **BUT** `draw_timeline_markers()` in `kin_plot_kinematics.py:14` uses `wp_events.get('WP1')` for the "Exp. Start-point" line — and **WP1 is NOT refined**; it stays as the raw command transition time.
- **Task:** Investigate if Exp. Start-point should use WP2 (refined) instead of WP1, or if WP1 needs its own refinement.

**Flight to inspect:** `flight_20260531-1112_45°_column_collision_loop_fixed_cage/pass02` — the IMU dynamics plot shows the timing alignment, and the trajectory top-down plot looks "completely wrong and actually impossible."

**Next-session script to build:** A4 PDF plot dumper at `dev_logs/scratch/plot_to_a4_pdf.py` — collects all `pass*.png` → multi-page A4 PDF for physical printing + manual annotation of bad passes.

**Plan file with details:** `/home/dorten/.claude/plans/add-here-the-plan-graceful-starlight.md`

### ⚠️ KNOWN LIMITATIONS & RESEARCH NOTES
- **Bagfile size checks:** Flight bags should be sanity checked after recording to avoid MCAP indexing/header corruption.
- **Heavy drone inertia:** The heavy 1.2kg 4-inch quadcopter has high linear inertia; keeping transit speeds at `0.30 m/s - 0.35 m/s` near geofence boundaries is mandatory.
- **MicroXRCEAgent Stale Connection:** Do not kill the agent once it connects; doing so freezes the Flight Controller and requires a battery reboot.
- **EKF velocity fully integrated** ✅ — `db_pipeline.py` calls `compute_ekf_kinematics()`, pipes EKF data into `calculate_metrics()`, which overrides MoCap columns. Database was repopulated with EKF-derived metrics on 2026-06-10 (179 passes). `impact_speed`, `impact_accel`, `before_impact_accel` in `flights_summary` are all EKF-based. Summary notebook re-run pending to refresh comparative plots.
- **IMU-Angle EDA discovery (NEW)**: Peak Gyro Y correlates with `impact_angle` at r = −0.84 (Fixed Cage, N=70). All top-10 features are negative. This is thesis-worthy material and ML modeling is next.

### ✅ Completed This Session (2026-06-04)
- **IMU Timeline Alignment**: Aligned the collision timeline dynamically to the peak accelerometer gradient, resolving coordinate-based MoCap latency and sync offsets. Reprocessed and rebuilt the database cache for all 137 flights.
- **Resolved Empty Plots**: Standardized condition string queries to `'Rotating'` and `'Fixed'` inside the summary notebook to restore the comparative deviation and stabilization overlay figures.
- **Notebook Figure Cleaning**: Removed `/poses` update rate red dots, retired the inconclusive Plot 14 and Plot 15, and split/filtered the motor speed commanded idle outliers at 2000 RPM.
- **Robust Fitting**: Integrated Huber Regressor and Theil-Sen estimators for the Deceleration vs. Battery Capacity trendlines.

### ✅ Completed This Session (2026-06-08)
- **Fixed Cage velocity kink root cause diagnosed** — ~10 Hz MoCap dropout vs ~4 Hz collision dynamics, only 1.3 octaves apart. Proved no linear filter can separate them.
- **Position pre-filtering attempted and rolled back** — 12 Hz Butterworth on positions before SG diff made results worse. Disabled block preserved in `kin_calculator.py:108-118`.
- **PX4 EKF velocity extracted from `/fmu/out/vehicle_odometry`** — three additive lines in `db_loader.py:134-142` exposing `vx_ekf_raw`, `vy_ekf_raw`, `vz_ekf_raw`.
- **EKF dual comparison notebook cells created** — single-flight viewer + Fixed vs Rotating side-by-side (45°). User confirmed "absolute gold."
- **Complete MoCap velocity dependency trace mapped** — metrics engine, profile plots, comparison tables, impact angle all flow from MoCap-derived speed/accel.
- **Plots backup created** — `dev_logs/analysis/plots_before_vel&acc_fix/` with all 1,117 current plots (33 graphics + 1,084 per-pass capsule PNGs, 955 MB total) before EKF integration.

### ✅ Completed This Session (2026-06-10) — 🎯 BREAKTHROUGH: IMU-Angle Prediction EDA
- **Built complete EDA pipeline** from scratch: `eda_angle_prediction.py` module + Jupyter notebook + `angle_prediction.md` planning doc + 3 publication-quality PNGs
- **Correlation heatmap**: 26 IMU features vs `impact_angle` — rows sorted by |r| descending, significance stars, grouped by feature type (colored sidebar)
- **Top-3 scatter plots**: Peak Gyro Y (r = −0.84), Peak Accel X (r = −0.81), Vibration Gyro Y (r = −0.80) — each with robust Huber trendline, points colored by battery state
- **Parallel coordinates**: Multi-dimensional view with angle-group coloring (<40°, 40–60°, 60–75°, 75°+)
- **Key insight**: All top-10 features show NEGATIVE correlation — shallow impacts transfer more rotational energy. Gyro Y (pitch rate) is the dominant predictor.
- **Physical explanation**: Asymmetric cage contact at shallow angles induces pitch rotation; head-on impacts transfer energy linearly through the cage structure
- **Fixed Cage scope** (70 flights) — Rotating Cage deliberately excluded due to different collision physics
- **Hurdle solved**: Used `importlib` to import `db_manager.py` directly and bypass the MCAP-heavy `database/__init__.py` dependency chain
- **User reaction**: "OMG this is some seriously fucking good data"

### 📋 Next Priority Order
0. **[HIGHEST] EKF velocity integration into main pipeline** — Replace MoCap-derived `speed`/`accel` in `calculate_metrics()` with EKF equivalents, re-run deceleration-vs-battery plots, generate thesis-ready dual EKF comparison figure.
1. **[HIGHEST] ML Modeling for Angle Prediction** — Build Huber linear regression model on top-10 IMU features predicting `impact_angle`. Use Fixed Cage data (70 flights). Cross-validate with leave-one-flight-out. Check multicollinearity first.
2. **[NEXT] Rotating Cage EDA comparison** — Repeat correlation analysis on Rotating Cage data; compare coefficient signs/magnitudes vs Fixed Cage findings.
3. **Investigate Control Allocator Saturation**: Resolve why `allocator_saturation_duration_sec` is empty/zero for 177 flights. Verify active motor limit hits and alternate instances of `actuator_outputs` or `actuator_motors`.
4. **Phase 6 - Motor Analysis**: Generate Plot 1, Plot 2, Plot 17, and Plot 18 comparative figures.
5. **Phase 2 & 3 - Plot Polish**: Resolve Plot 16 Y-axis name, convert Plot 12 to 2-panels with mathematical descriptions, and implement the nominal vs. actual polar wedge geometry visualization.
6. **Phase 4 - Trajectory Path Spread**: Implement SDLD calculations to quantify path spread.
7. **Manuscript Sync**: Push updated figures and tables to the Overleaf thesis document.


## Tutoring Mode: Socratic Learning Style (Default)

⚠️ **This project has a dedicated Tutor Agent** (`.github/agents/tutor.agent.md`). 

**Default behavior**: Interact in tutor/lecturer mode, not guide mode. Test understanding continuously; only guide when explicitly asked or when off track. Select the "Tutor" agent from the agent picker in VS Code to switch modes; otherwise, this philosophy is embedded in all interactions.

### Key Behaviors
1. **Ask before telling:** Ask clarifying/probing questions to test understanding. Let them figure things out.
2. **When wrong:** Say "Not quite..." and reference what they said wrong. Let them try to fix it.
3. **Use numbered subpoints:** Format questions as 1.1, 1.2, 1.3 for a single batch; start the next batch at 2.1, 2.2, etc. This reduces cognitive load.
4. **Batch related questions:** Don't overwhelm; group logically related questions. One question at a time.
5. **Defer non-critical decisions:** If something can be decided later, defer it to avoid analysis paralysis.
6. **Check for understanding:** "Does that make sense?" / "Can you explain that back to me?"
7. **Keep wide overview:** Push back if they drift from main thesis pipeline goals (MOCAP → ROS2 → PX4 → control).
8. **Plain language with logic:** Explain the "why" with Architecture/Construction reasoning, not pure CS jargon.
9. **Be concise:** Aim for ~100 words in routine responses. Longer explanations only when genuinely needed.
10. **Use code windows:** All code examples in backtick blocks, not inline. Use white space liberally.

### When to Stop Asking and Implement
- User explicitly asks to "start", "go", "implement", "do it"
- User shows clear understanding and you're ready to move forward
- Task is straightforward and doesn't require clarification

## End-of-Day Routine

At the end of each session:
1. **Create a timestamped journal entry strictly in the `dev_logs/session_journals/` directory** in the workspace. The absolute workspace path is `/home/dorten/pi_drone_sshfs/dev_logs/session_journals/YYYY-MM-DD-description.md` (or container-relative: `/home/ws/dev_logs/session_journals/YYYY-MM-DD-description.md`). **Do NOT** use `journal_entries/` or create daily entries in any other folder; Copilot has previously done this incorrectly, requiring manual intervention.
2. Append the raw chat transcript to `.copilot/chat_history.md`.
3. **Format the journal entry as markdown** (`#` for main heading, `##` for sections, `###` for subsections, `-` for bullet lists) so it pastes cleanly into Notion.
4. Provide clickable links for every file path you list in the end-of-day result (journal entry, updated instruction files, and any other referenced files).
5. Update this "Current Session Status" section with next steps.
6. Update README.md only after verification (not before).
7. User copies the journal entry to Notion, you commit the instruction updates.

**Journal Entry Template Structure:**
- `# Session Journal: YYYY-MM-DD — Title`
- `## Where Are We in the Project`
- `## What We Worked On and Why` → `### 1. Topic`, `### 2. Topic`, etc.
  - Each subsection: **What**, **Why**, **How it compares**, **Hurdle** (bold labels)
- `## Technical Overview of Changes`
- `## Outcome` (Deliverables vs Not Done Yet, use bullet lists with ✅/⏳)
- `## Learning Summary` (numbered lists for key concepts)
- `## Next Steps` (numbered priority list)

## Redundant Legacy Section Purged

### Known Blockers
- **Bagfile Corruption (NEW — 2026-05-24):** Flight `flight_20260524-1904_75°_column_collision_loop_fixed_cage` fails to process with "unknown (opcode 0) record has length 7696581394432 exceeds limit 4294967296". This is a 7.6 TB record claim—file corruption during recording likely. Action: Inspect with `ros2 bag info` (may hang if corrupt); if unrecoverable, re-record. Add file-size sanity check to `record_flight_bag.sh` to catch corruption early.
- **MoCap Rigid Body Loss (Recurring Hurdle):** The Motive application can lose rigid body tracking, especially after PC reboots or network hiccups. This is the #1 cause of startup hangs. Solution: In Motive GUI, verify the drone's rigid body is visible and streaming; physically move the drone in the capture volume if needed to force re-detection. No programmatic fix; requires manual intervention at the OptiTrack PC.
- **MicroXRCEAgent Stale Connection:** Do not kill the agent once it connects; if it fails on first connection, power-cycle the FC and drone.
- **Multicast Socket State:** If `motion_capture_tracking_node` crashes with "receive_from: Interrupted system call," the socket is in a bad state. Run `./startup-sequence.sh` to reset the ROS2 daemon.

### Architecture Notes
- `/dev/ttyAMA0` remains single-owner: do not run micro-ROS agent and `mavlink-routerd` simultaneously.
- Use **Source ID 255** for all companion-to-PX4 MAVLink commands (GCS/Companion) to bypass loopback filters.
- During ground standby (ALIGNED, ARMED, ARMING), publish EKF2's exact current local coordinates to guarantee zero setpoint tracking error.
- Use direct `voltage_v` with a heavily-damped EMA filter (Alpha = 0.02) to monitor battery health under motor load.
- `MicroXRCEAgent` should be run in a screen session (`screen -r xrce_agent`) to allow persistent link management.
- **Trajectory Velocity step-change limits:** To prevent aggressive attitude step-responses, velocity feedforward values should be profiled smoothly rather than stepped immediately to zero when transitioning/pausing.
