# Copilot instructions for `MasterThesisDrone`

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
* **Thesis Manuscript (Overleaf):** https://overleaf.com/read/ptfbgvrbbkpq#ba9385
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

## Current Session Status (Last Update: 2026-05-24 19:30 UTC)

### 🎯 Mission Status
- **`ExpCollision75Deg` mission:** ✅ STABLE & READY. Symmetrized waypoint logic, hardcoded safety speeds (1.0 m/s transit, variable sweep). Geometry is locked at X=–0.186m offset (CAD-verified). Autolooping enabled. Expect to gather first 75° dataset tomorrow.
- **`experiments_analysis` pipeline:** ✅ FUNCTIONAL. Data ingestion → Savitzky-Golay filtering → event isolation → perpendicular error + closest-approach metrics → side-by-side box plots. Ready for flight processing.
- **Startup & diagnostics:** ✅ HARDENED. Configuration-driven (drone_config.json SSoT), multicast socket glitch fixed, health checks automated.
- **Velocity feedforward:** ✅ ENABLED. Smooth trajectory profiles active; no more jerky position hunting.

### ⚠️ CRITICAL BLOCKER FOR TOMORROW
**Bagfile Corruption:** Flight `flight_20260524-1904_75°_column_collision_loop_fixed_cage` reports:
```
[ERROR] Failed processing flight: unknown (opcode 0) record has length 7696581394432 that exceeds limit 4294967296
```
This is a **7.6 TB record claim**—almost certainly a file corruption during recording or a malformed ROS 2 bagfile header. 
**Action for tomorrow:**
1. Check bagfile integrity: `ros2 bag info /path/to/flight_20260524-1904_75°_column_collision_loop_fixed_cage` (may hang if corrupt)
2. If stuck, kill the process; bagfile is likely unrecoverable
3. Re-record the flight (it was "pretty good" per user, so worth repeating)
4. Add file-size sanity check to `record_flight_bag.sh` to catch corruption early in future runs

### ✅ Completed This Session (2026-05-24)
- Engineered `experiments_analysis` package (6 modules + Jupyter notebook)
- Implemented symmetrical 75° collision loop with safety hardening
- Configuration-driven startup pipeline (IP/multicast dynamic binding)
- Fixed trajectory visualization (equal aspect ratio, 0.5m grid, drone SVG icons)
- Enabled velocity feedforward for smooth offboard control
- Created comprehensive end-of-day journal documenting all changes

### 📋 Tomorrow's Priority Order
1. **Verify startup end-to-end:** `./startup-sequence.sh` → check mocap streaming in Motive GUI
2. **Diagnose bagfile corruption:** Determine if `flight_20260524-1904_*` is recoverable
3. **Execute fresh 75° collision run:** Record new clean bagfile to `/dev_logs/`
4. **Process through pipeline:** Load into `experiments_analysis.ipynb`, verify tracking metrics (expect 0.1–0.3 m error)
5. **Build first comparative plot:** If time permits, run 3× with-cage / 3× without-cage at 75°
6. **Iterate angle series:** Begin 60°, 45°, 30°, 0° variants (if 75° yields good data)

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
2. Append the raw chat transcript to `/home/ws/dev_logs/chat_history.md`.
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

### Current Session Status (Last Update: 2026-05-23)

### What Was Completed
- **Core Experimental Analysis Pipeline (20-30% Thesis Milestone):** Conceived and engineered the core mathematical and statistical data pipeline (`experiments_analysis` package) to automatically clean raw high-frequency telemetry using Savitzky-Golay filtering, isolate active sweep events, calculate perpendicular tracking errors and closest approach clearances, and compile 2x2 side-by-side comparative boxplots (With vs. Without Cage) and multi-angle progression trends.
- **Symmetrical 75° Collision Loop Optimization:** Refactored `exp_collision_75deg.py` into a robust `ExpCollision75Deg` loop class. Symmetrized the waypoint logic to fly a straight-line vertical sweep at a constant offset of $X = -0.186$ m, matching your CAD sketch and `/poses` column offset. Hardcoded explicit `1.0 m/s` transit/recovery speeds to protect the drone, keeping `sweep_speed` strictly limited to the active `WP2 -> WP3` segment.
- **MoCap Recovery & Diagnostics:** Debugged startup failures after the OptiTrack PC power outage reboot. Replaced all hardcoded IP references in `startup-sequence.sh` with configuration-driven queries from `config/drone_config.json`, fixed the multicast node tracking join timeout pattern, and introduced custom health diagnostics for Companion-to-FC connectivity checking.
- **Physical Safety Aspect Ratio & Grid Restoration:** Fixed the spatial trajectory plot in `exa_plot_trajectory.py` to enforce a strictly equal aspect ratio and 0.5m grid normalization squares, restoring the CAD drone top vector illustrations at WP2, WP3, and closest approach.
- **Offboard Heartbeat Velocity Fix:** Enabled `hb.velocity = True` inside [flight_director.py](file:///home/dorten/pi_drone_sshfs/drone_control/flight_director.py) to activate PX4 offboard velocity feedforward. Resolves jerky speed profiles and position hunting.

### Next Steps (Priority Order — Tomorrow 2026-05-25)
1. **Verify Startup End-to-End:** Run `./startup-sequence.sh` and ensure MoCap rigid body tracking is actively streaming in Motive GUI (expect ~980 B/frame rate, <1 mm residual).
2. **Diagnose Bagfile Corruption:** Investigate `flight_20260524-1904_75°_column_collision_loop_fixed_cage` — check if recoverable with `ros2 bag info`. If corrupt, re-record the flight (it was good data worth repeating).
3. **Execute Fresh 75° Collision Run:** Use `ExpCollision75Deg` mission to record clean bagfile to `/dev_logs/` (expect 2–3 loops per flight).
4. **Process Through Pipeline:** Load bagfile into `experiments_analysis.ipynb`, verify Savitzky-Golay filtering is working, check perpendicular error metrics (expect 0.1–0.3 m).
5. **Build First Comparative Plot:** If single 75° run validates well, execute 2–3 more runs (with & without cage) and generate box plots.
6. **Iterate Angle Series:** Begin 60°, 45°, 30°, 0° collision angles (if 75° yields consistent data).
7. **Implement Deceleration Ramping (Future):** Smooth velocity step-changes when transitioning between waypoints.
8. **Add Automatic Battery Failsafe (Future):** Voltage-based auto-Land when capacity < 40%.

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
