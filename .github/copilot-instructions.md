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
- **Coordinate Frames**: PX4 strictly requires NED (+X physical front, +Z physical down). The Motive Rigid Body pivot must be manually aligned so its internal X-axis points out the physical nose of the drone.
- **Visualization:** Foxglove bridge runs on Pi port 8765 for real-time browser viewing on laptop.

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
1. Create a timestamped journal entry in `/home/ws/dev_logs/journal_entries/YYYY-MM-DD-description.md` (This ensures it is saved in the repository, tracked by Git, and accessible to any AI).
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

### Current Session Status (Last Update: 2026-05-18)

### What Was Completed
- **Autonomous Takeoff & Hover Flight:** Successfully executed a full offboard hover takeoff and landing using MoCap ENU coordinates mapped to PX4 EKF2 frames!
- **10-Sample Rolling Average Alignment:** Eliminated EKF2 startup noise/drift by collecting a 10-sample rolling average on initialization to lock in a noise-free transform offset.
- **Zero-Error Ground Standby Setpoints:** Added EKF2 position streaming during ground standby states to guarantee 100% Offboard mode and Arming acceptance.
- **Direct Filtered Voltage Failsafes:** Implemented a highly-damped raw `voltage_v` low-pass filter (Alpha = 0.02 EMA) to absorb heavy takeoff voltage sags while securely landing at a true chemical threshold of 20.4V.
- **Universal Keyboard Control Engine:** Developed custom, modular deliberate keybindings (`[ A ]`, `[ T ]`, `[ L ]`, `[SPACE]`) running in a dedicated thread.
- **Emergency Kill MAVLink ID Fix:** Changed `emergency_kill.py` to use `source_system = 255`, ensuring immediate MAVLink forced-motor-cut delivery.
- **Offloaded Flight Recorder:** Offloaded MCAP bag recording to [flight_recorder.py](file:///home/dorten/pi_drone_sshfs/drone_control/flight_recorder.py) with automated post-landing recording and 3.0s delayed impact capture for motor kills.

### Next Steps (Priority Order)
1. **Rectangle Flight Mission:** Implement a `RectangleFlight` mission (`missions/rectangle_flight.py`) to verify multi-point trajectory tracking, horizontal translations, and yaw alignment.
2. **Repository Clean Up:** Organize the codebase, archive draft/old files, and clean up temporary logs/bags.
3. **Post-Flight Data Review:** Analyze the recorded MCAP bags and PX4 ULogs from today's successful hover to verify absolute vertical holding stability.

### Known Blockers
- None! Autopilot links, battery failsafes, alignment transforms, and emergency kill overrides are fully stable and verified.

### Architecture Notes
- `/dev/ttyAMA0` remains single-owner: do not run micro-ROS agent and `mavlink-routerd` simultaneously.
- Use **Source ID 255** for all companion-to-PX4 MAVLink commands (GCS/Companion) to bypass loopback filters.
- During ground standby (ALIGNED, ARMED, ARMING), publish EKF2's exact current local coordinates to guarantee zero setpoint tracking error.
- Use direct `voltage_v` with a heavily-damped EMA filter (Alpha = 0.02) to monitor battery health under motor load.
- `MicroXRCEAgent` should be run in a screen session (`screen -r xrce_agent`) to allow persistent link management.
