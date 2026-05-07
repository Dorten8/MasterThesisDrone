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
- `mocap_px4_bridge/launch/run.launch.py` starts both `mocap` (OptiTrack launch) and the bridge node; topic wiring is parameterized in `mocap_px4_bridge/config/params.yaml`.
- MAVLink control/telemetry is handled separately through `mavlink-routerd` using `config/mavlink-router/main.conf`:
  - UART endpoint: `/dev/ttyAMA0` at `921600`
  - UDP endpoint: `192.168.74.9:14550`
- `drone_controller/` contains Python joystick/manual-control tooling (`pymavlink`, `pyserial`) for direct MAVLink command flow.

## User Profile

**Developer:** Dorten
- **Terminal Shell:** Fish 🐟 (not Bash)
- **Dev Machine:** Ubuntu 22.04 on Legion laptop
- **When providing commands:** Always use Fish syntax (`set -x` for env vars, `fish` script headers)
- **Pi hostname:** `dorten-pi5drone` (mDNS: `dorten-pi5drone.local`)
- **Pi network:** 192.168.74.x range (verify with `hostname -I` on Pi)

## Key repository conventions

- Read `README.md` intro first before making architectural or workflow changes.
- Treat this repository root as the ROS workspace root (`/home/ws`), not just a source checkout.
- Keep/expect these ROS env defaults in container sessions:
  - `ROS_DOMAIN_ID=0`
  - `ROS_LOCALHOST_ONLY=0`
  - `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- Serial link convention across components is `/dev/ttyAMA0` with baud `921600`.
- `/dev/ttyAMA0` is single-owner at runtime: do not run `micro_ros_agent` and `mavlink-routerd` against it simultaneously.
- For PX4↔ROS 2 bridge work, use `src/micro-ROS-Agent` (ROS package). Do not use `src/Micro-XRCE-DDS-Agent` in colcon workflows.
- Motion-capture topic convention is driven by `mocap_px4_bridge/config/params.yaml`; ensure `mocap_topic` matches the OptiTrack rigid body topic name, and keep PX4 output topic aligned with `/fmu/in/vehicle_visual_odometry` unless intentionally changed.
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
1. Create a timestamped journal entry in `/home/dorten/.copilot/session-state/19b4e3cc-e8aa-4925-be73-ff4691152ab3/journal_entries/YYYY-MM-DD-description.md`
2. **Format as markdown** (`#` for main heading, `##` for sections, `###` for subsections, `-` for bullet lists) so it pastes cleanly into Notion
3. Provide clickable links for every file path you list in the end-of-day result (journal entry, updated instruction files, and any other referenced files)
4. Update this "Current Session Status" section with next steps
5. Update README.md only after verification (not before)
6. User copies journal entry to Notion, you commit the instruction updates

**Journal Entry Template Structure:**
- `# Session Journal: YYYY-MM-DD — Title`
- `## Where Are We in the Project`
- `## What We Worked On and Why` → `### 1. Topic`, `### 2. Topic`, etc.
  - Each subsection: **What**, **Why**, **How it compares**, **Hurdle** (bold labels)
- `## Technical Overview of Changes`
- `## Outcome` (Deliverables vs Not Done Yet, use bullet lists with ✅/⏳)
- `## Learning Summary` (numbered lists for key concepts)
- `## Next Steps` (numbered priority list)

## Current Session Status (Last Update: 2026-05-07)

### What Was Completed
- Built complete RViz visualization pipeline: `drone_odometry.rviz` config + `launch_rviz_laptop.sh` for real-time pose monitoring on laptop.
- Created flight telemetry recording script (`record_flight_bag.sh`) to capture mocap→PX4 chain topics for post-flight analysis.
- Created multi-process startup helper (`startup-sequence.sh`) for reproducible commissioning.
- Updated copilot-instructions.md with User Profile (Dorten) and Visualization section (Foxglove on port 8765).
- Consolidated networking and ROS 2 workflow docs in `dev_notes.md`.

### Next Steps (Priority Order)
1. **Verify RViz on laptop with Pi network** — Connect to Pi ROS 2 topics via SSHFS mount; confirm 30 Hz real-time updates in drone_odometry.rviz.
2. **Confirm frame semantics (NED/ENU)** — Test with known reference pose; verify PX4 accepts MoCap visual odometry in expected frame.
3. **Extend offboard_control.py** — Add waypoint loop or hover setpoint; validate command flow end-to-end with logged acknowledgments.
4. **Test flight recording & replay** — Short manual flight → record bag → replay in RViz with mocap input vs. PX4 estimates side-by-side.

### Known Blockers
- Frame-semantics design decision still pending (NED vs. ENU transform responsibility).
- RViz integration test deferred pending next session (requires Pi network access).

### Architecture Notes
- `/dev/ttyAMA0` remains single-owner: do not run micro-ROS agent and `mavlink-routerd` simultaneously.
- Visualization stack: RViz on laptop (port 11311 for DDS) + Foxglove on Pi (port 8765 for browser) + flight bag replay for offline analysis.

