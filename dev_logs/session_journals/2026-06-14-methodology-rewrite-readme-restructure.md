# Session Journal: 2026-06-14 — Methodology Rewrite & README Restructure

## What We Worked On

### 1. Methodology.tex: Replaced Setup Workflow with System Architecture

Sections 5 (Autonomous Drone Companion Computer Setup Workflow) through 6.9 (Troubleshooting Reference) — ~315 lines of verbose step-by-step instructions — were replaced with a concise **System Architecture** section organised into five layers:

- **Hardware Platform** — Pi 5 as companion computer
- **Flight Controller** — Pixhawk 6C + PX4 v1.16.1rc, magnetometer disabled
- **Motion Capture Integration** — OptiTrack → `motion_capture_tracking` → `mocap_px4_bridge`
- **Communication Middleware** — uXRCE-DDS exclusively during experiments; MAVLink for debug only (not simultaneous)
- **Software Framework** — ROS 2 nodes: `flight_director.py`, `ghost_flight.py`, `flight_recorder.py`, `px4_ros_com`/`px4_msgs`. With a `\todo{}` note to replace with a dedicated node-graph diagram.

Two LLM NOTES added:
- `\todo{UPDATE THE SYSTEM ARCHITECTURE DIAGRAM to reflect this 5-layer layout}`
- `\todo{REPLACE THE SOFTWARE FRAMEWORK PARAGRAPH BELOW WITH A DEDICATED DIAGRAM SHOWING THE ROS 2 NODE GRAPH AND DATA FLOWS}`

The "Data Processing for Collision Analysis" subsection was kept as-is (already solid per the existing LLM NOTE).

### 2. README.md: Full Restructure

Replaced the messy draft README with a structured reproduction guide:
- Hardware BOM table with design rationale
- Pi 5 host setup (condensed from old Methodology)
- Container setup (devcontainer)
- PX4 integration (wiring table + parameter table)
- **🚨 Critical Gotchas** — 8 hard-won lessons from session journals (EKF2 magnetometer, velocity feedforward flag, cage radius geofence, voltage sag, NatNet up-axis, 90° cross-coupling, QoS durability, EV_DELAY calibration)
- Experiment pipeline + collision sweep loop coordinates
- Data analysis pipeline summary
- Troubleshooting table

### 3. Key Decisions

- PX4 parameter config moved entirely to README (not in thesis text — user requested this)
- Comms clarified: uXRCE-DDS OR MAVLink, not both simultaneously
- Nodes corrected to match the actual `experiments_pipeline.pdf` diagram (flight_director, ghost_flight, flight_recorder, px4_ros_com, px4_msgs)

## Files Touched

- `thesis/Sections/Methodology.tex` — Major rewrite: replaced sections 5–6.9 with new System Architecture (5-layer breakdown)
- `README.md` — Full restructure: reproduction guide with hardware BOM, setup steps, critical gotchas, pipeline docs
