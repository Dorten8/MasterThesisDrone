## 2026-05-08 Chat Session
**User**: Hey, can u tell me what is this all about and how you are supposed to initilize stuff?
**AI**: Summarized project (ROS 2 Humble, Pi 5, Pixhawk 6C, OptiTrack) and how to initialize (Dev Container, colcon build, MicroXRCEAgent, mocap nodes).
**User**: so what is for you as an AI the SSoT?
**AI**: The `README.md` file located in the root of this repository.
**User**: fuck, that is wronf...I mean u have correct answer..but it was supposed to be changed by Gemini Code Assist and it wasnt please eppend there this /home/dorten/pi_drone_sshfs/.github/copilot-instructions.md this file in this repo is the most important SSoT for AI initialiaztion
**AI**: Updated GEMINI.md to point to `.github/copilot-instructions.md`. Confirmed reading it.
**User**: Great stuff, and u read it too right? so please run end of the day routine
**AI**: Running End of Day Routine.

## 2026-05-14 Chat Session
**User**: Reported issues with Wi-Fi power management causing connectivity drops on the Pi unless a monitor is attached. Provided instructions for installing `iw` and permanently disabling power save in NetworkManager.
**AI**: Created journal entry `2026-05-14-network-stability-fix.md` and updated session status to record the permanent fix (`wifi.powersave = 2`).

## Session: 2026-05-14 (Mocap Stabilization)
- Resolved QoS mismatch for position messages.
- Discovered "Source ID 255" requirement for arming.
- Measured 60ms EKF2_EV_DELAY via Jerk Test.
- Implemented NaN setpoint safety for trajectory generation.
- Created screen-based XRCE agent protocol.
- Full technical breakdown in: [dev_logs/journal_entries/2026-05-14-mocap-pipeline-stabilization.md](file:///home/dorten/pi_drone_sshfs/dev_logs/journal_entries/2026-05-14-mocap-pipeline-stabilization.md)

## Session: 2026-05-19 (Pass-By Column Coordination)
- Coded and validated the absolute-coordinate transit and pass-by column mission trajectory.
- Diagnosed and fixed the 1m+ flight drift bug by restoring EKF2-to-Motive translation offset inside `flight_director.py`.
- Tailored the trajectory endpoints to align perfectly with the Motive grid axes.
- Full technical breakdown in: [dev_logs/session_journals/2026-05-19-mocap-geofence-and-rectangle-flight.md](file:///home/dorten/pi_drone_sshfs/dev_logs/session_journals/2026-05-19-mocap-geofence-and-rectangle-flight.md)

## Session: 2026-05-22 (Velocity Feedforward & Flight Hardening)
- Enabled offboard velocity feedforward in the heartbeat flags to completely fix jerky speed tracking and deceleration overshoots.
- Shifted sweep loop waypoints slightly Southward (1.200m Y-limit) to maintain a safe physical clearance margin (12cm) for the custom PETG-printed glass-fiber cage.
- Discovered and diagnosed the loopback deceleration hookup (WP4 to WP1) due to instantaneous velocity feedforward step-changes.
- Measured battery consumption under active motor load (20% capacity depletion per minute of flight, estimated max flight time of 3 minutes).
- Full technical breakdown in: [dev_logs/session_journals/2026-05-22-flight-hardening-and-loopback-optimization.md](file:///home/dorten/pi_drone_sshfs/dev_logs/session_journals/2026-05-22-flight-hardening-and-loopback-optimization.md)

## Session: 2026-05-23 (MoCap Recovery & Universal Modular Analysis Pipeline)
- Resolved startup hangs and socket multicast timeouts caused by OptiTrack PC power outage reboots.
- Replaced hardcoded IP references in `startup-sequence.sh` with robust, config-driven JSON queries from `drone_config.json`.
- Refactored `experiments_analysis.ipynb` into a clean parameter-driven thin-notebook backed by the modular `experiments_analysis` package.
- Restored strict 1:1 aspect ratios, equal 0.5m grid ticks, and scaled vector CAD drone viewport overlays inside 2D top-down trajectory plots.
- Integrated a self-healing Jupyter module reload guard to clear memory cache dynamically and successfully verified headless notebook execution and vector TikZ compilation (exit code 0).
- Configured, renamed, and mathematically validated the `exp_collision_75deg.py` mission. Forced a safe `1.0 m/s` rapid transit/approach and symmetrized coordinates to trace a strictly vertical straight-line sweep at $X = -0.186$ m, matching CAD and MoCap `/poses` column offset offsets.
## Session: 2026-05-24 (Experimental Analysis Pipeline & Symmetrical Collision Loop)
- Conceived and engineered the core mathematical and statistical data pipeline (`experiments_analysis` package) to clean raw high-frequency telemetry using Savitzky-Golay filtering, isolate active sweep events, calculate perpendicular tracking errors and closest approach clearances, and compile 2x2 comparative plots.
- Symmetrized the waypoint logic in `exp_collision_75deg.py` to fly a straight-line vertical sweep at a constant offset of $X = -0.186$ m, matching CAD sketch and `/poses` column offset.
- Replaced all hardcoded IP references in `startup-sequence.sh` with configuration-driven queries from `drone_config.json`, fixed the multicast node tracking join timeout pattern, and introduced custom health diagnostics for Companion-to-FC connectivity checking.
- Enabled `hb.velocity = True` inside [flight_director.py](file:///home/dorten/pi_drone_sshfs/drone_control/flight_director.py) to activate PX4 offboard velocity feedforward.
- Full technical breakdown in: [dev_logs/session_journals/2026-05-24-experimental-analysis-pipeline-and-collision-loop.md](file:///home/dorten/pi_drone_sshfs/dev_logs/session_journals/2026-05-24-experimental-analysis-pipeline-and-collision-loop.md)

## Session: 2026-05-25 (Corrupted MCAP Auto-Repair & DB Population)
- Designed and built a robust low-level MCAP auto-repair engine utilizing `StreamReader(record_size_limit=None)` to recover **150,905 raw messages** from the corrupted `flight_20260525-1706` bag.
- Implemented dynamic ROS 2 message schema registration using `writer.register_msgdef()` during pass slicing to eliminate deserialization `KeyError` crashes in standard readers.
- Added a direct `__main__` entry launcher block to `exa_pipeline.py` enabling fully automated, terminal-based pipeline execution and database population with duplicate-skipping support.
- Fully cached 9 verified sweep passes in `collision_experiments.db`, successfully updating metrics for battery capacity, sweep speed, impact speed, acceleration, and perpendicular rebound deviations.
- Full technical breakdown in: [dev_logs/session_journals/2026-05-25-telemetry-pipeline-upgrades-and-mcap-repair.md](file:///home/dorten/pi_drone_sshfs/dev_logs/session_journals/2026-05-25-telemetry-pipeline-upgrades-and-mcap-repair.md)

## Session: 2026-05-27 (Waypoint Event Slicing & Database Web Inspector)
- Implemented `/flight_director/active_waypoint` event publisher inside the Flight Director to log control state machine events deterministically.
- Updated `record_flight_bag.sh` to record the new event topic.
- Created `mcap_event_segmenter.py` using state-machine-driven parser logic to cleanly slice multi-loop bags into individual micro-bags.
- Integrated SQLite database updates tracking cage physical conditions (`condition`) and actual `collision` outcome indicators.
- Created a standalone HTML SQLite web inspector `db_inspector.html` in dark glassmorphism.
- Full technical breakdown in: [dev_logs/session_journals/2026-05-27-mcap-event-slicing-and-database-hardening.md](file:///home/dorten/pi_drone_sshfs/dev_logs/session_journals/2026-05-27-mcap-event-slicing-and-database-hardening.md)

## Session: 2026-05-28 (Robust Kinematics & Self-Healing Pipeline)
- Resolved Jupyter notebook `IndexError` by fixing trajectory setpoint classification and waypoint extraction edge-cases.
- Hardened V1 vs V2 mission detection (`detect_mission_class`) using stable pause-duration checks (counting coordinate matches near `Y=0.950` vs `1.100` for $>10$ messages) instead of transient interpolation.
- Added self-healing waypoint pass reconstruction inside `exa_kinematics.py` for aborted/crashed flights where WP1 (Stage) and WP2 (Sweep Entry) were reached, estimating WP3/WP4 from telemetry boundaries.
- Verified end-to-end notebook execution via headless `nbconvert` and successfully restored clean unstaged repository state.
- Full technical breakdown in: [dev_logs/session_journals/2026-05-28-robust-kinematics-and-self-healing-pipeline.md](file:///home/dorten/pi_drone_sshfs/dev_logs/session_journals/2026-05-28-robust-kinematics-and-self-healing-pipeline.md)



