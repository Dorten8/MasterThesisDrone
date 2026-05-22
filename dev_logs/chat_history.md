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
- Shifted sweep loop waypoints slightly Southward (1.200m Y-limit) to maintain a safe physical clearance margin (12cm) for the custom carbon cage.
- Discovered and diagnosed the loopback deceleration hookup (WP4 to WP1) due to instantaneous velocity feedforward step-changes.
- Measured battery consumption under active motor load (20% capacity depletion per minute of flight, estimated max flight time of 3 minutes).
- Full technical breakdown in: [dev_logs/session_journals/2026-05-22-flight-hardening-and-loopback-optimization.md](file:///home/dorten/pi_drone_sshfs/dev_logs/session_journals/2026-05-22-flight-hardening-and-loopback-optimization.md)
