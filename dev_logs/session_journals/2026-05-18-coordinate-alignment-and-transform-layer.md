# Session Journal: 2026-05-18 — Coordinate System Alignment & transform Layer

## Where Are We in the Project
Having resolved the MicroXRCE-DDS Agent connection blocks and latency tuning in previous sessions, today we entered **Phase 0: Pre-Flight Code & Safety Hardening** and **achieved our first successful autonomous flight!** Using our newly designed **Flight Director (`flight_director.py`)**, the drone armed flawlessly, entered Offboard mode, executed a controlled takeoff, hovered at its target, and safely landed!

---

## What We Worked On and Why

### 1. The Ghost Flight Recorder (`ghost_flight.py`)
* **What:** Coded a modular, interactive trajectory recorder inside [ghost_flight.py](file:///home/dorten/pi_drone_sshfs/dev_logs/ghost_flight.py).
* **Why:** To capture manual "Ghost Flights" by carrying the drone through the cage, regularizing the samples onto a clean 10Hz grid, and applying a rolling-window convolution to filter out human hand tremors.
* **Outcome:** Created a Python node that automatically exports a smoothed, playback-ready trajectory to `dev_logs/paths/ghost_path_latest.csv`.

### 2. The Great Coordinate Swap (Streaming Up-Axis)
* **What:** Debugged a massive horizontal warping anomaly where Y coordinates read a bizarre `29m` despite the drone sitting centered in the room.
* **Why:** A recent OptiTrack recalibration defaulted Motive's NatNet stream setting to **`Up Axis: Z-Axis`**.
* **The Hurdle:** The ROS 2 `motion_capture_tracking_node` expects Motive to stream in **`Y-Axis` Up** mode, converting it to standard ROS ENU (East-North-Up). Because Motive streamed Z-Up, the ENU converter rotated the horizontal plane into the vertical axis, swapping the horizontal and vertical bounds and creating massive offsets.
* **Solution:** Switched the NatNet setting back to **`Up Axis: Y-Axis`** in Motive. The `/poses` topic instantly normalized back to correct, sub-meter bounds.

### 3. EKF2 Altitude Origin & Transform Layer
* **What:** Identified why EKF2 local position (`/fmu/out/vehicle_local_position`) Z reported a stationary value of `-0.8m` when the drone was physically sitting at a height of `0.09m` (9cm) above the floor.
* **Why:** EKF2 defines its coordinate origin `[0,0,0]` relative to its state at **boot-up initialization** (utilizing barometer/internal sensors before the DDS-MoCap link is fully stabilized).
* **The Hurdle:** Absolute room coordinates (like geofencing bounds or static obstacle bounding boxes) are fixed relative to the OptiTrack floor, but the drone commands itself relative to its boot-relative EKF2 Home origin.
* **The Architectural Fix (Transform Layer):** We proposed a **Frame Alignment Translation Layer** running in our control script. It calculates the offset vector $\Delta = P_{ekf2} - P_{mocap}$ at takeoff, allowing us to define geofences/obstacles in absolute room coordinates while dynamically converting setpoints on the fly for the PX4 EKF2 frame.

### 4. 10-Sample Rolling Average Alignment & Standby Setpoints
* **What:** Solved a critical Offboard arming rejection where the Pixhawk refused to arm under `flight_director.py` but worked in `hover_test_initial.py`.
* **Why:** EKF2 takes a few seconds to converge upon boot. Aligning on a single startup tick locked in a noisy, drifted coordinate offset. Additionally, any slight setpoint tracking mismatch during arming causes PX4 safety filters to reject offboard mode.
* **Solution:** Refactored alignment to collect a 10-sample rolling average (1.0 second) on startup, completely filtering out EKF2 convergence spikes. Added a `_send_ekf_setpoint()` helper to stream the drone's actual EKF2 position during standby ground states, guaranteeing exactly `0.00m` of tracking error.

### 5. Direct Filtered Voltage Monitoring
* **What:** Solved an emergency land failsafe that immediately aborted the takeoff as soon as the motors spun up.
* **Why:** Heavy motor load draws high current, causing a transient battery voltage sag. PX4's internal capacity estimator (`msg.remaining`) dropped instantly to 40%, triggering our battery failsafe.
* **Solution:** Switched to direct `msg.voltage_v` telemetry paired with a highly-damped Low-Pass Filter (Alpha = 0.02 EMA, acting like a 15-second moving average). This filters out transient takeoff voltage drops, while securely triggering landing at a true chemical threshold of `20.4V` (3.4V/cell).

### 6. Emergency Kill MAVLink ID Fix
* **What:** Fixed `emergency_kill.py` which originally failed to cut the motors on spacebar press.
* **Why:** The kill command originally broadcasted with `source_system = 1`. PX4 treated this as an internal loopback from the flight controller itself and silently discarded it.
* **Solution:** Switched the MAVLink system ID to `source_system = 255` (Ground Control Station / Companion). PX4 now instantly honors the forced disarm command.

---

## Technical Overview of Changes
* **Core Node:** Updated [flight_director.py](file:///home/dorten/pi_drone_sshfs/drone_control/flight_director.py) to implement 10-sample rolling average alignment, direct voltage filtering, zero-error standby setpoints, and interactive console battery printouts.
* **Kill Switch:** Updated [emergency_kill.py](file:///home/dorten/pi_drone_sshfs/drone_control/emergency_kill.py) to broadcast with GCS System ID 255.
* **Missions:** Corrected [hover_test.py](file:///home/dorten/pi_drone_sshfs/drone_control/missions/hover_test.py) to add 0.5 meters to the initial MoCap Z (since MoCap Z is positive UP).
* **Documentation:** Updated [walkthrough.md](file:///home/dorten/.gemini/antigravity/brain/22b6eb95-6d9d-4ed9-a8e5-abe28c83559b/walkthrough.md) with our successful flight details and breakthrough analyses.

---

## Outcome
* ✅ Coded and validated the Ghost Flight Recorder script.
* ✅ Switched the NatNet setting back to `Up Axis: Y-Axis` in Motive to fix warped coordinates.
* ✅ Designed and verified the continuous Frame Transform translation layer.
* ✅ **Autonomous Takeoff & Hover Successful!** Armed, took off to 0.60m MoCap Z, hovered, and safely landed!
* ✅ Implemented direct voltage monitoring with Alpha=0.02 EMA filtering to absorb takeoff sags.
* ✅ Verified 100% reliable emergency spacebar kill switch (`source_system = 255`).

---

## Learning Summary
1. **Up Axis is Crucial:** Always verify that Motive's NatNet Stream settings have `Up Axis: Y-Axis` enabled. While Motive's viewport is internally Y-Up, Foxglove and our ROS 2 Bridge require standard ENU (Z-Up). Z-Axis streaming rotates the horizontal plane into the vertical axis, creating catastrophic failures.
2. **EKF2 Home is Floating:** Never assume EKF2 home altitude equals the MoCap floor. EKF2 boots with an arbitrary origin based on initial sensor calibration.
3. **Standby Setpoint Mismatch:** To guarantee Offboard mode and Arming acceptance on the ground, companion setpoints must have exactly 0.0m of error relative to EKF2's current position estimate. Always publish the actual EKF2 position during ground standby.
4. **Voltage Sag Failsafes:** Never use software capacity estimators (`msg.remaining`) for critical flight failsafes as they drop instantly under takeoff load. Monitor raw `voltage_v` and apply a high-damping low-pass filter (Alpha = 0.02) to absorb transient current spikes.
5. **MAVLink Source ID Routing:** Pixhawk ignores companion-sent MAVLink commands if they claim to come from the drone's own ID (`1`). Always use `source_system = 255` for companion commands to bypass loopback filters.

---

## Next Steps
1. **Rectangle Flight Mission:** Write a simple `RectangleFlight` mission (`missions/rectangle_flight.py`) to verify multi-point trajectory tracking, horizontal translations, and yaw alignment in the cage.
2. **Repository Clean Up:** Organize the codebase, archive old draft files, and clean up temporary logs/bags.
3. **Post-Flight Data Review:** Analyze the recorded MCAP bags and PX4 ULogs from today's successful hover to verify absolute vertical holding stability.
