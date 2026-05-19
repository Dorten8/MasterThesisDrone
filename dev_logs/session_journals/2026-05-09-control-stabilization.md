# Session Journal: 2026-05-09 — Drone Control Stabilization & Command Validation

## Where Are We in the Project
We are finalizing the foundational control link and MoCap coordinate mapping. Before today, the drone suffered from startup hangs, serial freezes, and unverified data links. We have now stabilized the ROS 2 ↔ PX4 pipeline, mapped the rigid body coordinate frames, and successfully proven 2-way command communication (meaning the software killswitch is ready).

## What We Worked On and Why

### 1. Startup/Shutdown Sequence Refactor
**What**: Modified the Bash startup and shutdown scripts to use graceful `SIGINT` and dynamically read configurations.
**Why**: Hard killing (`kill -9`) processes corrupted the ROS 2 daemon, and constantly restarting the `MicroXRCEAgent` triggered a PX4 bug that froze the serial link.
**How it compares**: Previously, launching required multiple terminal gymnastics and constant FC power cycling. Now, a single script reliably boots everything and preserves the agent connection.
**Hurdle**: Identifying that the PX4 DDS client simply does not recover from a dropped serial connection without a hard reboot.

### 2. Coordinate Frame Discovery (NED vs ENU)
**What**: Used the "Arrow" (`rigid_body_18`) surrogate to trace OptiTrack data through the `mocap_px4_bridge` into the FC's EKF2 visual odometry.
**Why**: PX4 strictly requires a North-East-Down (NED) frame.
**How it compares**: If we hadn't mapped this, the drone would have flown sideways when commanded forward.
**Hurdle**: We discovered the OptiTrack World frame and the Rigid Body pivot are rotated 90 degrees relative to physical reality. This requires a physical adjustment in Motive before flight.

### 3. Command Link Validation
**What**: Fired a safe `DISARM` command from the Pi to the FC and successfully received a `VEHICLE_CMD_RESULT_ACCEPTED` acknowledgment.
**Why**: To prove that when the emergency software killswitch (`emergency_kill.py`) is triggered, the Flight Controller actually listens.
**How it compares**: A proven control link vs an assumed one. The acknowledgment round-trip took ~1.3 milliseconds.
**Hurdle**: Finding a dummy command that PX4 would acknowledge without spinning props (since `CUSTOM_1` was ignored).

## Technical Overview of Changes
- **`startup-sequence.sh` / `shutdown-sequence.sh`**: Switched to graceful kills, added ROS daemon cache resets, and protected `MicroXRCEAgent` from being killed. Added `jq` to read config JSON dynamically.
- **`config/drone_config.json`**: Solidified as the Single Source of Truth for the tracking body name.
- **`drone_control/offboard_control.py`**: Updated to test safe `DISARM` messages and parse acknowledgments.

## Outcome
- ✅ **Stable Startup**: Drone initializes perfectly every time without locking up.
- ✅ **Telemetry Flow**: OptiTrack data is successfully reaching PX4 visual odometry.
- ✅ **Command Authority**: Pi has proven 2-way command control over the FC.
- ⏳ **Motive Re-alignment**: Must adjust Rigid Body orientation in Motive.
- ⏳ **QGC Failsafes**: Must configure Data Link Loss to "Land".

## Learning Summary
1. **The Serial Zombie Bug**: If `MicroXRCEAgent` drops a serial connection, the PX4 client will freeze permanently until rebooted. *Never kill the agent.*
2. **Coordinate Relativity**: Position `[x,y,z]` is strictly relative to the World Origin, regardless of drone orientation.
3. **Rigid Body Orientation**: The drone's "compass" (the rigid body pivot) must have its X-axis pointing out the physical nose for the EKF to work correctly.

## Next Steps
1. **Motive Pivot Adjustment**: Rotate the Arrow/Drone Rigid Body pivot in Motive so X points forward.
2. **QGC Safety**: Set QGroundControl Data Link Loss failsafe to "Land".
3. **End-to-End Dry Run**: Run a full physical test with props off.
