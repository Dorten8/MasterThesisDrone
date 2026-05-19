# Session Journal: 2026-05-14 — Mocap Pipeline Stabilization

## Where Are We in the Project
We have moved from "observing anomalies" to "quantifying and fixing" the core bottlenecks. The drone's Mocap-to-PX4 pipeline is now synchronized, latency-compensated, and robust against dropouts. We have also resolved the "Identity Crisis" and QoS blockers that were preventing RC-less arming.

## What We Worked On and Why

### 1. Jerk Test & Latency Measurement
*   **What:** Performed a high-acceleration "Jerk Test" using a new automated script.
*   **Why:** To quantify the exact delay between physical movement (IMU) and Mocap feedback.
*   **How it compares:** Replaced guesswork with a measured **60.1 ms** delay.
*   **Hurdle:** Noise and slow movements produced broad peaks; only the "sharp snap" (Jerk #2) provided a reliable measurement.

### 2. Autonomous Arming & Offboard Heartbeat
*   **What:** Integrated a 10Hz Offboard heartbeat and Trajectory setpoints into the arming script.
*   **Why:** PX4 requires active setpoints and a 2-3s buffer before accepting an Offboard arm command without an RC link.
*   **How it compares:** Moved from failing "Manual Control" checks to a stable "Autonomous Input" state.
*   **Hurdle:** Encountered an "Identity Crisis" where `source_system = 1` caused PX4 to drop its own commands; fixed by using GCS ID `255`.

### 3. QoS & Middleware Standardization
*   **What:** Updated all ROS 2 publishers/subscribers to use `Best Effort` / `Volatile` QoS.
*   **Why:** Resolved a mismatch that was blocking position messages from reaching the control script.
*   **How it compares:** Achieved 100% data transparency between the PX4 Agent and the Python logic.

### 4. Operational "Ghost Connection" Protocol
*   **What:** Created [start_xrce_agent.sh](file:///home/dorten/pi_drone_sshfs/drone_control/start_xrce_agent.sh) to run the Agent in a separate `screen` session.
*   **Why:** To prevent dead UART sessions from "ghosting" after a power cycle and to allow inspection without stopping the ROS 2 pipeline.

## Technical Overview of Changes
- **PX4 Parameters:** Set `EKF2_EV_DELAY = 60ms`, `EKF2_EV_CTRL = 11`, `COM_RC_IN_MODE = 4`.
- **Scripts:** Updated [analyze_jerk_test.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analyze_jerk_test.py) and [offboard_control.py](file:///home/dorten/pi_drone_sshfs/drone_control/offboard_control.py).
- **Documentation:** Consolidated all findings into the "Architecture Blueprint" at the end of [flying_bottlenecks.md](file:///home/dorten/pi_drone_sshfs/dev_logs/flying_bottlenecks.md).

## Outcome
- ✅ Measured 60ms pipeline latency.
- ✅ Resolved RC-less arming blocks.
- ✅ Implemented `screen`-based Agent management.
- ✅ Confirmed 100% EKF2 fusion health.
- ⏳ Validation 1m Hover Flight (Scheduled for next session).

## Learning Summary
1. **Source IDs Matter:** Never use `1` for companion commands; PX4 ignores commands that claim to come from itself.
2. **NaN is Safety:** Trajectory setpoints must use `NaN` for unused fields to avoid generator crashes.
3. **QoS is Binary:** A single `Reliable` subscriber on a `Best Effort` topic blocks all data.
4. **Latency is measurable:** Correlating IMU spikes with Mocap velocity is the gold standard for `EKF2_EV_DELAY` tuning.

## Next Steps
1. **Execute 1m Hover Benchmark** using [offboard_control.py](file:///home/dorten/pi_drone_sshfs/drone_control/offboard_control.py).
2. **Analyze ULog** from the SD card to verify altitude stability during hover.
3. **Finalize Thesis Plots** using the now-standardized MCAP bag format.
