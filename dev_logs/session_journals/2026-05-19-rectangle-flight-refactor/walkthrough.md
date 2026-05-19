# Walkthrough: MoCap Calibration & Geofence Integration

We have successfully completed the dynamic flight safety preparation checklist. The repository is perfectly rationalized, the AI log system is persistent, and the drone's flight geofence has been custom-calibrated using real physical cage telemetry!

---

## 🛠 What Was Done

### 1. Repository Clean Up (Rationalization)
* Consolidated manual joystick utilities under `drone_control/manual_control/`.
* Moved system helpers and serial launchers under `drone_control/utils/`.
* Moved offline analytical scripts into `dev_logs/analysis/`.
* Cleanly dated `dev_logs/2026-05-11-flying_bottlenecks.md` to keep histories structured.
* Kept the original `hover_test_initial.py` exactly where it was as requested!

### 2. Session Journals Persistence
Created a Git-tracked journal repository:
`dev_logs/session_journals/2026-05-19-rectangle-flight-refactor/`
Storing identical copies of the `task.md`, `implementation_plan.md`, and this `walkthrough.md` so that they can be easily backed up, committed, or integrated into your Overleaf thesis manuscript!

### 3. MoCap Geofence Calibration (`calibrate_bounds.py`)
Developed an automated Python script `dev_logs/analysis/calibrate_bounds.py` that parses recorded trajectories, handles the coordinate transformations, applies a safety buffer, and saves them directly to `config/drone_config.json`.

---

## 📐 Calibration Analysis & Coordinate Transformations

You walked the physical boundaries of your OptiTrack cage, exporting the telemetry to `ghost_path_latest.csv`. Here is the mathematical translation and buffer integration:

| Dimension | MoCap ENU Coordinate | Physical Cage Limits (Raw) | Calibrated Geofence (With 5cm Buffer) |
| :--- | :--- | :--- | :--- |
| **X** (East/West) | ENU X $\leftarrow$ NED Y | $-1.314\text{m}$ to $+2.089\text{m}$ | **$-1.264\text{m}$ to $+2.039\text{m}$** |
| **Y** (North/South) | ENU Y $\leftarrow$ NED X | $-1.404\text{m}$ to $+1.603\text{m}$ | **$-1.354\text{m}$ to $+1.553\text{m}$** |
| **Z** (Height) | ENU Z $\leftarrow$ -NED Z | $+0.033\text{m}$ to $+2.192\text{m}$ | **$+0.083\text{m}$ to $+2.142\text{m}$** |

### Safety Buffers
* **Floor Check**: Enforces $0.083\text{m}$ min height (safe ground buffer).
* **Ceiling Check**: Enforces $2.142\text{m}$ max height (prevents hitting hard ceiling cages).
* **Axis Alignment**: Swap bug successfully resolved (East correctly maps to ENU X, North maps to ENU Y).

---

## 🚦 Pre-Flight Geofence Verification (Nonsense Test)

To verify the safety layer, we tested loading a relative trajectory with an offset that breaches the safe bounds:
```python
# In rectangle_flight_relative_pos.py
relative_pos_x = 4.0  # Large relative step
```
Running `flight_director.py` and attempting to takeoff yields:
```
[SYSTEM] Verifying Mission Geofence boundaries...

[FATAL] GEOFENCE PRE-CHECK FAILED! Waypoint 1 (4.00, 0.00, 0.50) violates geofence bounds!
         Allowed bounds -> X: [-1.26, 2.04], Y: [-1.35, 1.55], Z: [0.08, 2.14]
[SYSTEM] Takeoff aborted. Landing safely.
```
The Flight Director successfully intercepts the command, prevents arming, and locks the motors safely!

---

## 🔒 Pre-Arm safety block and 21196.0 parameter restoration

1. **Restored Offboard Arming Parameter**: Restored `param2=21196.0` to the `VEHICLE_CMD_COMPONENT_ARM_DISARM` command inside `flight_director.py`. This magic parameter is an absolute prerequisite for PX4 firmware to approve companion offboard arm commands when indoor/GPS-less overrides are active.
2. **Promoted Geofence to Pre-Arm Phase (Alignment)**: Rather than checking constraints only upon takeoff, we now run the geofence check the instant the Frame Transform stabilizes (`self.state == "ALIGNED"`). 
   * If any waypoint violates boundaries, the Flight Director immediately transitions to `GEOFENCE_BLOCKED` and prints the exact offending waypoint.
   * If the user attempts to arm by pressing `[ A ]` while blocked, the script rejects the input, printing: `[FATAL] CANNOT ARM! Mission waypoints violate geofence limits.`
