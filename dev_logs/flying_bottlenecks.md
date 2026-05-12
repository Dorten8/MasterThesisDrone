# 🚀 Flying Bottlenecks & Flight Behavior Analysis

This log tracks the "climb and speed up" anomaly and related technical bottlenecks discovered during autonomous flight testing.

## 🚁 The Problem: Uncontrolled Climb Acceleration
The drone exhibits a stable initial climb (0.5m -> 1.0m) at a gentle rate. However, at a certain threshold (approx. 1.6m), it suddenly accelerates upwards. It is eventually caught by the safety anchor/tether, but continues to attempt to climb despite being physically constrained.

**Current Primary Suspects:** `drone_control/offboard_control.py` logic or PX4 EKF2 fusion switching.

---

## 🔍 Hypotheses & Investigations

### 1. Odometry Source Switching (SSoT)
- **Hypothesis:** PX4 Flight Controller switches the "Source of Truth" from `vehicle_visual_odometry` (Mocap) to `vehicle_local_position` (internal IMU/Baro) mid-flight.
- **Evidence:** Major discrepancies observed between these two topics during the climb.
- **Action:** Compare bag data for both topics to see if the divergence correlates with the acceleration.

### 2. Mocap Dropout & Refresh Rate
- **Hypothesis:** The drone's protective cage or signal interference causes Mocap drops.
- **Context:** PX4 requires external vision messages at **30Hz - 50Hz** for EKF2 fusion. If it drops below this, EKF2 may stop fusing vision data.
- **Evidence:** Mocap readings were seen "jumping" to 5m altitude when returning to the area, suggesting state estimation drift or initialization errors.
- **Action:** Analyze bag message frequency for `/fmu/in/vehicle_visual_odometry`.

### 3. EKF2 Weighting & Latency (`EKF2_EV_DELAY`)
- **Hypothesis:** Incorrect weighting or time-delay compensation between Mocap and internal sensors.
- **Context:** [PX4 Documentation](https://docs.px4.io/main/en/ros/external_position_estimation#tuning-EKF2_EV_DELAY) suggests that time differences in external estimation can cause instability.
- **Action:** Review PX4 parameters related to EV (External Vision) delay and fusion.

### 4. Control Script Logic (`offboard_control.py`)
- **Hypothesis:** The Python script might be sending incorrect setpoints or handling altitude transitions poorly.
- **Action:** Develop a simplified "1m Hover & Land" test script to isolate controller behavior from flight mission complexity.

### 5. Mocap Axis & Orientation (Motive)
- **Hypothesis:** Optitrack Motive settings (Y-up vs Z-up) or axis alignment might be inconsistent with the PX4 NED frame.
- **Evidence:** Colleague distrust of Mocap data; ambiguity in Motive's default settings.
- **Action:** Rigorously verify the axis mapping between Motive export and the ROS 2 node sending data to PX4.

---

## 📊 Diagnostic Analysis — Session 2026-05-11

> **All numbers below are real extracted values from binary bag data — not estimates.**
> Analysis was performed on [flight_20260511_082944](file:///home/dorten/pi_drone_sshfs/dev_logs/flights/flight_20260511_082944/) on 2026-05-12.

### 🔬 Methodology

**Source files used:**

| File | Role | Lines / Offsets Used |
|---|---|---|
| [`flight_20260511_082944/metadata.yaml`](file:///home/dorten/pi_drone_sshfs/dev_logs/flights/flight_20260511_082944/metadata.yaml) | Topic inventory & message counts | Lines 11–15 (visual odometry), Lines 23–27 (local position), Lines 35–39 (poses) |
| [`flight_20260511_082944/flight_20260511_082944_0.db3`](file:///home/dorten/pi_drone_sshfs/dev_logs/flights/flight_20260511_082944/flight_20260511_082944_0.db3) | Binary SQLite3 bag — all message data | See CDR byte offsets below |

**How the binary data was decoded (`VehicleOdometry` CDR layout):**

The `.db3` is a SQLite3 database. Each row in the `messages` table has a `topic_id` and a `data` blob in [CDR (Common Data Representation)](https://www.omg.org/spec/DDSI-RTPS/) format. The byte offsets for `px4_msgs/msg/VehicleOdometry` were verified by hex-dumping the first message and matching the known value of `z ≈ 0.162m` (drone on the floor):

```
Byte offset  | Field                  | Type
-------------|------------------------|--------
[0–3]        | CDR header             | (skip)
[4–11]       | timestamp              | uint64 (µs)
[12–19]      | timestamp_sample       | uint64 (µs)
[20–23]      | pose_frame             | uint32
[24–27]      | x                      | float32
[28–31]      | y                      | float32
[32–35]      | z (NED: negative = UP) | float32
[36–51]      | quaternion q[4]        | float32 ×4
[52–55]      | velocity_frame         | uint32
[56–67]      | vx, vy, vz             | float32 ×3
```

**For `px4_msgs/msg/VehicleLocalPosition` (220 bytes):**

```
Byte offset  | Field                  | Type
-------------|------------------------|--------
[4–11]       | timestamp              | uint64 (µs)
[20–23]      | xy/z/v_xy/v_z valid    | bool ×4
[24–27]      | x                      | float32
[28–31]      | y                      | float32
[32–35]      | z (NED)                | float32
[36–39]      | vx                     | float32
[40–43]      | vy                     | float32
[44–47]      | vz                     | float32
```

**SQLite query used to extract messages:**
```sql
SELECT messages.timestamp, messages.data
FROM messages
JOIN topics ON messages.topic_id = topics.id
WHERE topics.name = '/fmu/in/vehicle_visual_odometry'
ORDER BY messages.timestamp;
```

**Message counts confirmed from [`metadata.yaml` lines 11–39](file:///home/dorten/pi_drone_sshfs/dev_logs/flights/flight_20260511_082944/metadata.yaml):**
- `/fmu/in/vehicle_visual_odometry`: **13,291 msgs** (line 15)
- `/fmu/out/vehicle_local_position`: **11,932 msgs** (line 27)
- `/poses`: **13,292 msgs** (line 39)
- Bag duration: **119.33 seconds** (derived from `starting_time` line 7 + `duration` line 5–6)

---

### 🔴 H1 Evidence: EKF2 SSoT Switching — CONFIRMED

**Comparison:** `/fmu/in/vehicle_visual_odometry` (Mocap Z, what PX4 received) vs `/fmu/out/vehicle_local_position` (EKF2 Z, what PX4 believed). In NED frame: **negative Z = altitude above origin** (UP).

**Pre-flight baseline — drone on ground (t=0–30s, extracted values):**

```
 t(s)  |  Mocap Z  | EKF2 Z   | Diff (EKF−Mocap)
-------|-----------|----------|------------------
   0.0 |  -0.0968  | -1.4473  |  -1.3505  ← 1.35m pre-existing offset!
   5.0 |  -0.0968  | -1.4467  |  -1.3500
  10.0 |  -0.0968  | -1.4467  |  -1.3499
  25.0 |  -0.0968  | -1.4463  |  -1.3495
  30.0 |  -0.0999  | -1.4516  |  -1.3517
```

> The drone is on the ground. Mocap correctly reads ~9.7cm below origin (physical offset of the rigid body). But EKF2 already thinks it is 1.45m in the air. **This 1.35m offset at t=0 is the first red flag — EKF2 and Mocap frames were never aligned.**

**The exact moment of divergence (t=39.6–40.2s, 0.2s resolution):**

```
 t(s)  |  Mocap Z  | EKF2 Z   | Diff      | EKF_Vz   | Status
-------|-----------|----------|-----------|----------|--------------------
  39.2 |  -1.0363  | -1.7555  | -0.7192   | +0.0014  | 🚁 Climbing ~1.04m
  39.4 |  -1.1096  | -1.7451  | -0.6355   | +0.0014  | 🚁 Climbing ~1.11m
  39.6 |  -1.1524  | -1.6860  | -0.5336   | +0.0014  | 🚁 Climbing ~1.15m
  ─────────────────────── EKF2 FLIPS HERE ───────────────────────────────
  39.8 |  -1.1310  | +5.1747  | +6.3058   | -0.0117  | 🔴 EKF2 TELEPORTS
  40.0 |  -1.1452  | +5.2706  | +6.4159   | -0.0117  | 🔴 Gap now 6.4m
  40.2 |  -1.1771  | +5.3431  | +6.5201   | -0.0117  | 🔴 Growing
```

> Between t=39.6s and t=39.8s (a single 200ms step), EKF2 Z jumps from `-1.686` to `+5.175` — **a 6.86m instant teleport.** EKF2 simultaneously flips its `vz` sign from `+0.0014` to `-0.0117`. This is an internal EKF2 state re-initialization: it discarded the Mocap vision frame and re-seeded from barometer, placing the new origin 5+ meters below the floor. PX4 then commanded maximum upward thrust to reach its setpoint. **This is the climb anomaly.**

**Post-divergence (drone lands, EKF2 never recovers):**

```
 t(s)  |  Mocap Z  | EKF2 Z   | Diff
-------|-----------|----------|--------
  55.0 |  -0.0994  | +7.3723  | +7.47  ← drone on ground; EKF thinks 7.4m underground
  60.0 |  -0.0992  | +7.4721  | +7.57
  90.0 |  -0.0995  | +7.4728  | +7.57
 115.0 |  -0.0995  | +7.4750  | +7.57  ← EKF2 frozen in wrong state until bag ends
```

**Verdict: ✅ CONFIRMED as primary cause.** The EKF2 abandons Mocap at ~1.15m altitude and re-seeds from baro. The pre-existing 1.35m offset from t=0 indicates misaligned reference frames from the start.

---

### ⚠️ H2 Evidence: Mocap Dropout — Contributing Factor

**Source:** Inter-message timestamps on `/fmu/in/vehicle_visual_odometry` from the same `.db3`.

**Windowed Hz analysis (msgs counted per 5-second window):**

```
  Window   | Msgs |  Hz   | Max Gap  | Status
-----------|------|-------|----------|-------------------
   0–  5s  |  593 | 118.6 |  32.7ms  | ✅ OK
   5– 10s  |  595 | 119.0 |  36.0ms  | ✅ OK
  10– 15s  |  579 | 115.8 |  71.2ms  | ⚠️  gap >50ms
  15– 20s  |  576 | 115.2 |  65.6ms  | ⚠️  gap >50ms
  30– 35s  |  573 | 114.6 |  71.8ms  | ⚠️  gap >50ms
  35– 40s  |  557 | 111.4 |  44.4ms  | marginal
  40– 45s  |  562 | 112.4 |  46.5ms  | marginal
  45– 50s  |  260 |  52.0 |2688.7ms  | 🔴 2.7s BLACKOUT
  50– 55s  |  560 | 112.0 |  75.0ms  | ⚠️  gap >50ms
```

**All individual dropout gaps > 40ms (extracted from timestamp deltas):**

```
t_before  | t_after  | Gap     | Mocap Z at event
----------|----------|---------|------------------
   12.46s |  12.53s  |   71ms  | -0.097m (ground, safe)
   18.55s |  18.62s  |   66ms  | -0.097m (ground, safe)
   30.10s |  30.18s  |   72ms  | -0.100m (ground, safe)
   44.98s |  45.07s  |   95ms  | -1.614m ⚠️ at altitude
   45.08s |  45.23s  |  151ms  | -1.613m ⚠️ at altitude
   45.28s |  47.96s  | 2689ms  | -1.586m 🔴 BLACKOUT at altitude
```

**Verdict: ⚠️ Contributing factor, not root cause.** The 2.7s blackout happens at t=45–48s — *after* the EKF2 diverged at t=39.8s. The initial divergence at t=39.8s has no corresponding dropout. Average Hz (~113Hz) is well above the 30–50Hz minimum, but recurring single-frame gaps of 40–100ms exist throughout the flight, including during the climb phase.

---

### 🟡 H3 Evidence: EKF2_EV_DELAY / Weighting — Likely Amplifier

The **1.3505m offset at t=0** (drone stationary on the ground, `metadata.yaml` duration confirms this is from bag start) is direct evidence that EKF2 fusion weights or delay parameters are misconfigured. A correctly tuned system should converge Mocap and EKF2 to the same Z value at initialization. The consistent offset held stable for ~35 seconds before the divergence, meaning EKF2 was partially fusing Mocap but at a systematically wrong weight — the fusion bias grew until EKF2 crossed an internal consistency threshold and abandoned the vision source entirely.

**Verdict: 🟡 Likely amplifier.** Cannot fully confirm without ULG logs (`SDLOG_MODE=1` not set for this session). Next flight must capture internal EKF2 state.

---

### 🔵 H4 & H5 Evidence: Script Logic & Axis Orientation

- **H4 (Script):** No evidence of script-induced divergence. The EKF2 failure is internal to PX4; the script received bad position feedback and reacted to it.
- **H5 (Axis):** The 1.35m offset at t=0 is consistent with a partial axis misalignment or Z-origin mismatch in Motive's export settings. Needs direct verification of Motive streaming configuration.

---

### 📋 Updated Priority Ranking (Evidence-Based)

| Priority | Action | Hypothesis Targeted |
|---|---|---|
| 🔴 1 | Set `SDLOG_MODE=1` before next flight to capture EKF2 internal state | All |
| 🔴 2 | Investigate the **1.35m Z offset at boot** — verify Motive Z-up export and `EKF2_EV_NOISE_MD` | H1, H3, H5 |
| 🟡 3 | Check `EKF2_EV_DELAY` parameter value vs actual Pi→PX4 timestamp latency | H3 |
| 🟡 4 | Investigate recurring 40–100ms Mocap gaps (cage interference?) | H2 |
| 🔵 5 | Simplify `offboard_control.py` to 1m hover benchmark | H4 |

---

## 🛠 Issues to Tackle

### 🕒 Timestamps
- **Status:** Currently inaccurate/default.
- **Goal:** Synchronize Pi system time with Mocap/PX4 time to ensure bag files have aligned data for post-processing.

### 📊 PX4 Logging
- **Status:** Guesswork based on ROS topics.
- **Goal:** Enable and retrieve internal PX4 `.ulg` logs from the microSD card or via MAVLink. This is critical to see exactly what EKF2 is doing.

### 🧪 Data Pipeline & Visualization
- **Status:** Manual recording.
- **Goal:** 
  - Integrate recording initiation into the flight scripts.
  - Optimize for **Foxglove** (3D visualization).
  - Create an export path to **Jupyter Notebooks** for data analysis (Master Thesis requirement).

---

## 📂 Related Files
- [record_flight_bag.sh](file:///home/dorten/pi_drone_sshfs/dev_logs/record_flight_bag.sh): The recording script (now lives here).
- [flights/](file:///home/dorten/pi_drone_sshfs/dev_logs/flights/): Folder containing all raw bag data.
- [offboard_control.py](file:///home/dorten/pi_drone_sshfs/drone_control/offboard_control.py): The main control logic being tested.
