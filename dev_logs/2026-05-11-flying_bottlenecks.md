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

<details>
<summary>Click to view May 11 analysis (Climb Anomaly Discovery)</summary>

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
- [analyze_jerk_test.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analyze_jerk_test.py): Post-processing script for jerk test bags and ULogs.

</details>

---

## 📊 Diagnostic Analysis — Session 2026-05-14 (Jerk Test & Infrastructure Fixes)

> **Source files:** [`flight_20260514_150702_0.mcap`](file:///home/dorten/pi_drone_sshfs/dev_logs/flights/flight_20260514_150702/) (57.3s) and ULog [`14_50_00.ulg`](file:///home/dorten/pi_drone_sshfs/dev_logs/flights/px4_sd_logs/2026-05-14/14_50_00.ulg) (1131s / 18.9min).
> All values extracted by `analyze_deep_dissect.py` — no estimates.

### 🔬 Hard Evidence Table

| Metric | Raw Value | Source Field | Target | Result |
|---|---|---|---|---|
| Pi wall clock (bag `log_time`) | `2026-05-14 15:07:03 UTC` | `log_time_ns` | Correct date | ✅ |
| PX4 internal `timestamp` | `2026-05-14 15:07:03 UTC` | `vehicle_odometry.timestamp` | Correct date | ✅ SYNCED |
| **Pi ↔ PX4 drift** | **16 ms** | `timestamp` − `log_time_ns` | < 100 ms | ✅ Excellent |
| Bridge internal delta (mean) | **0.9 ms** | `timestamp` − `timestamp_sample` | < 5 ms | ✅ |
| Bridge internal delta (max) | **7.6 ms** | `timestamp` − `timestamp_sample` | < 20 ms | ✅ |
| Mocap raw rate (mean) | **53.8 Hz** | `/poses` inter-message gaps | > 30 Hz | ✅ |
| **Max raw Mocap gap** | **284.7 ms** | `/poses` max gap | < 33 ms | ⚠️ Dropout |
| VVO mean rate to PX4 | **66.3 Hz** | `/fmu/in/vehicle_visual_odometry` | ≥ 50 Hz | ✅ |
| **VVO median gap** | **8.6 ms** | inter-message gap | — | ℹ️ Bursty (see below) |
| **VVO gaps > 33ms** | **761 / 3795 (20%)** | gap distribution | 0 | ⚠️ Bursty bridge |
| **VVO max gap** | **59 ms** | max single gap | < 33 ms | ⚠️ |
| **EV Fusion (Active Phase)** | **100%** | `cs_ev_pos`, `cs_ev_hgt` | > 90% | ✅ FULL FUSION |
| EV Fusion (Full ULog Mean) | **65%** | `cs_ev_pos` mean | — | ℹ️ Binary 0/100% |
| Jerk delay (Jerk #2) | **40.4 ms** | IMU vs Mocap peak delta | Measure only | 📐 EKF2_EV_DELAY |
| ULog file header date | `1970-01-01` | `start_timestamp` | 2026 date | ❌ Boot-before-sync |

---

### H1: Odometry Source Switching — ✅ RESOLVED

**Fix applied:** `EKF2_HGT_REF` → `3` (Vision).

**Evidence from ULog `14_50_00.ulg` (window-by-window dissection):**

```
  Window (s)    EV Pos%   EV Hgt%   Interpretation
  ---------------------------------------------------
     0–300s       0%        0%      🔴 EKF2 not receiving Mocap — startup
    300–310s      15%       15%     🟡 Mocap connecting
    310–1110s    100%      100%     ✅ FULL FUSION — Mocap fully trusted
   1110–1140s     0%        0%      🔴 Mocap disconnected (end of session)
```

> [!IMPORTANT]
> **The "65% average" was misleading.** The EKF2 was never partially fusing — it was **binary**: either 0% (no Mocap) or 100% (full Mocap). The mean of 65% is a mathematical artifact of 5 minutes without Mocap at the start of an 18-minute session.
>
> **What actually happened in the first 5 minutes at 0% fusion:**
> The ULog starts at power-on. The `startup-sequence.sh` takes ~2–3 minutes to initialize. The Mocap node then needs to connect to the OptiTrack server and begin streaming. Until that connection is established, there is literally nothing for the EKF2 to fuse — hence 0%.
>
> **After t=310s: 100% fusion for 13 continuous minutes.** This is the real result. The EKF2 is accepting every Mocap packet when the pipeline is healthy. There is NO evidence of delay-based rejection during normal operation.

**Implication for the delay hypothesis:** My earlier claim that "delay causes 35% rejection" was **incorrect**. The data shows 100% fusion when Mocap is connected. The EKF2_EV_DELAY parameter may still need tuning for accuracy during fast motion, but it is NOT causing packet rejection under normal conditions.

---

### H2: Mocap Dropout & Watchdog — ✅ RESOLVED (with caveat)

**Evidence — 5-second window breakdown (Bag):**

```
  Window    Poses   PoseHz   MaxGap    VVO    VVOHz   VVOMaxGap
  ---------------------------------------------------------------
   0–5s      271    54.2Hz   216.8ms   324   64.8Hz    51.5ms ⚠️
   5–10s     280    56.0Hz   217.5ms   332   66.4Hz    53.8ms ⚠️
  10–15s     272    54.4Hz   154.3ms   319   63.8Hz    55.1ms ⚠️
  15–20s     289    57.8Hz   124.3ms   340   68.0Hz    59.0ms ⚠️
  20–25s     269    53.8Hz   180.1ms   329   65.8Hz    53.5ms ⚠️
  25–30s     268    53.6Hz   284.7ms   340   68.0Hz    51.5ms ⚠️ ← biggest dropout
  30–35s     246    49.2Hz   238.2ms   333   66.6Hz    53.3ms ⚠️
  35–40s     277    55.4Hz   162.8ms   336   67.2Hz    52.0ms ⚠️
  40–45s     264    52.8Hz   220.5ms   326   65.2Hz    53.1ms ⚠️
  45–50s     275    55.0Hz   124.4ms   337   67.4Hz    53.3ms ⚠️
  50–55s     248    49.6Hz   135.1ms   331   66.2Hz    51.7ms ⚠️
  55–60s     121    24.2Hz    70.8ms   149   29.8Hz    52.8ms ⚠️ ← recording ended
```

**Watchdog proof:** Raw Mocap had gaps up to **284.7ms** every window. VVO max gap never exceeded **59ms** in any window — the watchdog is actively filling every dropout.

> [!WARNING]
> **New finding — VVO stream is bursty, not smooth.**
> The VVO gap distribution (Section A) shows the bridge sends **rapid bursts** then pauses, rather than a smooth 66Hz stream. 20% of inter-message gaps exceed 33ms (the 30Hz threshold). The median gap is only 8.6ms, but the mean is 15.1ms — this is classic bursty transmission.
>
> **Why?** The bridge fires immediately on each incoming Mocap packet (pass-through mode), and the watchdog timer fires at 50Hz. These two are not synchronized — they combine to produce uneven timing. During Mocap dropouts, the watchdog fires at 50Hz (20ms). During healthy Mocap, the bridge fires at 53Hz (18.8ms) but with large natural gaps in the OptiTrack stream.
>
> **Risk:** 20% of VVO packets arrive with >33ms gaps. PX4's EKF2 can tolerate this (as proven by 100% fusion), but it is not ideal. The PX4 docs cite 30–50Hz as minimum — our **mean** is well above (66Hz) but our **distribution** is wide.

**VVO Gap Distribution:**

```
  Range (ms)     Count     %    Interpretation
  ------------------------------------------------
   0–10ms        2109    55.6%  Fast back-to-back (Mocap bursts)
  10–20ms         663    17.5%  Normal
  20–30ms         219     5.8%  Normal
  30–40ms         335     8.8%  ⚠️ Below 30Hz threshold
  40–50ms         400    10.5%  ⚠️ Below 25Hz threshold
  50–60ms          69     1.8%  🔴 Below 20Hz (watchdog holdover)
```

---

### H3: EKF2 Latency (`EKF2_EV_DELAY`) — 🟡 MEASURED, NOT YET SET

**Was the drone in the Mocap volume throughout?**

```
  t(s)      X(m)      Y(m)      Z(m)   Status
  -----------------------------------------------
   0.0    -0.793    -0.332     0.801   ✅ In volume
   5.0    -0.793    -0.332     0.801   ✅ In volume (stationary)
  10.0    -0.793    -0.332     0.801   ✅ In volume (stationary)
  15.0    -0.793    -0.332     0.801   ✅ In volume (stationary)
  25.0    -0.795    -0.333     0.801   ✅ In volume (stationary)
  30.0    -0.609    -0.127     1.257   ✅ In volume (being jerked)
  35.0    -0.471    -0.233     1.429   ✅ In volume (being jerked)
  40.0    -0.652    -0.304     1.357   ✅ In volume (being jerked)
  50.0    -0.782    -0.351     0.800   ✅ In volume (returned to ground)
```

**Drone never left the capture volume.** Position throughout stays within ±1.5m of origin.

**Jerk test measurements:**

| Jerk | IMU Excess (m/s²) | Measured Delay | Reliability |
|---|---|---|---|
| 1 | 52.5 | 210 ms | 🔴 Slow movement — diffuse Mocap velocity peak |
| 2 | 78.8 | **40 ms** | ✅ Sharp snap — clean narrow Mocap spike |
| 3 | 28.2 | 229 ms | 🔴 Slow movement |
| 4 | 29.7 | 436 ms | 🔴 Very slow — near 500ms watchdog limit |

**Why only Jerk 2 is reliable:** The Mocap velocity peak (rate of position change) is only sharp and narrow when the physical movement itself is sharp. A slow 0.5-second shake produces a broad, gradual velocity curve — the algorithm picks the maximum of that curve which could be anywhere in a wide window. A fast 50ms snap produces a spike with a clear single maximum.

**Action:** Set `EKF2_EV_DELAY = 50ms`. Repeat the test with **snapping** movements (not shaking). The script is ready and auto-detects the newest bag.

---

### Clock Sync & ULog Date — ✅ Pi Clock Working / ❌ ULog Header Still 1970

**What works:** Pi ↔ PX4 drift = **16ms**. Bag data has correct 2026 timestamps.

**Why ULog headers still show 1970:** PX4 creates the log file at the exact moment of power-on. The startup sequence takes ~2–3 min to boot, connect the DDS Agent, and push the time sync. The log file header is already written at 1970 before any sync arrives. With `SDLOG_MODE=from boot until shutdown`, this is unavoidable.

**Implication for thesis:** Only the file metadata shows 1970. Internal data timestamps are correct after t≈30s from boot. For thesis figures, use bag data (correct timestamps) or reference data points by their relative time-in-log, not absolute date.

## 🛠 Architecture Blueprint: PX4 / ROS 2 Offboard Mocap Integration (Established May 14)

### 1. The Core Problems (Root Causes Identified)
*   **The "Talking to Myself" Bug:** Commands sent with `source_system = 1` are dropped by the FC. Use **`255`** (GCS ID).
*   **The Mocap Velocity Trap:** EKF2 rejects Mocap if 3D velocity fusion is enabled but only position is sent. Disable velocity fusion in `EKF2_EV_CTRL`.
*   **The NaN Setpoint Requirement:** Unused fields in `TrajectorySetpoint` (velocity, acceleration, etc.) **must** be set to `float('nan')`, not `0.0`, to prevent the PX4 trajectory generator from crashing.
*   **The 3-Second Arming Buffer:** Offboard mode requires a heartbeat of at least 2-3 seconds before an Arm command will be accepted.

### 2. Final Parameter Blueprint (Indoor Autonomous)
| Parameter | Value | Rationale |
| :--- | :--- | :--- |
| **`EKF2_EV_CTRL`** | `11` | Horizontal, Vertical, Yaw only (No Velocity). |
| **`EKF2_EV_DELAY`** | `60 ms` | Based on May 14 Jerk Test results. |
| **`COM_ARM_WO_GPS`** | `1` | Allow arming without Global Position. |
| **`COM_RC_IN_MODE`** | `4` | Disable stick requirement for autonomous flight. |
| **`COM_RCL_EXCEPT`** | `7` | Prevent RC Loss failsafe in Offboard mode. |
| **`NAV_DLL_ACT`** | `0` | Disable Data Link Loss (USB unplugged check). |
| **`CBRK_IO_SAFETY`** | `22027` | Disable physical safety button requirement. |
| **`SDLOG_MODE`** | `1` | Record from boot to shutdown for persistent data. |

### 3. Operational Protocol (The XRCE Protocol)
*   **Power Cycle:** If the Flight Controller reboots, the `MicroXRCEAgent` on the Pi **must** be restarted to reset the serial handshake.
*   **Agent Inspection:** Use the new [start_xrce_agent.sh](file:///home/dorten/pi_drone_sshfs/drone_control/start_xrce_agent.sh) script to run the agent in a detached `screen` session. 
    *   **To inspect logs:** `screen -r xrce_agent`
    *   **To detach:** `Ctrl+A` then `D`.
    *   This prevents the "Ghost Connection" trap where the agent remains running on a dead serial link.
*   **Failsafe:** If Mocap is lost, the drone is now configured to fail over to `Hold` mode rather than `RTL`.
