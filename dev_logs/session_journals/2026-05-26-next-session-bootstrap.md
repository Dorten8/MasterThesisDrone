# Next Session Bootstrap: 2026-05-26 — Multi-Angle Collision Sweep Testing & Comparative Kinematics Analysis

Welcome back, Dorten! This bootstrap plan is designed to help you (and your AI assistant) instantly ramp up and systematically execute the next phase of your Master's Thesis experiments.

---

## 🎯 Primary Goals for Today
1. **Accelerate Data Population:** Run consecutive 75° loops under both `Rotating Cage` and `Fixed Cage` configurations to log structured telemetry.
2. **Classify Impacts (`collision` Column):** Add a boolean `collision` column to the SQLite database to filter between actual structural impacts and clean/grazing bypass sweeps.
3. **Iterate Angle Sweeps & Edge Cases:** Begin testing 60° (and other angles) and execute the `odometry along the wall` test to establish nominal baseline envelopes.
4. **Initial Cage Comparative Analysis:** Extract initial rotating vs. fixed cage performance metrics and trace early IMU shock patterns to predict contact vectors.

---

## 🛠️ Step-by-Step Execution Playbook

### STEP 1: Add the `collision` Boolean Column to the SQLite Schema
Before flying, update your SQLite database schema so you can cleanly classify actual structural contact:
1. **Target File:** [exa_database.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/experiments_analysis/exa_database.py)
2. **Schema Update:** Add a `collision INTEGER` (0 or 1) column to the `flights_summary` table schema:
   ```sql
   CREATE TABLE IF NOT EXISTS flights_summary (
       flight_name TEXT PRIMARY KEY,
       condition TEXT,
       sweep_speed REAL,
       battery_at_start REAL,
       impact_speed REAL,
       impact_accel REAL,
       impact_angle REAL,
       avg_dev_after REAL,
       max_dev_after REAL,
       recovery_area REAL,
       closest_clearance REAL,
       collision INTEGER, -- <-- 0 = Clean Bypass, 1 = Structural Impact
       timestamp TEXT
   )
   ```
3. **Automatic Classification Logic:** Inside [exa_pipeline.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/experiments_analysis/exa_pipeline.py), automatically set `collision = 1` if `closest_clearance <= 0` (indicating the physical cage boundary intersected or penetrated the column radius) or if the high-frequency IMU shock wave exceeds `5.0 m/s²` within the active sweep window. Otherwise, set it to `0`.

### STEP 2: Live Flight Operations Protocol
Run consecutive flight sweeps to rapidly populate the raw dataset:
1. **Motive Verification Checklist:**
   - In the Motive GUI, verify the drone's rigid body is active and streaming (target rate: ~358 Hz, payload size: ~980 B/frame, residual: <1.0 mm).
   - Physically move the drone in the volume to force Motive to re-detect it if tracking drops.
2. **Initialization Sequence (Companion & FC):**
   ```bash
   ./startup-sequence.sh
   ```
3. **Execute 75° Sweeps (Rotating vs. Fixed):**
   - Run 3–5 sweeps under `Rotating Cage` (loosen the central Z-axis bearing collar).
   - Run 3–5 sweeps under `Fixed Cage` (lock the bearing collar).
   - *Command:* In the flight control terminal, execute the `ExpCollision75Deg` loop mission.

### STEP 3: The "Odometry Along the Wall" Test
To measure baseline drift and nominal wall-tracking bounds:
- **Concept:** Commands the drone to fly a straight-line waypoint path parallel to a flat vertical surface, measuring the spatial coordinate drift of PX4 EKF2 odometry versus the OptiTrack ground truth.
- **Action:** Utilize the manual offboard joystick node or construct a simple 2-waypoint linear path in `drone_control/missions/` to trace the wall segment at a safe constant offset (e.g. 20cm).

### STEP 4: Segment, Repair, and Populate
Immediately after your flight sessions, run the automated pipeline to parse the telemetry:
1. **Physical Pass Segmenter & Auto-Repair:**
   ```bash
   python3 dev_logs/analysis/experiments_analysis/mcap_segmenter.py
   ```
   This will automatically scan all new flight bags, fix any file truncations, and slice them into sequential `-pass01.mcap`, `-pass02.mcap` bags.
2. **Database Population:**
   ```bash
   python3 -m dev_logs.analysis.experiments_analysis.exa_pipeline
   ```
   This updates the SQLite database cache idempotently.

---

## 📈 Analysis & IMU Shock Predictions

### 1. Rotating vs. Fixed Cage Indicators: What to Look For
When reviewing the generated database stats, track these key performance metrics:
- **Max Trajectory Deviation ($d_{\text{max}}$):** Does the `Rotating Cage` experience a smaller peak rebound deviation due to rotational force absorption compared to the `Fixed Cage`?
- **Recovery Area (SIAE):** Compare the spatial recovery envelope area (in $\text{cm}^2$). A smaller area means the drone recovered its nominal commanded path faster.
- **Yaw Rotational Surge ($w_z$):** In the IMU subplots, does the rotating cage isolate the internal Pixhawk fuselage from yaw spikes during impact compared to the fixed cage?

### 2. High-Frequency IMU Impact Shock Prediction
To train a model that estimates the impact vector solely from the onboard IMU:
- Locate the high-frequency accelerometer spike along the Pixhawk body axes ($a_x, a_y$) at the exact millisecond of contact.
- **Hypothesis:** The ratio of longitudinal shock ($a_y$) to lateral shock ($a_x$) should correlate directly with the physical impact angle $\theta$ normal to the column:
  $$\theta_{\text{predicted}} = \arctan2(a_y, a_x)$$
- Compare this IMU-derived angle against the OptiTrack spatial ground truth (`impact_angle` in the database) to measure predictive accuracy.

---

## ⚠️ What to Look Out For & Warnings
* > [!IMPORTANT]
  > **High Linear Inertia:** The drone weighs ~1.2kg. Do not exceed `0.30 m/s` transit speed near the gate ($WP1$) to prevent geofence overshoot breaches when braking.
* > [!WARNING]
  > **MicroXRCEAgent Stale Socket:** If you restart the Pi or disconnect the agent, always power-cycle the Pixhawk first to prevent the serial DDS client from hanging.
* > [!TIP]
  > **Battery Capacity:** Monitor battery levels closely. Start each flight loop at $\ge 80\%$ capacity; execute an auto-Land if voltage drops below `14.8V` under load.

Let's crush these sweep runs tomorrow!
