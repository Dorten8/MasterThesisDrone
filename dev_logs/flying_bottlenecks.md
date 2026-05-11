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
