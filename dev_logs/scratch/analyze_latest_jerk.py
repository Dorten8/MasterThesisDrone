import numpy as np
from mcap_ros2.reader import read_ros2_messages
import datetime
import os

BAG_PATH = "/home/dorten/pi_drone_sshfs/dev_logs/flights/flight_20260514_181505/flight_20260514_181505_0.mcap"

print(f"Analyzing {BAG_PATH}...")

topic_data = {"/fmu/out/sensor_combined": [], "/poses": []}

for msg in read_ros2_messages(BAG_PATH):
    t = msg.channel.topic
    if t in topic_data:
        topic_data[t].append(msg)

print(f"Loaded {len(topic_data['/fmu/out/sensor_combined'])} IMU messages")
print(f"Loaded {len(topic_data['/poses'])} Mocap messages")

# 1. IMU Accelerator magnitude
imu_times, imu_mag = [], []
for m in topic_data["/fmu/out/sensor_combined"]:
    # sensor_combined.accelerometer_m_s2 is a float[3]
    ax, ay, az = m.ros_msg.accelerometer_m_s2
    imu_times.append(m.log_time_ns * 1e-9)
    imu_mag.append(np.sqrt(ax**2 + ay**2 + az**2))

imu_times = np.array(imu_times)
imu_mag = np.array(imu_mag)

# 2. Mocap position change
mocap_times, mocap_x = [], []
for m in topic_data["/poses"]:
    t = m.log_time_ns * 1e-9
    # NamedPoseArray.poses is a list
    for p in m.ros_msg.poses:
        if "jake" in p.name.lower() or "drone" in p.name.lower():
            mocap_times.append(t)
            mocap_x.append(p.pose.position.x)
            break

mocap_times = np.array(mocap_times)
mocap_x = np.array(mocap_x)

if len(imu_times) == 0 or len(mocap_times) == 0:
    print("Missing data topics!")
    exit(1)

# Find Jerks
g = 9.81
imu_excess = np.abs(imu_mag - g)
thresh = np.mean(imu_excess) + 3.0 * np.std(imu_excess)
peak_idx = np.where(imu_excess > thresh)[0]

if len(peak_idx) == 0:
    print(f"No jerks found above {thresh:.2f} m/s2")
    exit(0)

# Cluster
clusters = []
if len(peak_idx) > 0:
    cs = peak_idx[0]
    for i in range(1, len(peak_idx)):
        if peak_idx[i] - peak_idx[i-1] > 100: # 100 samples gap
            clusters.append((cs, peak_idx[i-1]))
            cs = peak_idx[i]
    clusters.append((cs, peak_idx[-1]))

print(f"Found {len(clusters)} jerk events\n")

for i, (start, end) in enumerate(clusters):
    best_imu_idx = start + np.argmax(imu_excess[start:end+1])
    t_imu = imu_times[best_imu_idx]
    
    # Mocap velocity proxy (dX/dt)
    mv = np.abs(np.diff(mocap_x) / np.diff(mocap_times))
    mv_t = (mocap_times[:-1] + mocap_times[1:]) / 2
    
    # Find mocap peak within 0.5s of IMU peak
    window = (mv_t > t_imu - 0.1) & (mv_t < t_imu + 0.5)
    if np.sum(window) > 0:
        t_mocap = mv_t[window][np.argmax(mv[window])]
        delay = (t_mocap - t_imu) * 1000
        print(f"Event {i+1}: IMU Peak @ {t_imu:.3f}s, Mocap Peak @ {t_mocap:.3f}s, Delay = {delay:.1f} ms")
    else:
        print(f"Event {i+1}: IMU Peak @ {t_imu:.3f}s, No Mocap correlate found")
