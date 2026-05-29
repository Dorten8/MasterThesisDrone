import os
import sys
import math

# Add project root to sys path dynamically
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, "..", ".."))
sys.path.append(os.path.join(project_root, "dev_logs", "analysis"))

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

def analyze_impact(bag_path, flight_name):
    print(f"\n{'='*60}")
    print(f"💥 ANALYZING IMU IMPACT DYNAMICS FOR: {flight_name}")
    print(f"{'='*60}")
    
    if not os.path.exists(bag_path):
        print(f"[ERROR] Bag file not found: {bag_path}")
        return

    accel_data = []
    decoder_factory = DecoderFactory()
    
    try:
        with open(bag_path, "rb") as f:
            reader = make_reader(f, decoder_factories=[decoder_factory])
            for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=['/fmu/out/sensor_combined']):
                accel_data.append({
                    't': message.log_time,
                    'ax': ros_msg.accelerometer_m_s2[0],
                    'ay': ros_msg.accelerometer_m_s2[1],
                    'az': ros_msg.accelerometer_m_s2[2],
                    'gx': ros_msg.gyro_rad[0],
                    'gy': ros_msg.gyro_rad[1],
                    'gz': ros_msg.gyro_rad[2]
                })
    except Exception as e:
        print(f"[ERROR] Failed to read bag: {e}")
        return

    if not accel_data:
        print("[WARN] No '/fmu/out/sensor_combined' data found in bag!")
        return

    min_t = accel_data[0]['t']
    max_accel = 0.0
    max_accel_time = 0.0
    
    spikes = []
    threshold = 5.0
    
    for row in accel_data:
        t_sec = (row['t'] - min_t) / 1e9
        a_mag = math.sqrt(row['ax']**2 + row['ay']**2 + row['az']**2)
        g_mag = math.sqrt(row['gx']**2 + row['gy']**2 + row['gz']**2)
        a_deviation = abs(a_mag - 9.81)
        
        if a_deviation > max_accel:
            max_accel = a_deviation
            max_accel_time = t_sec
            
        if a_deviation > threshold:
            spikes.append((t_sec, a_deviation, g_mag))
            
    print(f"📊 Maximum Acceleration Deviation: {max_accel:.2f} m/s^2 at t = {max_accel_time:.2f}s")
    
    if spikes:
        print(f"⚠️  Detected High-Acceleration Events (> {threshold} m/s^2 deviation):")
        # Group closely spaced spikes (within 1 second of each other)
        grouped_spikes = []
        current_group = []
        for spike in spikes:
            if not current_group:
                current_group.append(spike)
            else:
                if spike[0] - current_group[-1][0] < 1.0:
                    current_group.append(spike)
                else:
                    grouped_spikes.append(current_group)
                    current_group = [spike]
        if current_group:
            grouped_spikes.append(current_group)
            
        for group in grouped_spikes:
            # Find peak in group
            peak = max(group, key=lambda x: x[1])
            print(f"  -> Candidate Impact at t = {peak[0]:.2f}s | Accel Deviation: {peak[1]:.2f} m/s^2 | Gyro Surge: {peak[2]:.2f} rad/s")
    else:
        print("✅ No severe physical impacts detected (no spikes > 5 m/s^2).")

if __name__ == "__main__":
    flights = [
        "flight_20260524-1136_75°_column_collision_loop_rotating_cage",
        "flight_20260524-1143_75°_column_collision_loop_rotating_cage"
    ]
    base_dir = os.path.join(project_root, "dev_logs", "flights")
    for f in flights:
        bag = os.path.join(base_dir, f, f + "_0.mcap")
        analyze_impact(bag, f)
