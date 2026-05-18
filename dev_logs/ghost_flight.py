#!/usr/bin/env python3
"""
Ghost Flight Path Recorder
==========================
Subscribes to PX4's VehicleLocalPosition topic (NED frame), logs the coordinates,
and outputs a smoothed, regularized XYZ trajectory CSV file for autonomous playback.
"""

import os
import sys
import time
import csv
import datetime
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition

class GhostFlightRecorder(Node):
    def __init__(self):
        super().__init__('ghost_flight_recorder')
        
        # Standard best-effort QoS for PX4 status streams
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._position_cb,
            qos_profile
        )
        
        self.recorded_data = []
        self.recording = False
        self.start_time = None
        
        # Connection status tracking
        self.last_msg_time = 0.0
        self.connection_timer = self.create_timer(1.0, self._check_connection)
        self.connected = False

    def _position_cb(self, msg):
        self.last_msg_time = time.time()
        if not self.connected:
            self.connected = True
            print("\n📍 [MoCap Lock] Active tracking stream detected! Ready to record.")
            
        if self.recording:
            if self.start_time is None:
                self.start_time = time.time()
            
            elapsed_ms = int((time.time() - self.start_time) * 1000)
            
            # Record NED frame: X=North, Y=East, Z=Down (negative = UP)
            self.recorded_data.append({
                'timestamp_ms': elapsed_ms,
                'x': msg.x,
                'y': msg.y,
                'z': msg.z,
                'yaw': msg.heading
            })

    def _check_connection(self):
        if time.time() - self.last_msg_time > 2.0:
            if self.connected:
                self.connected = False
                print("\n⚠️ [MoCap Lost] No tracking updates from EKF2. Check OptiTrack or XRCE-DDS Agent!")

    def start_recording(self):
        self.recorded_data.clear()
        self.start_time = None
        self.recording = True
        print("\n⏺️  Recording started! Move the drone through the flight volume...")

    def stop_recording(self):
        self.recording = False
        print(f"\n⏹️  Recording stopped. Captured {len(self.recorded_data)} raw data points.")
        return self.recorded_data


def live_feedback_loop(node, stop_event):
    """Prints live XYZ coordinates at 5Hz during active recording."""
    while not stop_event.is_set():
        if node.recording and len(node.recorded_data) > 0:
            last_pt = node.recorded_data[-1]
            elapsed_sec = last_pt['timestamp_ms'] / 1000.0
            # z is negative UP in NED frame, so print positive Height for user comfort
            height = -last_pt['z']
            sys.stdout.write(
                f"\r   T+{elapsed_sec:5.1f}s | X: {last_pt['x']:6.3f}m | Y: {last_pt['y']:6.3f}m | Height: {height:6.3f}m (Points: {len(node.recorded_data):<5})"
            )
            sys.stdout.flush()
        time.sleep(0.2)
    print()


def save_and_smooth_path(raw_data, output_dir):
    """Processes the raw trajectory, applies smoothing, and saves as a clean CSV."""
    if len(raw_data) < 10:
        print("❌ Error: Too few data points to save a valid path.")
        return

    os.makedirs(output_dir, exist_ok=True)
    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filepath = os.path.join(output_dir, f"ghost_path_{timestamp_str}.csv")
    symlink_path = os.path.join(output_dir, "ghost_path_latest.csv")

    # 1. Regularize and smooth the path (moving average to filter out human hand tremor)
    timestamps = np.array([pt['timestamp_ms'] for pt in raw_data])
    xs = np.array([pt['x'] for pt in raw_data])
    ys = np.array([pt['y'] for pt in raw_data])
    zs = np.array([pt['z'] for pt in raw_data])
    yaws = np.array([pt['yaw'] for pt in raw_data])

    # Re-sample to a clean 10Hz grid (every 100ms)
    t_min, t_max = timestamps[0], timestamps[-1]
    t_new = np.arange(t_min, t_max, 100) # 100ms intervals

    x_new = np.interp(t_new, timestamps, xs)
    y_new = np.interp(t_new, timestamps, ys)
    z_new = np.interp(t_new, timestamps, zs)
    yaw_new = np.interp(t_new, timestamps, yaws)

    # Apply rolling window smoothing (window size = 5 samples / 0.5 seconds)
    window = 5
    def smooth(arr):
        return np.convolve(arr, np.ones(window)/window, mode='same')

    x_smooth = smooth(x_new)
    y_smooth = smooth(y_new)
    z_smooth = smooth(z_new)
    # Ensure edge cases from 'same' convolution don't taper off to 0
    x_smooth[0:2], x_smooth[-2:] = x_new[0:2], x_new[-2:]
    y_smooth[0:2], y_smooth[-2:] = y_new[0:2], y_new[-2:]
    z_smooth[0:2], z_smooth[-2:] = z_new[0:2], z_new[-2:]

    # 2. Write smoothed path to CSV
    with open(filepath, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp_ms', 'x', 'y', 'z', 'yaw'])
        for i in range(len(t_new)):
            # Convert elapsed time relative to start (t_new[0] = 0)
            elapsed = int(t_new[i] - t_min)
            writer.writerow([
                elapsed,
                round(float(x_smooth[i]), 4),
                round(float(y_smooth[i]), 4),
                round(float(z_smooth[i]), 4),
                round(float(yaw_new[i]), 4)
            ])

    # Update symlink for easy access in flight scripts
    if os.path.exists(symlink_path):
        os.remove(symlink_path)
    os.symlink(os.path.basename(filepath), symlink_path)

    print(f"\n📂 Path saved successfully!")
    print(f"   💾 Raw & Smoothed Data: {filepath}")
    print(f"   🔗 Latest Reference:    {symlink_path}")


def main():
    rclpy.init()
    node = GhostFlightRecorder()

    # Spin ROS 2 in a background thread so we can use interactive inputs in main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("==================================================")
    print("🛸 GHOST FLIGHT RECORDER ACTIVE")
    print("==================================================")
    print("   Waiting for Pixhawk EKF2 position updates...")

    # Wait for the EKF2 update stream
    while not node.connected and rclpy.ok():
        time.sleep(0.5)

    input("\n👉 Press ENTER when you are ready to begin walking the path...")
    
    node.start_recording()

    # Start live terminal printout thread
    stop_live_print = threading.Event()
    print_thread = threading.Thread(target=live_feedback_loop, args=(node, stop_live_print), daemon=True)
    print_thread.start()

    try:
        input("\n🚶 Move the drone. Press ENTER to stop recording and process the path...\n")
    except KeyboardInterrupt:
        pass

    stop_live_print.set()
    print_thread.join()

    raw_path = node.stop_recording()
    
    # Save files to dev_logs/paths
    output_directory = "/home/dorten/pi_drone_sshfs/dev_logs/paths"
    save_and_smooth_path(raw_path, output_directory)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
