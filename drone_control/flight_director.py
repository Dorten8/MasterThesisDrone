#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, BatteryStatus
from motion_capture_tracking_interfaces.msg import NamedPoseArray

import sys
import json
import time
import termios
import tty
import threading
import os
import math
import subprocess
import signal

# Ensure script directory is in Python path for reliable imports
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

from flight_recorder import FlightRecorder
from missions.hover_test import HoverTest

# Static Mission Registry
MISSIONS = {
    "1": ("hover_test", HoverTest)
}

class FlightDirector(Node):
    def __init__(self, mission_class):
        super().__init__('flight_director')

        # Load Mission
        self.mission = mission_class()

        # QoS Profiles
        # PX4 input subscriptions expect TRANSIENT_LOCAL durability
        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # MoCap and PX4 output topics require VOLATILE durability for compatibility
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', command_qos)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', command_qos)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', command_qos)

        # Subscribers
        self.ekf_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._ekf_cb, sensor_qos)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self._status_cb, sensor_qos)
        self.battery_sub = self.create_subscription(BatteryStatus, '/fmu/out/battery_status', self._battery_cb, sensor_qos)
        self.mocap_sub = self.create_subscription(NamedPoseArray, '/poses', self._mocap_cb, sensor_qos)

        # State Variables
        self.ekf_pos = None
        self.mocap_pos = None
        self.vehicle_status = None
        
        self.transform_offset = None # Delta = EKF2 - Mocap
        self.mocap_at_takeoff = None
        
        self.arming_counter = 0
        
        # Trajectory Smoother State
        self.smoothed_target_enu = None
        self.max_speed_mps = 0.2  # Default max speed: 0.2 meters per second
        
        self.armed = False
        self.in_air = False
        self.aborted = False
        
        # Flight Recorder Instance
        self.recorder = FlightRecorder()

        self.last_tick = time.time()
        
        # Geofence Bounds (Absolute MoCap Space: 3x3m room, 2m height limit)
        # Z is negative UP in ENU/NED representations typically. 
        # Since MoCap /poses provides ENU (Z is positive UP), wait, let's check:
        # We'll extract raw XYZ from MoCap and enforce bounds.
        self.geo_x_max = 2.5
        self.geo_y_max = 2.5
        self.geo_z_max = 2.0  # If MoCap Z is positive UP
        
        self.state = "INIT"  # INIT -> ALIGNED -> ARMED -> TAKEOFF -> MISSION -> LANDING

        # 10Hz Timer
        self.timer = self.create_timer(0.1, self._control_loop)
        
        # Read drone name from config if possible (fallback "jake_drone_frame_01")
        self.drone_name = "jake_drone_frame_01" 
        try:
            config_path = os.path.join(os.path.dirname(script_dir), "config", "drone_config.json")
            with open(config_path, 'r') as f:
                config = json.load(f)
                for body in config.get("tracked_bodies", []):
                    if body.get("role") == "primary":
                        self.drone_name = body.get("name", self.drone_name)
                        break
        except Exception as e:
            self.get_logger().warn(f"Could not read drone name from config, using fallback: {self.drone_name}. Error: {e}") 

    def _ekf_cb(self, msg):
        self.ekf_pos = msg

    def _status_cb(self, msg):
        self.vehicle_status = msg
        was_armed = self.armed
        # Hardcoded 2 represents ARMING_STATE_ARMED in PX4. 
        # Using 2 directly avoids silent rclpy AttributeError crashes if the constant is not defined on the class.
        self.armed = (msg.arming_state == 2)
        
        # Auto-stop recording upon disarming after landing
        if was_armed and not self.armed:
            print("\n[SYSTEM] Drone Disarmed! Stopping recorder...", flush=True)
            self.recorder.stop_recording()
        elif not was_armed and self.armed:
            if self.state == "ARMING":
                self.state = "ARMED"
                print("\n[SYSTEM] Drone successfully ARMED and in OFFBOARD mode! Ready for Takeoff [T].", flush=True)
                
                # Start automatic bag recording the moment we arm
                self.recorder.start_recording()

    def _battery_cb(self, msg):
        if self.aborted: return
        raw_voltage = msg.voltage_v
        pct = msg.remaining * 100.0
        
        # Initialize filter on first message
        if not hasattr(self, 'filtered_voltage'):
            self.filtered_voltage = raw_voltage
            print(f"\n[SYSTEM] Initial Battery: {pct:.1f}% ({raw_voltage:.2f}V)")
        else:
            # High-damping Low-Pass Filter (Alpha = 0.02) to absorb 5-10 second motor takeoff voltage sags
            self.filtered_voltage = 0.98 * self.filtered_voltage + 0.02 * raw_voltage
            
        # Periodic status update every 5 seconds
        now = time.time()
        if not hasattr(self, 'last_battery_print_time'):
            self.last_battery_print_time = now
        elif now - self.last_battery_print_time >= 5.0:
            self.last_battery_print_time = now
            print(f"\r[STATUS] Battery: {pct:.1f}% ({self.filtered_voltage:.2f}V)   ", end="", flush=True)
            
        # For a 6S LiPo: 
        # - Full: 25.2V (4.2V/cell)
        # - Warning: 21.6V (3.6V/cell)
        # - Critical Land: 20.4V (3.4V/cell)
        if self.filtered_voltage < 20.4:
            print(f"\n\033[91m[FATAL] Critical Battery Voltage: {self.filtered_voltage:.2f}V! Triggering Emergency LAND!\033[0m")
            self.command_land()
            self.aborted = True
        elif self.filtered_voltage < 21.6:
            # Red warning for low battery
            print(f"\r\033[91m[WARN] Low Battery Voltage: {self.filtered_voltage:.2f}V ({pct:.1f}%)   \033[0m", end="", flush=True)

    def _mocap_cb(self, msg):
        for named_pose in msg.poses:
            if named_pose.name == self.drone_name:
                self.mocap_pos = named_pose.pose.position
                self._check_geofence(self.mocap_pos)
                break

    def _check_geofence(self, pos):
        if self.aborted: return
        # Assuming ENU from /poses, Z is positive UP
        if abs(pos.x) > self.geo_x_max or abs(pos.y) > self.geo_y_max or pos.z > self.geo_z_max:
            print(f"\n\033[91m[FATAL] GEOFENCE BREACH: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}! LANDING!\033[0m")
            self.command_land()
            self.aborted = True

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def command_arm(self):
        self.get_logger().info("Commanding ARM...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0, param2=21196.0)

    def command_offboard(self):
        self.get_logger().info("Commanding OFFBOARD Mode...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def command_land(self):
        self.get_logger().warn("Commanding LAND...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.state = "LANDING"
        # We do NOT stop recording here; it will stop automatically when EKF2 reports disarm!

    def command_kill(self):
        self.get_logger().fatal("BRUTAL KILL TRIGGERED! Motors off!")
        # Burst publish 5 times with 10ms spacing to guarantee delivery under transient packet drops
        for _ in range(5):
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0, param2=21196.0)
            time.sleep(0.01)
        self.state = "LANDING"
        # Delayed recording stop by 3 seconds to capture descent/impact
        self.recorder.stop_recording_delayed(3.0)

    def _control_loop(self):
        now = time.time()
        dt = now - self.last_tick
        self.last_tick = now

        # Publish heartbeat (position control)
        hb = OffboardControlMode()
        hb.position = True
        hb.velocity = False
        hb.acceleration = False
        hb.attitude = False
        hb.body_rate = False
        hb.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(hb)

        if self.aborted:
            return
            
        if self.state == "LANDING":
            # Continue sending drone's actual EKF2 setpoint to keep the Offboard link healthy and trusted by PX4.
            # If the offboard link drops due to missing setpoints, PX4 enters failsafe and ignores MAVLink disarm commands!
            self._send_ekf_setpoint()
            return

        if self.state == "INIT":
            if self.ekf_pos is not None and self.mocap_pos is not None:
                if not hasattr(self, 'alignment_samples'):
                    self.alignment_samples = []
                
                # Collect relative offset sample
                m_ned_x = self.mocap_pos.y
                m_ned_y = self.mocap_pos.x
                m_ned_z = -self.mocap_pos.z
                
                dx = self.ekf_pos.x - m_ned_x
                dy = self.ekf_pos.y - m_ned_y
                dz = self.ekf_pos.z - m_ned_z
                
                self.alignment_samples.append((dx, dy, dz, self.mocap_pos.x, self.mocap_pos.y, self.mocap_pos.z))
                
                # Wait for 10 stable samples to filter out startup glitches or EKF2 convergence spikes
                if len(self.alignment_samples) >= 10:
                    avg_dx = sum(s[0] for s in self.alignment_samples) / 10.0
                    avg_dy = sum(s[1] for s in self.alignment_samples) / 10.0
                    avg_dz = sum(s[2] for s in self.alignment_samples) / 10.0
                    
                    avg_m_x = sum(s[3] for s in self.alignment_samples) / 10.0
                    avg_m_y = sum(s[4] for s in self.alignment_samples) / 10.0
                    avg_m_z = sum(s[5] for s in self.alignment_samples) / 10.0
                    
                    self.transform_offset = (avg_dx, avg_dy, avg_dz)
                    
                    # Define a static, immutable pose object to avoid Python reference updates
                    class StaticPose:
                        def __init__(self, x, y, z):
                            self.x = x
                            self.y = y
                            self.z = z
                    self.mocap_at_takeoff = StaticPose(avg_m_x, avg_m_y, avg_m_z)
                    self.smoothed_target_enu = [avg_m_x, avg_m_y, avg_m_z, 0.0]
                    
                    self.state = "ALIGNED"
                    print(f"\n[SYSTEM] Frame Transform Aligned! EKF2 Z-Offset: {avg_dz:.2f}m (Averaged over 10 samples)")
                    print("\n" + "="*45)
                    print("🕹️  FLIGHT DIRECTOR ACTIVE")
                    print("="*45)
                    print(" [ A ]     -> ARM the drone")
                    print(" [ T ]     -> TAKE OFF (Start Mission)")
                    print(" [ENTER]   -> PROCEED (Next Phase)")
                    print(" [ L ]     -> SAFE LAND")
                    print(" [SPACE]   -> KILL MOTORS")
                    print("="*45 + "\n")
            return

        if self.state == "ALIGNED" or self.state == "ARMED":
            # Send drone's actual EKF2 position to guarantee PX4 accepts Offboard Mode with zero error
            self._send_ekf_setpoint()
            return

        if self.state == "ARMING":
            # Send drone's actual EKF2 position to guarantee PX4 accepts Offboard Mode with zero error
            self._send_ekf_setpoint()
            
            # Step 10: Switch to Offboard Mode
            if self.arming_counter == 10:
                self.command_offboard()
                
            # Step 30: Command Arm
            if self.arming_counter == 30:
                self.command_arm()
                
            if self.arming_counter < 31:
                self.arming_counter += 1
            return

        if self.state == "MISSION":
            if self.mission.is_finished:
                self.command_land()
                return

            sp = self.mission.get_next_setpoint(self.mocap_pos, dt)
            if sp:
                # Interpolate smoothed_target towards requested sp
                # sp format: (x, y, z, yaw)
                max_step = self.max_speed_mps * dt
                
                dx = sp[0] - self.smoothed_target_enu[0]
                dy = sp[1] - self.smoothed_target_enu[1]
                dz = sp[2] - self.smoothed_target_enu[2]
                
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                if dist <= max_step:
                    self.smoothed_target_enu[0] = sp[0]
                    self.smoothed_target_enu[1] = sp[1]
                    self.smoothed_target_enu[2] = sp[2]
                else:
                    self.smoothed_target_enu[0] += (dx / dist) * max_step
                    self.smoothed_target_enu[1] += (dy / dist) * max_step
                    self.smoothed_target_enu[2] += (dz / dist) * max_step
                    
                self.smoothed_target_enu[3] = sp[3] # Snap yaw for now
                
                self._send_mocap_setpoint(*self.smoothed_target_enu)

    def _send_mocap_setpoint(self, x_enu, y_enu, z_enu, yaw):
        """ Translates an absolute MoCap ENU setpoint into the PX4 EKF2 NED frame """
        m_ned_x = y_enu
        m_ned_y = x_enu
        m_ned_z = -z_enu

        msg = TrajectorySetpoint()
        msg.position = [
            m_ned_x + self.transform_offset[0],
            m_ned_y + self.transform_offset[1],
            m_ned_z + self.transform_offset[2]
        ]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def _send_ekf_setpoint(self):
        """ Sends the drone's actual current EKF2 position to guarantee 0.0 tracking error during ground standby """
        if self.ekf_pos is not None:
            msg = TrajectorySetpoint()
            msg.position = [self.ekf_pos.x, self.ekf_pos.y, self.ekf_pos.z]
            msg.yaw = 0.0
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_pub.publish(msg)

    def trigger_takeoff(self):
        if self.state != "ARMED" and self.state != "ARMING":
            print("[WARN] Must ARM first before Takeoff!")
            return
        print("[SYSTEM] Engaging Offboard and Starting Mission...")
        self.command_offboard()
        self.mission.on_start(self.mocap_at_takeoff)
        self.state = "MISSION"

    def destroy_node(self):
        self.recorder.stop_recording()
        super().destroy_node()

# --- Keyboard Thread ---
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def input_thread(node):
    while rclpy.ok():
        ch = getch()
        if ch == 'a' or ch == 'A':
            if node.state == "ALIGNED":
                print("\n[SYSTEM] Initiating Arming sequence (Switching to Offboard, then Arming)...", flush=True)
                node.state = "ARMING"
                node.arming_counter = 0
        elif ch == 't' or ch == 'T':
            node.trigger_takeoff()
        elif ch == '\r' or ch == '\n':
            if node.state == "MISSION":
                node.mission.on_proceed()
        elif ch == 'l' or ch == 'L':
            node.command_land()
        elif ch == ' ':
            node.command_kill()
        elif ch == '\x03': # Ctrl+C
            node.command_land()
            break

# --- Mission Loader ---
def load_mission():
    print("\n--- Available Missions ---", flush=True)
    for key, (name, _) in MISSIONS.items():
        print(f" [{key}] {name}", flush=True)
    
    choice = input("\nSelect mission to load [1-{}]: ".format(len(MISSIONS)))
    choice = choice.strip()
    
    if choice not in MISSIONS:
        print("Invalid choice.", flush=True)
        sys.exit(1)
        
    name, mission_class = MISSIONS[choice]
    print(f"\n[SYSTEM] Loaded Mission: {mission_class.__name__}", flush=True)
    return mission_class

def main(args=None):
    mission_class = load_mission()

    rclpy.init(args=args)
    node = FlightDirector(mission_class)
    
    thread = threading.Thread(target=input_thread, args=(node,), daemon=True)
    thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.command_land()
        
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
