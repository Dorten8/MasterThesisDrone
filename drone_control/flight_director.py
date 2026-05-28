#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, BatteryStatus
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from std_msgs.msg import String

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
from flight_recorder import FlightRecorder
import importlib
import inspect
from missions.base_mission import BaseMission

class StaticPose:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class FlightDirector(Node):
    def __init__(self, mission_class, cage_mode="rotating_cage"):
        super().__init__('flight_director')
        self.cage_mode = cage_mode

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
        
        # Event publisher for automated loop pass segmentation
        self.waypoint_status_pub = self.create_publisher(String, '/flight_director/active_waypoint', 10)

        # Subscribers
        self.ekf_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._ekf_cb, sensor_qos)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self._status_cb, sensor_qos)
        self.battery_sub = self.create_subscription(BatteryStatus, '/fmu/out/battery_status', self._battery_cb, sensor_qos)
        self.mocap_sub = self.create_subscription(NamedPoseArray, '/poses', self._mocap_cb, sensor_qos)

        # State Variables
        self.ekf_pos = None
        self.mocap_pos = None
        self.vehicle_status = None
        
        self.transform_offset = (0.0, 0.0, 0.0) # Delta = EKF2 - Mocap
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
        
        # Geofence Bounds (Absolute MoCap Space: Default 2.5x2.5m room, 2m height limit)
        self.geo_x_min = -2.5
        self.geo_x_max = 2.5
        self.geo_y_min = -2.5
        self.geo_y_max = 2.5
        self.geo_z_min = 0.05
        self.geo_z_max = 2.0
        
        # Load geofence limits dynamically from config/drone_config.json if calibrated
        config_path = os.path.join(os.path.dirname(script_dir), "config", "drone_config.json")
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    config = json.load(f)
                if "mocap_geofence" in config:
                    gf = config["mocap_geofence"]
                    self.geo_x_min = gf["x_min"]
                    self.geo_x_max = gf["x_max"]
                    self.geo_y_min = gf["y_min"]
                    self.geo_y_max = gf["y_max"]
                    self.geo_z_min = gf["z_min"]
                    self.geo_z_max = gf["z_max"]
                    print(f"[SYSTEM] Loaded geofence limits from drone_config.json:")
                    print(f"         X: {self.geo_x_min:.2f}m to {self.geo_x_max:.2f}m")
                    print(f"         Y: {self.geo_y_min:.2f}m to {self.geo_y_max:.2f}m")
                    print(f"         Z: {self.geo_z_min:.2f}m to {self.geo_z_max:.2f}m")
                else:
                    print("[WARN] No 'mocap_geofence' found in drone_config.json. Using defaults.")
            except Exception as e:
                print(f"[WARN] Failed to load geofence limits: {e}. Using defaults.")
        else:
            print("[WARN] config/drone_config.json not found! Using defaults.")
        
        self.state = "INIT"  # INIT -> ALIGNED -> ARMED -> TAKEOFF -> MISSION -> LANDING
        self.column_pos = None  # Live rigid body location tracked from MoCap

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
                
                # Start automatic bag recording the moment we arm with cage configuration suffix
                self.recorder.start_recording(self.mission.MISSION_NAME + "_" + self.cage_mode)

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
            elif named_pose.name == "jake_column_drone":
                self.column_pos = named_pose.pose.position

    def _check_geofence(self, pos):
        if self.aborted: return
        # Only enforce runtime geofence checking during active flight missions
        if self.state != "MISSION": return

        # Assuming ENU from /poses, Z is positive UP
        if not (self.geo_x_min <= pos.x <= self.geo_x_max) or not (self.geo_y_min <= pos.y <= self.geo_y_max) or not (self.geo_z_min <= pos.z <= self.geo_z_max):
            print(f"\n\033[91m[FATAL] GEOFENCE BREACH: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}! LANDING!\033[0m")
            print(f"         Allowed bounds -> X: [{self.geo_x_min:.2f}, {self.geo_x_max:.2f}], Y: [{self.geo_y_min:.2f}, {self.geo_y_max:.2f}], Z: [{self.geo_z_min:.2f}, {self.geo_z_max:.2f}]")
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
        # Start automatic bag recording the moment we command arm with cage configuration suffix
        self.recorder.start_recording(self.mission.MISSION_NAME + "_" + self.cage_mode)
        # Official PX4 offboard arming uses param1=1.0 and param2=21196.0 for offboard override force arm.
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0, param2=21196.0)

    def command_offboard(self):
        self.get_logger().info("Commanding OFFBOARD Mode...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def command_land(self):
        self.get_logger().warn("Commanding Controlled Offboard LAND...")
        self.state = "LANDING"
        self._publish_active_waypoint("LANDING")
        
        # Initialize offboard landing coordinates at current target or actual position
        if self.smoothed_target_enu is not None:
            self.landing_target_x = self.smoothed_target_enu[0]
            self.landing_target_y = self.smoothed_target_enu[1]
            self.landing_target_z = self.smoothed_target_enu[2]
            self.landing_target_yaw = self.smoothed_target_enu[3]
        elif self.mocap_pos is not None:
            self.landing_target_x = self.mocap_pos.x
            self.landing_target_y = self.mocap_pos.y
            self.landing_target_z = self.mocap_pos.z
            self.landing_target_yaw = 0.0
        else:
            self.landing_target_x = 0.0
            self.landing_target_y = 0.0
            self.landing_target_z = 0.5
            self.landing_target_yaw = 0.0
            
        self.landing_start_time = time.time()

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

        # Publish heartbeat (position control with velocity feedforward)
        hb = OffboardControlMode()
        hb.position = True
        hb.velocity = True
        hb.acceleration = False
        hb.attitude = False
        hb.body_rate = False
        hb.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(hb)

        if self.state == "LANDING":
            elapsed_landing = now - getattr(self, 'landing_start_time', now)
            
            # Ground detection: if MoCap Z is below 8cm, or we have been landing for > 6 seconds
            if (self.mocap_pos is not None and self.mocap_pos.z < 0.08) or elapsed_landing > 6.0:
                self.get_logger().warn("[SYSTEM] Ground detected or landing timeout reached. Disarming motors.")
                # Publish normal disarm, and fallback to hard disarm
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
                # Stop recording cleanly
                self.recorder.stop_recording_delayed(1.0)
                # Send EKF setpoint to maintain offboard mode clean exit
                self._send_ekf_setpoint()
                return
                
            # Descend slowly at 0.15 m/s while holding horizontal position
            self.landing_target_z = max(0.0, self.landing_target_z - 0.15 * dt)
            
            self._send_mocap_setpoint(
                self.landing_target_x, 
                self.landing_target_y, 
                self.landing_target_z, 
                self.landing_target_yaw,
                vx_enu=0.0, 
                vy_enu=0.0, 
                vz_enu=-0.15
            )
            return

        if self.aborted:
            return

        if self.state == "INIT":
            if self.ekf_pos is not None and self.mocap_pos is not None:
                if not hasattr(self, 'alignment_samples'):
                    self.alignment_samples = []
                
                # Collect relative offset sample (Mapped to match mocap_px4_bridge's custom transform)
                m_ned_x = self.mocap_pos.x
                m_ned_y = -self.mocap_pos.y
                m_ned_z = -self.mocap_pos.z
                
                dx = self.ekf_pos.x - m_ned_x
                dy = self.ekf_pos.y - m_ned_y
                dz = self.ekf_pos.z - m_ned_z
                
                self.alignment_samples.append((dx, dy, dz, self.mocap_pos.x, self.mocap_pos.y, self.mocap_pos.z))
                
                # Maintain sliding window of 20 samples (2 seconds at 10Hz)
                if len(self.alignment_samples) > 20:
                    self.alignment_samples.pop(0)
                
                # Verify EKF2 convergence by checking standard deviation of samples
                if len(self.alignment_samples) == 20:
                    dx_vals = [s[0] for s in self.alignment_samples]
                    dy_vals = [s[1] for s in self.alignment_samples]
                    dz_vals = [s[2] for s in self.alignment_samples]
                    
                    avg_dx = sum(dx_vals) / 20.0
                    avg_dy = sum(dy_vals) / 20.0
                    avg_dz = sum(dz_vals) / 20.0
                    
                    var_dx = sum((x - avg_dx)**2 for x in dx_vals) / 20.0
                    var_dy = sum((y - avg_dy)**2 for y in dy_vals) / 20.0
                    
                    std_dx = math.sqrt(var_dx)
                    std_dy = math.sqrt(var_dy)
                    
                    # Print EKF2 convergence status in real-time
                    print(f"\r[SYSTEM] EKF2 state estimator converging... (std_x: {std_dx*1000:.1f}mm, std_y: {std_dy*1000:.1f}mm) [Goal < 3.0mm] ", end="", flush=True)
                    
                    # Transition to ALIGNED only when state estimate is settled (std < 3.0mm)
                    if std_dx < 0.003 and std_dy < 0.003:
                        avg_m_x = sum(s[3] for s in self.alignment_samples) / 20.0
                        avg_m_y = sum(s[4] for s in self.alignment_samples) / 20.0
                        avg_m_z = sum(s[5] for s in self.alignment_samples) / 20.0
                        
                        self.transform_offset = (avg_dx, avg_dy, avg_dz)
                        self.mocap_at_takeoff = StaticPose(avg_m_x, avg_m_y, avg_m_z)
                        self.smoothed_target_enu = [avg_m_x, avg_m_y, avg_m_z, 0.0]
                        
                        # Pre-flight geofence check (simulate start to extract waypoints)
                        self.mission.on_start(self.mocap_at_takeoff)
                        geofence_passed = True
                        if hasattr(self.mission, 'ENFORCE_GEOFENCE') and self.mission.ENFORCE_GEOFENCE:
                            print("\n[SYSTEM] Verifying Mission Geofence boundaries...", flush=True)
                            waypoints = self.mission.get_all_absolute_waypoints()
                            for i, wp in enumerate(waypoints):
                                x, y, z = wp[0], wp[1], wp[2]
                                if not (self.geo_x_min <= x <= self.geo_x_max) or not (self.geo_y_min <= y <= self.geo_y_max) or not (self.geo_z_min <= z <= self.geo_z_max):
                                    print(f"\n\033[91m[FATAL] GEOFENCE PRE-CHECK FAILED! Waypoint {i} ({x:.2f}, {y:.2f}, {z:.2f}) violates geofence bounds!\033[0m", flush=True)
                                    print(f"         Allowed bounds -> X: [{self.geo_x_min:.2f}, {self.geo_x_max:.2f}], Y: [{self.geo_y_min:.2f}, {self.geo_y_max:.2f}], Z: [{self.geo_z_min:.2f}, {self.geo_z_max:.2f}]", flush=True)
                                    print("\033[91m[SYSTEM] Arming BLOCKED. Refusing to start flight with unsafe parameters.\033[0m\n", flush=True)
                                    geofence_passed = False
                                    break
                        
                        if not geofence_passed:
                            self.state = "GEOFENCE_BLOCKED"
                            print("\n" + "="*45, flush=True)
                            print("🕹️  FLIGHT DIRECTOR LOCKED (GEOFENCE BLOCKED)", flush=True)
                            print("="*45, flush=True)
                            print(" [ SPACE ] -> KILL MOTORS / RESET", flush=True)
                            print("="*45 + "\n", flush=True)
                            return
                        
                        self.state = "ALIGNED"
                        print(f"\n\n[SYSTEM] EKF2 Estimator Fully Converged! State Settled.")
                        print(f"[SYSTEM] Frame Transform Locked. EKF2 Ned Offset: X: {avg_dx:.3f}m | Y: {avg_dy:.3f}m | Z: {avg_dz:.3f}m")
                        print("[SYSTEM] Geofence check PASSED.")
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
                self._publish_active_waypoint("FINISHED")
                self.command_land()
                return

            if self.mission.is_paused:
                # If paused, hold the last smoothed target
                self._send_mocap_setpoint(*self.smoothed_target_enu)
                return

            # Keep track of previous state to publish transitions
            prev_wp_idx = self.mission.current_wp_idx
            prev_loop = self.mission.loop_counter

            sp = self.mission.get_next_setpoint(self.mocap_pos, dt)

            # Check if waypoint index or loop counter changed
            curr_wp_idx = self.mission.current_wp_idx
            curr_loop = self.mission.loop_counter

            # Dynamic label assignment based on active mission coordinates
            if curr_wp_idx < len(self.mission.waypoints):
                target = self.mission.waypoints[curr_wp_idx]
                target_x, target_y = target[0], target[1]
                
                if hasattr(self.mission, 'exp_sp') and abs(target_x - self.mission.exp_sp[0]) < 0.01 and abs(target_y - self.mission.exp_sp[1]) < 0.01:
                    label = "EXP_START_POINT"
                elif hasattr(self.mission, 'exp_ep') and abs(target_x - self.mission.exp_ep[0]) < 0.01 and abs(target_y - self.mission.exp_ep[1]) < 0.01:
                    label = "EXP_END_POINT"
                else:
                    label = "OTHER"
            else:
                label = "FINISHED"

            if curr_wp_idx != prev_wp_idx or curr_loop != prev_loop or not hasattr(self, '_last_published_wp') or self._last_published_wp != (curr_loop, label):
                event_str = f"LOOP_{curr_loop}_{label}"
                self._publish_active_waypoint(event_str)
                self._last_published_wp = (curr_loop, label)

            if sp:
                # sp format: (x, y, z, face_forward_bool, [speed_mps])
                target_x, target_y, target_z, face_forward = sp[0], sp[1], sp[2], sp[3]
                
                # Check for explicit speed override, otherwise fallback to default
                active_speed = sp[4] if len(sp) >= 5 else self.max_speed_mps
                max_step = active_speed * dt
                
                dx = target_x - self.smoothed_target_enu[0]
                dy = target_y - self.smoothed_target_enu[1]
                dz = target_z - self.smoothed_target_enu[2]
                
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                if dist <= max_step:
                    self.smoothed_target_enu[0] = target_x
                    self.smoothed_target_enu[1] = target_y
                    self.smoothed_target_enu[2] = target_z
                    # Arrived: zero feedforward velocity to hold position
                    vx_ff, vy_ff, vz_ff = 0.0, 0.0, 0.0
                else:
                    unit_x = dx / dist
                    unit_y = dy / dist
                    unit_z = dz / dist
                    self.smoothed_target_enu[0] += unit_x * max_step
                    self.smoothed_target_enu[1] += unit_y * max_step
                    self.smoothed_target_enu[2] += unit_z * max_step
                    # Feedforward velocity: tells PX4 the desired velocity, not just position
                    vx_ff = unit_x * active_speed
                    vy_ff = unit_y * active_speed
                    vz_ff = unit_z * active_speed
                    
                # Auto-yaw computation if face_forward is true
                if face_forward and dist > 0.05: # Only yaw if we have meaningful travel vector
                    # PX4's Z-axis is Down, so positive yaw is clockwise. 
                    # math.atan2 gives counter-clockwise angles, so we negate it.
                    self.smoothed_target_enu[3] = -math.atan2(dy, dx)
                
                self._send_mocap_setpoint(*self.smoothed_target_enu, vx_enu=vx_ff, vy_enu=vy_ff, vz_enu=vz_ff)

    def _send_mocap_setpoint(self, x_enu, y_enu, z_enu, yaw, vx_enu=0.0, vy_enu=0.0, vz_enu=0.0):
        """ Translates an absolute MoCap ENU setpoint into the PX4 EKF2 NED frame """
        m_ned_x = x_enu
        m_ned_y = -y_enu
        m_ned_z = -z_enu

        # Use the statically locked offset calculated during the INIT state.
        # This prevents EKF2 estimation latency and time-lag from introducing dynamic feedback
        # distortion as a function of the drone's flight velocity.
        msg = TrajectorySetpoint()
        msg.position = [
            m_ned_x + self.transform_offset[0],
            m_ned_y + self.transform_offset[1],
            m_ned_z + self.transform_offset[2]
        ]
        # Velocity feedforward in NED (ENU -> NED: y and z negated)
        # This tells PX4 how fast to move, not just where to go
        msg.velocity = [vx_enu, -vy_enu, -vz_enu]
        
        # Explicitly set unused derivatives to NaN per standard PX4 implementation
        # This prevents PX4 from forcing 0.0 acceleration, which causes massive integrator windup
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.yawspeed = float('nan')
        
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def _send_ekf_setpoint(self):
        """ Sends the drone's actual current EKF2 position to guarantee 0.0 tracking error during ground standby """
        if self.ekf_pos is not None:
            msg = TrajectorySetpoint()
            msg.position = [self.ekf_pos.x, self.ekf_pos.y, self.ekf_pos.z]
            
            # Explicitly set all unused derivatives to NaN per standard PX4 implementation
            msg.velocity = [float('nan'), float('nan'), float('nan')]
            msg.acceleration = [float('nan'), float('nan'), float('nan')]
            msg.jerk = [float('nan'), float('nan'), float('nan')]
            msg.yawspeed = float('nan')
            
            msg.yaw = 0.0
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_pub.publish(msg)

    def trigger_takeoff(self):
        if self.state != "ARMED" and self.state != "ARMING":
            print("[WARN] Must ARM first before Takeoff!")
            return
            
        print("[SYSTEM] Engaging Offboard and Starting Mission...")
        self.command_offboard()
        self.state = "MISSION"

    def destroy_node(self):
        self.recorder.stop_recording()
        super().destroy_node()

    def _publish_active_waypoint(self, status_str):
        msg = String()
        msg.data = status_str
        self.waypoint_status_pub.publish(msg)
        self.get_logger().info(f"[EVENT] Active Waypoint status: {status_str}")

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
                # Dynamically set takeoff coordinates to the drone's actual physical position at this moment
                current_m_x = node.mocap_pos.x
                current_m_y = node.mocap_pos.y
                current_m_z = node.mocap_pos.z
                
                node.mocap_at_takeoff = StaticPose(current_m_x, current_m_y, current_m_z)
                node.smoothed_target_enu = [current_m_x, current_m_y, current_m_z, 0.0]
                
                print(f"\n[SYSTEM] Locking Takeoff Position: X={current_m_x:.3f}m, Y={current_m_y:.3f}m, Z={current_m_z:.3f}m", flush=True)
                
                # Re-initialize the mission waypoints based on this new takeoff position
                node.mission.on_start(node.mocap_at_takeoff)
                
                # Re-verify the safety geofence limits with the dynamic waypoints
                geofence_passed = True
                if hasattr(node.mission, 'ENFORCE_GEOFENCE') and node.mission.ENFORCE_GEOFENCE:
                    print("[SYSTEM] Verifying Dynamic Mission Geofence boundaries...", flush=True)
                    waypoints = node.mission.get_all_absolute_waypoints()
                    for i, wp in enumerate(waypoints):
                        x, y, z = wp[0], wp[1], wp[2]
                        if not (node.geo_x_min <= x <= node.geo_x_max) or not (node.geo_y_min <= y <= node.geo_y_max) or not (node.geo_z_min <= z <= node.geo_z_max):
                            print(f"\n\033[91m[FATAL] GEOFENCE PRE-CHECK FAILED! Dynamic Waypoint {i} ({x:.2f}, {y:.2f}, {z:.2f}) violates geofence bounds!\033[0m", flush=True)
                            print(f"         Allowed bounds -> X: [{node.geo_x_min:.2f}, {node.geo_x_max:.2f}], Y: [{node.geo_y_min:.2f}, {node.geo_y_max:.2f}], Z: [{node.geo_z_min:.2f}, {node.geo_z_max:.2f}]", flush=True)
                            print("\033[91m[SYSTEM] Arming BLOCKED. Refusing to start flight with unsafe parameters.\033[0m\n", flush=True)
                            geofence_passed = False
                            break
                
                if not geofence_passed:
                    node.state = "GEOFENCE_BLOCKED"
                    print("\n" + "="*45, flush=True)
                    print("🕹️  FLIGHT DIRECTOR LOCKED (GEOFENCE BLOCKED)", flush=True)
                    print("="*45, flush=True)
                    print(" [ SPACE ] -> KILL MOTORS / RESET", flush=True)
                    print("="*45 + "\n", flush=True)
                    continue

                print("\n[SYSTEM] Initiating Arming sequence (Switching to Offboard, then Arming)...", flush=True)
                node.state = "ARMING"
                node.arming_counter = 0
            elif node.state == "GEOFENCE_BLOCKED":
                print("\n\033[91m[FATAL] CANNOT ARM! Mission waypoints violate geofence limits.\033[0m", flush=True)
        elif ch == 't' or ch == 'T':
            node.trigger_takeoff()
        elif ch == '\r' or ch == '\n':
            if node.state == "MISSION":
                node.mission.toggle_pause()
        elif ch == 'l' or ch == 'L':
            node.command_land()
        elif ch == ' ':
            node.command_kill()
        elif ch == '\x03': # Ctrl+C
            node.command_land()
            break

# --- Mission Loader ---
def load_mission():
    missions_dir = os.path.join(script_dir, "missions")
    available_missions = []
    
    # Auto-discover mission classes
    for filename in sorted(os.listdir(missions_dir)):
        if filename.endswith(".py") and filename not in ["__init__.py", "base_mission.py"]:
            module_name = f"missions.{filename[:-3]}"
            try:
                module = importlib.import_module(module_name)
                for name, obj in inspect.getmembers(module, inspect.isclass):
                    # Check if it inherits from BaseMission but is not BaseMission itself
                    if obj.__module__ == module_name and hasattr(obj, 'MISSION_NAME'):
                        available_missions.append((filename, obj))
            except Exception as e:
                print(f"[WARN] Failed to load {filename}: {e}")
                
    if not available_missions:
        print("[FATAL] No missions found in the missions directory!")
        sys.exit(1)

    print("\n--- Available Missions ---", flush=True)
    for idx, (filename, cls) in enumerate(available_missions, 1):
        desc = getattr(cls, 'MISSION_DESCRIPTION', 'No description')
        print(f" [{idx}] {cls.MISSION_NAME} ({filename})\n     -> {desc}", flush=True)
    
    choice = input("\nSelect mission to load [1-{}]: ".format(len(available_missions)))
    choice = choice.strip()
    
    try:
        idx = int(choice) - 1
        if idx < 0 or idx >= len(available_missions):
            raise ValueError()
    except ValueError:
        print("Invalid choice.", flush=True)
        sys.exit(1)
        
    filename, mission_class = available_missions[idx]
    print(f"\n[SYSTEM] Loaded Mission: {mission_class.MISSION_NAME}", flush=True)
    return mission_class

def main(args=None):
    mission_class = load_mission()

    # Prompt user for cage configuration before starting the flight node
    print("\n=============================================")
    print("🛡️ CAGE CONFIGURATION CALIBRATION")
    print("=============================================")
    cage_choice = input("Is the drone safety cage ROTATING or FIXED for this flight? [R/F] (Default is R): ").strip().upper()
    if cage_choice == 'F' or "FIXED" in cage_choice:
        cage_mode = "fixed_cage"
    else:
        cage_mode = "rotating_cage"
    print(f"[SYSTEM] Calibrated cage profile: {cage_mode.upper().replace('_', ' ')}")
    print("=============================================\n")

    rclpy.init(args=args)
    node = FlightDirector(mission_class, cage_mode=cage_mode)
    
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
