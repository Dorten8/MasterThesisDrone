#!/usr/bin/env python3
"""
Jerk Test Script — Live ARM + Post-Analysis
==============================================
Usage:
  1. Run startup-sequence.sh first (Mocap + XRCE Agent must be running).
  2. Run this script inside the Docker container.
  3. It will establish an Offboard heartbeat, ARM for 3s, and DISARM.
  4. You perform the jerk test.
  5. The script auto-analyzes the latest bag file.
"""

import numpy as np
import datetime
import glob
import os
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    VehicleCommand, 
    VehicleStatus, 
    OffboardControlMode, 
    TrajectorySetpoint,
    VehicleLocalPosition
)

class ArmController(Node):
    def __init__(self):
        super().__init__('jerk_test_arm')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._pos_cb, qos_profile)
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self._status_cb, qos_profile)
        
        self.local_pos = None
        self.armed = False
        
        # State Machine Variables
        self.state = "WAITING_FOR_POS"
        self.heartbeat_counter = 0
        self.arm_start_time = 0.0
        self.test_complete = False
        
        # 10Hz Timer for state machine and heartbeat
        self.timer = self.create_timer(0.1, self._timer_cb)

    def _pos_cb(self, msg):
        self.local_pos = msg

    def _status_cb(self, msg):
        self.armed = (msg.arming_state == 2)

    def _send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def _timer_cb(self):
        # 1. ALWAYS PUBLISH HEARTBEAT
        ocm = OffboardControlMode()
        ocm.position = True
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        ocm.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(ocm)
        
        if self.local_pos:
            ts = TrajectorySetpoint()
            ts.position = [self.local_pos.x, self.local_pos.y, self.local_pos.z]
            ts.yaw = self.local_pos.heading
            ts.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_pub.publish(ts)

        # 2. STATE MACHINE LOGIC
        if self.state == "WAITING_FOR_POS":
            if self.local_pos is not None:
                print("📍 Local position received — Building heartbeat buffer...")
                self.state = "ESTABLISHING_HEARTBEAT"
                
        elif self.state == "ESTABLISHING_HEARTBEAT":
            self.heartbeat_counter += 1
            if self.heartbeat_counter >= 10:  # 1 full second of stable heartbeat
                print("🚀 Heartbeat established. Engaging OFFBOARD mode...")
                self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=6.0)
                print("🔒 Sending FORCE ARM command...")
                self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0, p2=21196.0)
                self.arm_start_time = time.time()
                self.state = "HOLDING_ARM"
                
        elif self.state == "HOLDING_ARM":
            elapsed = time.time() - self.arm_start_time
            if elapsed > 3.0:
                print("✅ 3 seconds elapsed. Disarming to save log...")
                self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=0.0)
                self.state = "CLEANUP"
                
        elif self.state == "CLEANUP":
            # Give it a fraction of a second to process the disarm before shutting down the node
            time.sleep(0.5)
            self.test_complete = True


def run_arm_phase():
    """Spins the node until the state machine finishes the arm/disarm cycle."""
    rclpy.init()
    node = ArmController()

    print("⏳ Waiting for Mocap position stream...")
    
    # Spin manually so we can exit cleanly once the test_complete flag is set
    while rclpy.ok() and not node.test_complete:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()
    print("✅ Arm Phase Node shut down cleanly. Proceeding to Jerk Test.")

# ── STEP 1: ARM BLIP ──
run_arm_phase()

# ── STEP 2: JERK TEST ──
# ... (Keep the rest of your original analysis code exactly the same below this line) ...