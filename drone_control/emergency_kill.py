#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand
import threading
import sys
import tty
import termios

class EmergencyKillNode(Node):
    def __init__(self):
        super().__init__('emergency_kill_node')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
            
        self.get_logger().info('Emergency Kill Node Started!')
        self.get_logger().warn('*** HOTKEYS ACTIVE ***')
        self.get_logger().warn('PRESS [ENTER] -> GRACEFUL LAND')
        self.get_logger().warn('PRESS [SPACE] -> BRUTAL FORCE DISARM (DROP FROM SKY)')

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
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def trigger_force_kill(self):
        self.get_logger().fatal('SPACE PRESSED! BRUTAL FORCE DISARM TRIGGERED!')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0, param2=21196.0)

    def trigger_land(self):
        self.get_logger().error('ENTER PRESSED! COMMANDING SAFE LANDING...')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def input_thread(node):
    while rclpy.ok():
        ch = getch()
        if ch == ' ':
            node.trigger_force_kill()
        elif ch == '\r' or ch == '\n':
            node.trigger_land()
        elif ch == '\x03': # Ctrl+C
            break

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyKillNode()
    
    # Start the keyboard listener in a separate thread so it doesn't block ROS2 spinning
    thread = threading.Thread(target=input_thread, args=(node,), daemon=True)
    thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
