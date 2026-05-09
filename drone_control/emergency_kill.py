#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand
import threading
import sys

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
        self.get_logger().warn('*** PRESS ENTER IN THIS TERMINAL AT ANY TIME TO KILL MOTORS AND LAND ***')

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

    def trigger_kill(self):
        self.get_logger().error('KILL SWITCH ACTIVATED! SENDING DISARM AND LAND COMMANDS...')
        # Send disarm command
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        # Send land command just in case
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().error('COMMANDS SENT.')

def input_thread(node):
    while rclpy.ok():
        # Wait for user to press Enter
        sys.stdin.readline()
        node.trigger_kill()

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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
