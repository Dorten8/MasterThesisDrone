#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition

from rclpy.qos import qos_profile_sensor_data

class TelemetryPrinter(Node):
    def __init__(self):
        super().__init__('telemetry_printer')
        
        # Subscribe to PX4's local position estimate using Sensor Data QoS
        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.listener_callback,
            qos_profile_sensor_data
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(
            f"PX4 Position (NED) -> X (Front): {msg.x:.3f} m, Y (Right): {msg.y:.3f} m, Z (Down): {msg.z:.3f} m",
            throttle_duration_sec=0.5
        )

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryPrinter()
    
    print("\n--- PX4 Telemetry Dry-Run Print Test ---")
    print("Waiting for /fmu/out/vehicle_local_position...")
    print("Pick up the drone. Z should go NEGATIVE as you lift it UP.\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
