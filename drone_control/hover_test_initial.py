
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode, 
    TrajectorySetpoint, 
    VehicleCommand,
    VehicleLocalPosition, 
    VehicleStatus,
)
import threading
import sys
import tty
import termios

class OffboardControl(Node):
    """Unified node for controlling a vehicle and handling emergency kills."""

    def __init__(self) -> None:
        super().__init__('offboard_control')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        
        self.state = "WAITING_FOR_HOME"
        self.initial_pos = None
        self.current_setpoint_z = 0.0
        self.target_z = 0.0
        self.offboard_setpoint_counter = 0

        self.timer_ticks = 0

        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        if self.state == "WAITING_FOR_HOME":
            self.initial_pos = msg
            self.state = "READY"
            self.get_logger().info(f"Home position locked. Z={msg.z:.2f}m")
            self.get_logger().info("Force Disarming to guarantee safety...")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0, param2=21196.0)
            
            # Print unified hotkey instructions
            print("\n" + "="*50)
            print("🛫 UNIFIED FLIGHT CONTROLLER ACTIVE 🛫")
            print("="*50)
            print(" [ENTER] -> ARM & TAKEOFF to 0.5m")
            print(" [ L ]   -> GRACEFUL LAND (If in air)")
            print(" [SPACE] -> BRUTAL FORCE DISARM (Drop from sky)")
            print("="*50 + "\n")

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def publish_vehicle_command(self, command, **params) -> None:
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
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0, param2=21196.0)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0 
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def timer_callback(self) -> None:
        self.publish_offboard_control_heartbeat_signal()

        if self.state == "WAITING_FOR_HOME":
            return

        if self.state == "READY":
            # Send current position as setpoint so PX4 doesn't complain when we switch
            self.publish_position_setpoint(self.initial_pos.x, self.initial_pos.y, self.initial_pos.z)
            return

        if self.state == "TAKEOFF":
            # Tick 10: Request Offboard Mode
            if self.offboard_setpoint_counter == 10:
                self.get_logger().info("Commanding OFFBOARD mode...")
                self.engage_offboard_mode()
            
            # Tick 15 (0.5s later): Command Arm
            if self.offboard_setpoint_counter == 30:
                self.get_logger().info("Commanding ARM...")
                self.arm()
            
            # Stop incrementing once we pass the arm trigger
            if self.offboard_setpoint_counter < 31:
                self.offboard_setpoint_counter += 1


            # Smoothly ramp the Z setpoint UP (negative direction in NED)
            if self.current_setpoint_z > self.target_z:
                self.current_setpoint_z -= 0.02
                
            # Print a live altimeter to the screen every 0.5 seconds
            if self.timer_ticks % 5 == 0:
                current_height = abs(self.vehicle_local_position.z - self.initial_pos.z)
                self.get_logger().info(f"Live Height: {current_height:.2f}m / 0.50m Target")
                
            self.publish_position_setpoint(self.initial_pos.x, self.initial_pos.y, self.current_setpoint_z)
            self.timer_ticks += 1

# --- Keyboard Input Handling ---
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def input_thread(node):
    while rclpy.ok():
        ch = getch()
        if ch == ' ':
            node.get_logger().fatal("SPACE PRESSED! BRUTAL FORCE DISARM TRIGGERED!")
            node.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0, param2=21196.0)
            
        elif ch == '\r' or ch == '\n':
            if node.state == "READY":
                node.get_logger().warn("ENTER PRESSED! Initiating Offboard and Arming...")
                node.state = "TAKEOFF"
                node.current_setpoint_z = node.initial_pos.z
                node.target_z = node.initial_pos.z - 0.5 
            elif node.state == "TAKEOFF":
                node.get_logger().error("ENTER PRESSED AGAIN! COMMANDING SAFE LANDING...")
                node.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                
        elif ch == 'l' or ch == 'L':
            node.get_logger().error("L PRESSED! COMMANDING SAFE LANDING...")
            node.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            
        elif ch == '\x03': # Ctrl+C
            node.get_logger().error("Ctrl+C detected! COMMANDING SAFE LANDING...")
            node.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            break

def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardControl()
    
    # Start the keyboard listener in a separate thread
    thread = threading.Thread(target=input_thread, args=(node,), daemon=True)
    thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
