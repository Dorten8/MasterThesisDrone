import rclpy
from px4_msgs.msg import TrajectorySetpoint
def test():
    msg = TrajectorySetpoint()
    print("Position:", msg.position)
    print("Velocity:", msg.velocity)
    print("Acceleration:", msg.acceleration)
    print("Jerk:", msg.jerk)
if __name__ == '__main__':
    test()
