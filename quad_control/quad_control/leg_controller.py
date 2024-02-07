import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point
import math

class LegController(Node):
    def __init__(self):
        super().__init__('leg_controller')
        self.publisher_group = self.create_publisher(JointTrajectory, '/joint_group_trajectory_controller/joint_trajectory', 10)
        # Assuming you have a way to input x, y for the end effector. 
        # This could be another subscriber or a service call handling part in a real application.
        self.leg_length = 0.22  # meters
        self.joint_limits = math.radians(120)  # converting degrees to radians
        self.joint_names = ['thigh_joint_fr', 'leg_joint_fr', 'thigh_joint_fl', 'leg_joint_fl', 'thigh_joint_rr', 'leg_joint_rr', 'thigh_joint_rl', 'leg_joint_rl']

        self.subscription = self.create_subscription(Point, '/target_coordinates', self.callback, 10)
        self.subscription

    def callback(self, msg):
        x = msg.x
        y = msg.y
        self.get_logger().info(f'Received coordinates {x}, {y}')
        joint_positions = self.send_joint_command(x, y)

    def calculate_inverse_kinematics(self, x, y):
        # Using the geometric approach for inverse kinematics
        L = self.leg_length
        distance = math.sqrt(x**2 + y**2)  # distance from origin to end effector
        if distance > 2 * L:  # Cannot reach this point
            self.get_logger().warn('Point out of reach')
            return None

        # Law of cosines to find joint angles
        cos_angle2 = (x**2 + y**2 - 2*L**2) / (2*L**2)
        angle2 = math.acos(cos_angle2)  # angle for the leg_joint
        k1 = L + L * math.cos(angle2)
        k2 = L * math.sin(angle2)
        angle1 = math.atan2(y, x) - math.atan2(k2, k1)  # angle for the thigh_joint

        # Check joint limits
        if abs(angle1) > self.joint_limits or abs(angle2) > self.joint_limits:
            self.get_logger().warn('Joint limit exceeded')
            return None

        return angle1, angle2

    def send_joint_command(self, x, y):
        angles = self.calculate_inverse_kinematics(x, y)
        if angles is None:
            self.get_logger().warn('Invalid command, not sending')
            return

        angle1, angle2 = angles
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [angle1, angle2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 1  # Adjust timing as needed
        trajectory_msg.points.append(point)

        self.publisher_group.publish(trajectory_msg)
        self.get_logger().info('Joint command sent')

def main(args=None):
    rclpy.init(args=args)
    leg_controller = LegController()

    rclpy.spin(leg_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()