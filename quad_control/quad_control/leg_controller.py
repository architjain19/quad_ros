import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point
import math
import numpy as np

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

        self.trajectory = None
        self.i = 0
        # points = self.generate_stance_phase_trajectory(0.1, -0.1, 2.0, 10)
        # self.get_logger().info(f'Stance path points: {points}')

    def callback(self, msg):
        '''
        Subscriber callback to to input x, y coordinates and perform inverse kinematics and send the joint command  
        '''
        x = msg.x
        y = msg.y
        self.get_logger().info(f'Received coordinates {x}, {y}')
        joint_positions = self.send_joint_command(x, y)

    def calculate_inverse_kinematics(self, x, y):
        '''
        Calculate inverse kinematics of system based on input x, y and return joint angles
        '''
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
        angle1 = math.atan2(x, y) - math.atan2(k2, k1)  # angle for the thigh_joint

        # Check joint limits
        if abs(angle1) > self.joint_limits or abs(angle2) > self.joint_limits:
            self.get_logger().warn('Joint limit exceeded')
            return None

        return angle1, angle2

    def send_joint_command(self, x, y):
        '''
        Calculate inverse kinematics based on input x, y and send joint command on respective topic
        '''
        angles = self.calculate_inverse_kinematics(x, y)
        if angles is None:
            self.get_logger().warn('Invalid command, not sending')
            return

        angle1, angle2 = angles
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        # point.positions = [angle1, angle2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.positions = [angle1, angle2]*4
        point.time_from_start.sec = 1  # Adjust timing as needed
        trajectory_msg.points.append(point)

        self.publisher_group.publish(trajectory_msg)
        self.get_logger().info('Joint command sent')

    def generate_stance_phase_trajectory(self, start_x, end_x, step_height, num_points):
        x_values = np.linspace(start_x, end_x, num_points)
        # Keeping y constant, as we want a straight line trajectory for the stance phase
        y_values = [step_height] * num_points  # stance_height is constant for all points
        return list(zip(x_values, y_values))
    
    def trace_trajecrtory_cb(self):
        itr = self.i

        x = self.trajectory[itr][0]
        y = self.trajectory[itr][1]

        angles = self.calculate_inverse_kinematics(x, y)
        if angles is None:
            self.get_logger().warn(f'Invalid command, not sending. Skipping current point {x}, {y}')
        else:
            angle1, angle2 = angles
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = self.joint_names
            point = JointTrajectoryPoint()
            # point.positions = [angle1, angle2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            point.positions = [angle1, angle2]*4
            point.time_from_start.sec = 1  # Adjust timing as needed
            trajectory_msg.points.append(point)

            self.publisher_group.publish(trajectory_msg)
            self.get_logger().info(f'Joint command sent for x={x}, y={y}')

        self.i = itr + 1

        if itr >= len(self.trajectory) - 1:
            self.get_logger().info(f'Timer process completed cleanly!')
            self.trace_trajecrtory.cancel()

    def send_stance_phase_commands(self, start_x, end_x, step_height, num_points):
        '''
        Calculate stance trajectory points 
        and calculate joint angles using inverse kinematics over these x, y points 
        and send the joint command to respective topics 
        '''
        self.trajectory = self.generate_stance_phase_trajectory(start_x, end_x, step_height, num_points)

        self.trace_trajecrtory = self.create_timer(0.25, self.trace_trajecrtory_cb)

    def generate_swing_phase_trajectory(self, center_x, center_y, radius, num_points):
        # Theta values for a semicircle from 0 to pi
        theta_values = np.linspace(np.pi, 2*np.pi, num_points)
        # Calculate x and y positions for the semicircle trajectory
        x_values = center_x + radius * np.cos(theta_values)
        y_values = center_y + radius * np.sin(theta_values)
        
        return list(zip(x_values, y_values))

    def send_swing_phase_commands(self, center_x, center_y, radius, num_points):
        '''
        Calculate swing trajectory points
        and calculate joint angles using inverse kinematics over these x, y points
        and send the joint command to respective topics.
        Note: Here center x, y given; so if x=0 and radius is 0.05, then it'll start from x=-0.05 to x=+0.05
        '''
        self.trajectory = self.generate_swing_phase_trajectory(center_x, center_y, radius, num_points)

        self.trace_trajecrtory = self.create_timer(0.25, self.trace_trajecrtory_cb)


def main(args=None):
    rclpy.init(args=args)
    leg_controller = LegController()
    # print(leg_controller.generate_stance_phase_trajectory(0.15, -0.1, 0.35, 10))
    # leg_controller.send_stance_phase_commands(0.1, -0.1, 0.275, 10)
    leg_controller.send_swing_phase_commands(0.0, 0.3, 0.05, 30)
    rclpy.spin(leg_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()