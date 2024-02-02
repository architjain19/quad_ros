# *****************************************************************************************
# *
# *  Filename:              quad_traj.py
# *  Created:				02/02/2024
# *  Last Modified:	        02/02/2024
# *  Author:				Archit Jain
# *  
# *****************************************************************************************


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class HopperWalker(Node):
    def __init__(self):
        super().__init__('hopper_walker')

        self.leg_link = 0.220
        self.thigh_link = 0.220
        self.joint_limit = 2.00712863979

        self.publisher_group = self.create_publisher(JointTrajectory, '/joint_group_trajectory_controller/joint_trajectory', 10)

        # self.publish_joint_angles(self.deg_to_rad(-30), self.deg_to_rad(90))

        # ang1, ang2 = self.ik_solver(0.25, -0.1)
        # if ang1 is not None and ang2 is not None:
            # self.publish_joint_angles(self.deg_to_rad(ang1), self.deg_to_rad(ang2))

        # Path parameters
        self.x_start, self.y_start = 0.25, 0.0
        self.x_end, self.y_end = 0.25, -0.1
        self.frequency, self.amplitude = 2, 0.05
        self.current_step = 0
        self.num_steps = 100

        # Timer and path state
        self.timer = self.create_timer(0.1, self.follow_sine_wave_path)  # Timer with callback every 0.1 seconds

    def deg_to_rad(self, angle):
        return angle*3.14159265359/180.0

    def rad_to_deg(self, angle):
        return angle*180.0/3.14159265359

    def publish_joint_angles(self, angle1, angle2):
        if abs(angle1) > self.joint_limit or abs(angle2) > self.joint_limit:
            self.get_logger().warn(f"Joint angle is greater than joint limits. Be cautious!")
        
        joint_angles = [angle1, angle2]
        joint_trajectory_msg = JointTrajectory()
        joint_trajectory_msg.joint_names = ['thigh_joint_fr', 'leg_joint_fr', 'thigh_joint_fl', 'leg_joint_fl', 'thigh_joint_rr', 'leg_joint_rr', 'thigh_joint_rl', 'leg_joint_rl']
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = joint_angles*4
        trajectory_point.time_from_start.sec = 1
        trajectory_point.time_from_start.nanosec = 0
        joint_trajectory_msg.points.append(trajectory_point)
        
        self.publisher_group.publish(joint_trajectory_msg)

    def ik_solver(self, x, y):
        L1 = self.leg_link
        L2 = self.thigh_link

        try:
            # Calculate the distance from the base to the point (x, y)
            D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)

            # Check if the target is reachable
            if D < -1 or D > 1:
                self.get_logger().warn("Target is unreachable.")
                return None, None

            # Calculate joint angles
            theta2 = math.atan2(math.sqrt(1 - D**2), D)  # Elbow angle
            theta1 = math.atan2(y, x) - math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))  # Shoulder angle

            # Ensure angles are within the joint limits
            if not(-self.joint_limit <= theta1 <= self.joint_limit and -self.joint_limit <= theta2 <= self.joint_limit):
                self.get_logger().warn(f"Calculated joint angles {self.rad_to_deg(theta1)} and {self.rad_to_deg(theta2)} are out of limits.")
                return None, None

             # Convert to degrees and adjust for joint limits
            theta1_deg = math.degrees(theta1)
            theta2_deg = math.degrees(theta2)

            return theta1_deg, theta2_deg

        except ValueError as e:
            # Handle math domain errors
            self.get_logger().warn(f"Math domain error occurred: {e}")
            return None, None

    def follow_sine_wave_path(self):

        self.publish_joint_angles(-0.75, 1.57)

        if self.current_step > self.num_steps:
            self.timer.cancel()  # Stop the timer if the path is complete
            self.get_logger().info(f"Path completed")
            return

        self.current_step += 1


def main(args=None):
    rclpy.init(args=args)
    hopper_walker = HopperWalker()
    rclpy.spin(hopper_walker)
    hopper_walker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()