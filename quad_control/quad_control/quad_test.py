import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import numpy as np

class HopperWalker(Node):
    def __init__(self):
        super().__init__('hopper_walker')

        self.leg_link = 0.175
        self.thigh_link = 0.220
        self.joint_limit = 2.00712863979

        self.publisher = self.create_publisher(Float64MultiArray, '/joint_group_controller/commands', 10)

        self.publish_joint_angles(self.deg_to_rad(-30), self.deg_to_rad(90))

        self.follow_sine_wave_path(0.25, 0.0, 0.25, 0.3, 2, 0.05)


        # ang1, ang2 = self.ik_solver(0.25, -0.1)
        # if ang1 is not None and ang2 is not None:
        #     self.publish_joint_angles(ang1, ang2)

    def deg_to_rad(self, angle):
        return angle*3.14159265359/180.0

    def rad_to_deg(self, angle):
        return angle*180.0/3.14159265359

    def publish_joint_angles(self, angle1, angle2):
        if abs(angle1) > self.joint_limit or abs(angle2) > self.joint_limit:
            self.get_logger().warn(f"Joint angle is greater than joint limits. Be cautious!")

        joint_positions = Float64MultiArray()
        joint_positions.data = [angle1, angle2]
        self.publisher.publish(joint_positions)        

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
    
    def follow_sine_wave_path(self, x_start, y_start, x_end, y_end, frequency, amplitude):
        num_steps = 100
        for step in range(num_steps + 1):
            y = y_start + (y_end - y_start) * (step / num_steps)
            sine_offset = amplitude * np.sin(frequency * np.pi * step / num_steps)
            y_with_offset = y + sine_offset
            theta1_deg, theta2_deg = self.ik_solver(x_start, y_with_offset)
            if theta1_deg is not None and theta2_deg is not None:
                self.publish_joint_angles(self.deg_to_rad(theta1_deg), self.deg_to_rad(theta2_deg))
                import time
                time.sleep(0.5)
            else:
                print("Unable to calculate or publish joint angles for point:", x_start, y_with_offset)


def main(args=None):
    rclpy.init(args=args)
    hopper_walker = HopperWalker()
    rclpy.spin(hopper_walker)
    hopper_walker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
