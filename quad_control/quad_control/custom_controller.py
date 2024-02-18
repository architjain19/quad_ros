import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point
import math
import numpy as np

class CustomController(Node):
    def __init__(self):
        super().__init__('custom_controller')

        self.publisher_group = self.create_publisher(JointTrajectory, '/joint_group_trajectory_controller/joint_trajectory', 10)

        self.i = 0
        self.leg_length = 0.22  # meters
        
        self.trace_trajectory_cb_status = 0 # 0: UNOCCUPIED; 1: OCCUPIED
        self.joint_limits = math.radians(120)
        self.joint_names = ['thigh_joint_fr', 'leg_joint_fr', 'thigh_joint_fl', 'leg_joint_fl', 'thigh_joint_rr', 'leg_joint_rr', 'thigh_joint_rl', 'leg_joint_rl']
        self.command_functions = [
            (self.exec_custom_stance_phase_commands, (0.0, 0.1, 0.375, 0.25, 5, 0.05)),
            (self.exec_custom_stance_phase_commands, (0.1, 0.2, 0.25, 0.375, 5, 0.05)),
            (self.exec_custom_stance_phase_commands, (0.2, 0.0, 0.375, 0.375, 5, 0.05))
        ]
        self.current_command_index = 0
        
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

    def trace_trajectory_path(self, delay_trajectory, trajectory_path):
        def trace_trajectory_cb():
            self.trace_trajectory_cb_status = 1
            itr = self.i
            x = trajectory_path[itr][0]
            y = trajectory_path[itr][1]

            angles = self.calculate_inverse_kinematics(x, y)
            if angles is None:
                self.get_logger().warn(f'Invalid command, not sending. Skipping current point {x}, {y}')
            else:
                angle1, angle2 = angles
                trajectory_msg = JointTrajectory()
                trajectory_msg.joint_names = self.joint_names
                point = JointTrajectoryPoint()
                point.positions = [angle1, angle2]*4
                point.time_from_start.sec = 1
                trajectory_msg.points.append(point)
                self.publisher_group.publish(trajectory_msg)
                self.get_logger().info(f'Joint command sent for x={x}, y={y}')

            self.i = itr + 1
            if itr >= len(trajectory_path) - 1:
                self.i = 0
                self.get_logger().info(f'Trace timer process completed cleanly!')
                self.trace_trajectory_cb_status = 0
                trace_timer.cancel()

        trace_timer = self.create_timer(delay_trajectory, trace_trajectory_cb)

    def generate_custom_stance_phase_trajectory(self, x1, x2, y1, y2, num_points):
        x_values = np.linspace(x1, x2, num_points)
        y_values = np.linspace(y1, y2, num_points)
        return list(zip(x_values, y_values))

    def exec_custom_stance_phase_commands(self, x1, x2, y1, y2, num_points, delay_trajectory):
        '''
        Calculate vertical stance trajectory points 
        and calculate joint angles using inverse kinematics over these x, y points 
        and send the joint command to respective topics 
        '''
        trajectory = self.generate_custom_stance_phase_trajectory(x1, x2, y1, y2, num_points)
        self.trace_trajectory_path(delay_trajectory, trajectory)

    def loop_control(self):
        if not self.trace_trajectory_cb_status:
            self.get_logger().info(f'Executing next command...')
            function, parameters = self.command_functions[self.current_command_index]
            function(*parameters)
            self.current_command_index = (self.current_command_index + 1) % len(self.command_functions)
        else:
            # self.get_logger().info(f'Waiting for timer to complete...')
            pass

    def custom_gait(self, delay_loop):
        # self.exec_custom_stance_phase_commands(0.0, 0.1, 0.35, 0.25, 5)
        # print(self.trace_trajectory_cb_status)
        loop_control_timer = self.create_timer(delay_loop, self.loop_control)

def main(args=None):
    rclpy.init(args=args)
    custom_controller = CustomController()
    custom_controller.custom_gait(1)     # timer=1
    rclpy.spin(custom_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()