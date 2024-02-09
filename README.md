# quad_ros

- colcon build
- source install/setup.bash
- ros2 launch quad_description quad_gazebo_launch.py
- ros2 run quad_control trot_controller

  OR

- ros2 run quad_control creep_controller

# quad test

> Sample action send goal message for standing still-
```
ros2 action send_goal /joint_group_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "trajectory:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: ''
  joint_names: ['thigh_joint_fr', 'leg_joint_fr', 'thigh_joint_fl', 'leg_joint_fl', 'thigh_joint_rr', 'leg_joint_rr', 'thigh_joint_rl', 'leg_joint_rl']
  points: [{positions: [-0.75, 1.55, -0.75, 1.55, -0.75, 1.55, -0.75, 1.55], time_from_start: {sec: 1, nanosec: 0}}]
multi_dof_trajectory:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: ''
  joint_names: []
  points: []
path_tolerance: []
component_path_tolerance: []
goal_tolerance: []
component_goal_tolerance: []
goal_time_tolerance:
  sec: 0
  nanosec: 0"
```