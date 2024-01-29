#!/usr/bin/python3
# -*- coding: utf-8 -*-

''' 
*****************************************************************************************
*
*  Filename:			quad_gazebo_launch.py
*  Description:         Use this file to spawn quadruped inside quad warehouse world in the gazebo simulator and publish robot states.
*  Created:				22/12/2023
*  Last Modified:	    22/12/2023
*  Author:				Archit Jain
*  
*****************************************************************************************
'''

import launch
import launch_ros
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='quad_description').find('quad_description')

    xacro_file_quad = os.path.join(pkg_share, 'models/','quad/', 'quad.xacro')
    assert os.path.exists(xacro_file_quad), "The quad.xacro doesnt exist in "+str(xacro_file_quad)
    robot_description_config_quad = xacro.process_file(xacro_file_quad)
    robot_description = robot_description_config_quad.toxml()


    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('quad_description'), 'launch', 'start_world_launch.py'),
        )
    )

    robot_state_publisher_node_quad = launch_ros.actions.Node(
        package='robot_state_publisher',
        name='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"robot_description": robot_description}]
    )

    static_transform = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments = ["1.6", "-2.4", "-0.8", "3.14", "0", "0", "world", "odom"],
        output='screen')
    
    spawn_quad = launch_ros.actions.Node(
    	package='gazebo_ros', 
        name='quad_spawner',
    	executable='spawn_entity.py',
        arguments=['-entity', 'quad', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.0', '-Y', '0.0'],
        output='screen'
    )

    joint_state_publisher_node_quad = launch_ros.actions.Node(
        package="joint_state_publisher",
        name="joint_state_publisher",
        executable="joint_state_publisher"
    )
    
    joint_state_publisher_gui_node_quad = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    spawn_controller_fr = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_controller_fr", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_controller_fl = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_controller_fl", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        start_world,
        robot_state_publisher_node_quad,
        spawn_quad,
        joint_state_broadcaster_spawner,
        spawn_controller_fr,
        spawn_controller_fl
        # static_transform,
        # joint_state_publisher_node_quad,
        # joint_state_publisher_gui_node_quad
    ])