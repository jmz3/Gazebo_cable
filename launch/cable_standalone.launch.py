#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare(package='cable_sim').find('cable_sim')
    
    # Path to the URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'cable_model.urdf.xacro')
    
    # Launch Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'paused': 'false',
            'use_sim_time': 'true',
            'gui': 'true',
            'headless': 'false',
            'debug': 'false',
            'world': os.path.join(get_package_share_directory('gazebo_ros'), 'worlds', 'empty.world')
        }.items()
    )
    
    # Robot description parameter
    robot_description = Command(['xacro ', urdf_file])
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'cable', '-topic', 'robot_description'],
        output='screen'
    )
    
    # Cable controller node
    cable_controller = Node(
        package='cable_sim',
        executable='cable_controller_test_square.py',
        output='screen',
        parameters=[
            {'cable_PID.kP': 7.0},
            {'cable_PID.kI': 0.002},
            {'cable_PID.kD': 0.01}
        ]
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        cable_controller,
    ])