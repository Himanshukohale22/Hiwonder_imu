#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_path = get_package_share_directory('python_imu')
    urdf_file = os.path.join(pkg_path, 'urdf', 'imu.urdf')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'imu.rviz')

    # Read URDF file content
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            name='rviz_config',
            default_value=rviz_config_file,
            description='Absolute path to rviz config file'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')]
        ),
        Node(
            package='python_imu',  # Replace with your IMU package name
            executable='pub',  # Replace with your IMU node executable name
            name='imu_node',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB0', 'baudrate': 9600}],
            remappings=[('/imu/raw','/imu/data')]
        ),
       Node(
            package='python_imu',  # Replace with your IMU package name
            executable='tf_broad',  # Replace with your IMU node executable name
            name='imu_node',
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'imu_link']
        ),
    ])
