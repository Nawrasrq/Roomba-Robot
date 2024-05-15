#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf = os.path.join(
        get_package_share_directory('trashbot_description'),
        'urdf',
        'turtlebot3_burger.urdf')
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rviz_config_dir = os.path.join(
        get_package_share_directory('trashbot_description'),
        'rviz',
        'trashbot.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc,
                          'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen',
        ),
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='rqt_graph',
        ),
    ])
