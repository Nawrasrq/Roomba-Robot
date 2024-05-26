#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf = os.path.join(
        get_package_share_directory('trashbot_description'),
        'urdf',
        'trashbot.urdf')
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    
    world = os.path.join(
        get_package_share_directory('trashbot_bringup'),
        'worlds',
        'empty_world.sdf')
    
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        # Run gz_sim pkgs launch file to launch gazebo 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            # Specifying what world we want to load
            launch_arguments={
                'gz_args': '-r ' + world
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc,
                          'use_sim_time': use_sim_time}],
        ),
        Node(
            package='ros_gz_sim', 
            executable='create',
            output='screen',
            arguments=[
            '-name', 'trashbot',
            '-topic', '/robot_description',
            ],
        ),
    ])