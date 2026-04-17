#!/usr/bin/env python3
"""
Full Launch file for ROBO Rover System
Launches both the rover control node and RealSense camera node
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments for rover
    connection_string_arg = DeclareLaunchArgument(
        'connection_string',
        default_value='/dev/ttyACM1',
        description='MAVLink connection string for rover'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial connection'
    )
    
    # Include the rover launch file
    rover_launch_path = os.path.join(
        get_package_share_directory('robo_rover'),
        'launch',
        'rover_launch.py'
    )
    
    rover_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rover_launch_path),
        launch_arguments={
            'connection_string': LaunchConfiguration('connection_string'),
            'baud_rate': LaunchConfiguration('baud_rate'),
        }.items()
    )
    
    # RealSense camera node
    realsense_node = Node(
        package='realsense_publisher',
        executable='realsense_node',
        name='realsense_publisher',
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        # Arguments
        connection_string_arg,
        baud_rate_arg,
        
        # Launch files and nodes
        rover_launch,
        realsense_node,
    ])
