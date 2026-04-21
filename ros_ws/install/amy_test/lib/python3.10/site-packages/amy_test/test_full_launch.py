#!/usr/bin/env python3
"""
Full Launch file for ROBO Rover System
Launches:
- Rover control node (MAVLink communication)
- RealSense camera node
- RPLidar A1 sensor node (publishes /scan)
- Tape vision node (detects tape)
- Tape control node (PID line following with LIDAR contingency)
- Tape recovery node (handles tape loss)
- Turn right node (LIDAR-based right-turn detection)
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
    
    # Include the RPLidar launch file
    rplidar_launch_path = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'view_rplidar_a1_launch.py'
    )
    
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_path)
    )
    
    # RealSense camera node
    realsense_node = Node(
        package='realsense_publisher',
        executable='realsense_node',
        name='realsense_publisher',
        output='screen',
        emulate_tty=True,
    )
    
    tape_vision_node = Node(
        package='amy_test',
        executable='tape_vision_node',
        name='tape_vision_node',
        output='screen',
        emulate_tty=True,
    )

    tape_control_node = Node(
        package='amy_test',
        executable='tape_control_node',
        name='tape_control_node',
        output='screen',
        emulate_tty=True,
    )

    tape_recovery_node = Node(
        package='amy_test',
        executable='tape_recovery_node',
        name='tape_recovery_node',
        output='screen',
        emulate_tty=True,
    )

    # LIDAR turn detection node - processes /scan and publishes to /turn_right
    turn_right_node = Node(
        package='camera_navigation',
        executable='turn_right_node',
        name='turn_right_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        # Arguments
        connection_string_arg,
        baud_rate_arg,

        # Launch files and nodes
        rover_launch,
        rplidar_launch,
        realsense_node,
        tape_vision_node,
        tape_control_node,
        tape_recovery_node,
        turn_right_node,
    ])
