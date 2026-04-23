#!/usr/bin/env python3
"""
Odometry Launch File
Launches LiDAR, depth camera, IMU, and LIO odometry node for accurate pose estimation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    connection_string_arg = DeclareLaunchArgument(
        'connection_string',
        default_value='/dev/ttyACM1',
        description='MAVLink connection string for rover (IMU source)'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLiDAR'
    )
    
    # Include rover launch for IMU data
    rover_launch_path = os.path.join(
        get_package_share_directory('robo_rover'),
        'launch',
        'rover_launch.py'
    )
    
    rover_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rover_launch_path),
        launch_arguments={
            'connection_string': LaunchConfiguration('connection_string'),
        }.items()
    )
    
    # RPLiDAR node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }]
    )
    
    # RealSense node with depth enabled
    realsense_node = Node(
        package='realsense_publisher',
        executable='realsense_depth_node',
        name='realsense_depth_publisher',
        output='screen',
    )
    
    # LIO Odometry node
    lio_node = Node(
        package='amy_test',
        executable='lio_odometry',
        name='lio_odometry',
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        # Arguments
        connection_string_arg,
        serial_port_arg,
        
        # Nodes
        rover_launch,      # Provides IMU data (/imu/accel, /imu/gyro)
        rplidar_node,      # Provides LiDAR scans (/scan)
        realsense_node,    # Provides depth point cloud (/camera/depth/points)
        lio_node,          # Processes all sensors for odometry (/odom/lio)
    ])
