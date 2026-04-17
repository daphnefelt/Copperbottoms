#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    workspace_src_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    camera_nav_dir = os.path.join(workspace_src_dir, 'Camera_Navigation')

    follow_the_gap = ExecuteProcess(
        cmd=['python3', os.path.join(camera_nav_dir, 'follow_the_gap.py')],
        output='screen',
        emulate_tty=True,
    )

    rover_node = Node(
        package='robo_rover',
        executable='rover_node',
        name='rover_node',
        output='screen',
        emulate_tty=True,
    )

    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        lidar_node,
        rover_node,
        follow_the_gap,
    ])