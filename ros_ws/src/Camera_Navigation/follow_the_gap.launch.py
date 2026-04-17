#!/usr/bin/env python3

import os
import subprocess
import time
import rclpy
from rclpy.node import Node as RclpyNode
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def _get_running_node_names() -> set[str]:
    names: set[str] = set()
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            check=True,
            capture_output=True,
            text=True,
            timeout=3,
        )
    except Exception:
        return names

    for raw_line in result.stdout.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        names.add(line)
        if line.startswith('/'):
            names.add(line[1:])
    return names

def generate_launch_description():
    workspace_src_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    camera_nav_dir = os.path.join(workspace_src_dir, 'Camera_Navigation')
    running_nodes = _get_running_node_names()
    print(f"nodes already running at launch start: {running_nodes}")

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

    launch_actions = [follow_the_gap]

    if 'rplidar_node' not in running_nodes and '/rplidar_node' not in running_nodes:
        launch_actions.insert(0, lidar_node)

    if 'rover_node' not in running_nodes and '/rover_node' not in running_nodes:
        launch_actions.insert(0, rover_node)

    return LaunchDescription(launch_actions)