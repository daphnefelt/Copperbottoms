#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── RPLidar ───────────────────────────────────────────────────────────
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type':     'serial',
                'serial_port':      '/dev/ttyUSB0',
                'serial_baudrate':  115200,
                'frame_id':         'laser',
                'inverted':         False,
                'angle_compensate': True,
                'scan_mode':        'Standard',
            }],
            output='screen'),

        # ── Hallway centering ─────────────────────────────────────────────────
        Node(
            package='rplidar_ros',
            executable='hallway_center_node',
            name='hallway_center_node',
            output='screen'),

        # ── ArduPilot rover ───────────────────────────────────────────────────
        Node(
            package='robo_rover',
            executable='rover_node',
            name='rover_node',
            output='screen'),

    ])
