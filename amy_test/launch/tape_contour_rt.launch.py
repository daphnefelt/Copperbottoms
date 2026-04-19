#!/usr/bin/env python3
"""
Launch file for Real-Time Tape Contour Detection Testing
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'enable_motor_control',
            default_value='false',
            description='Enable motor control output (safety: disabled by default)'
        ),
        DeclareLaunchArgument(
            'forward_speed',
            default_value='0.25',
            description='Forward speed (m/s)'
        ),
        DeclareLaunchArgument(
            'max_turn',
            default_value='1.0',
            description='Maximum turn rate (rad/s)'
        ),
        DeclareLaunchArgument(
            'deadline_ms',
            default_value='70.0',
            description='Real-time deadline in milliseconds'
        ),
        DeclareLaunchArgument(
            'kp',
            default_value='0.8',
            description='PID Proportional gain'
        ),
        DeclareLaunchArgument(
            'ki',
            default_value='0.1',
            description='PID Integral gain'
        ),
        DeclareLaunchArgument(
            'kd',
            default_value='0.05',
            description='PID Derivative gain'
        ),
        
        # Tape Contour RT Node
        Node(
            package='amy_test',
            executable='tape_contour_rt',
            name='tape_contour_rt',
            output='screen',
            parameters=[{
                'image_topic': '/camera/color/image_raw',
                'cmd_vel_topic': '/cmd_vel',
                'debug_image_topic': '/tape_debug/image',
                'forward_speed': LaunchConfiguration('forward_speed'),
                'max_turn': LaunchConfiguration('max_turn'),
                'deadline_ms': LaunchConfiguration('deadline_ms'),
                'target_width': 320,
                'target_height': 240,
                'min_contour_area': 100,
                'publish_timing': True,
                'enable_motor_control': LaunchConfiguration('enable_motor_control'),
                'kp': LaunchConfiguration('kp'),
                'ki': LaunchConfiguration('ki'),
                'kd': LaunchConfiguration('kd'),
                'max_integral': 0.5,
            }]
        ),
    ])
