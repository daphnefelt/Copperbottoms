#!/usr/bin/env python3
"""
Launch file for TapeContourRT node with configurable parameters
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments with defaults
    enable_motor_control_arg = DeclareLaunchArgument(
        'enable_motor_control',
        default_value='true',
        description='Enable motor control output'
    )
    
    kp_arg = DeclareLaunchArgument(
        'kp',
        default_value='0.4',
        description='PID proportional gain'
    )
    
    ki_arg = DeclareLaunchArgument(
        'ki',
        default_value='0.02',
        description='PID integral gain'
    )
    
    kd_arg = DeclareLaunchArgument(
        'kd',
        default_value='0.15',
        description='PID derivative gain'
    )
    
    steering_deadband_arg = DeclareLaunchArgument(
        'steering_deadband',
        default_value='0.12',
        description='Steering deadband to reduce weaving'
    )
    
    forward_speed_arg = DeclareLaunchArgument(
        'forward_speed',
        default_value='0.22',
        description='Forward speed in m/s'
    )
    
    max_turn_arg = DeclareLaunchArgument(
        'max_turn',
        default_value='1.2',
        description='Maximum turn rate in rad/s'
    )
    
    # TapeContourRT node
    tape_contour_node = Node(
        package='amy_test',
        executable='tape_contour_rt',
        name='tape_contour_rt',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'enable_motor_control': LaunchConfiguration('enable_motor_control'),
            'kp': LaunchConfiguration('kp'),
            'ki': LaunchConfiguration('ki'),
            'kd': LaunchConfiguration('kd'),
            'steering_deadband': LaunchConfiguration('steering_deadband'),
            'forward_speed': LaunchConfiguration('forward_speed'),
            'max_turn': LaunchConfiguration('max_turn'),
        }]
    )
    
    return LaunchDescription([
        # Arguments
        enable_motor_control_arg,
        kp_arg,
        ki_arg,
        kd_arg,
        steering_deadband_arg,
        forward_speed_arg,
        max_turn_arg,
        
        # Node
        tape_contour_node,
    ])
