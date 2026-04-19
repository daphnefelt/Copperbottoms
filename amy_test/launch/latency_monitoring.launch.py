#!/usr/bin/env python3
"""
Latency Monitoring Launch File - Launches all monitoring infrastructure
Non-invasive monitoring system for RTES rover latency validation

Launch modes:
  - monitoring: Rate and latency monitors + aggregator + logger (default)
  - dashboard: All monitoring + live dashboard display
  - test: All monitoring + automated test (runs for test_duration then exits)

Usage:
  # Standard monitoring (background logging)
  ros2 launch amy_test latency_monitoring.launch.py
  
  # With live dashboard
  ros2 launch amy_test latency_monitoring.launch.py mode:=dashboard
  
  # Automated test (30 seconds)
  ros2 launch amy_test latency_monitoring.launch.py mode:=test
  
  # Custom test duration
  ros2 launch amy_test latency_monitoring.launch.py mode:=test test_duration:=60

Part of the latency monitoring infrastructure for RTES rover application
All monitoring code is non-invasive and contained in amy_test package
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for latency monitoring system"""
    
    # Launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='monitoring',
        description='Launch mode: monitoring (default), dashboard, or test'
    )
    
    test_duration_arg = DeclareLaunchArgument(
        'test_duration',
        default_value='30.0',
        description='Test duration in seconds (for test mode)'
    )
    
    log_dir_arg = DeclareLaunchArgument(
        'log_dir',
        default_value='~/performance_logs',
        description='Directory for performance logs'
    )
    
    # Monitoring nodes (always launched)
    topic_rate_monitor = Node(
        package='amy_test',
        executable='topic_rate_monitor',
        name='topic_rate_monitor',
        output='screen',
        parameters=[{
            'window_duration': 5.0,
            'monitor_camera': True,
            'monitor_cmd_vel': True,
            'monitor_imu': True,
        }]
    )
    
    vision_latency_monitor = Node(
        package='amy_test',
        executable='vision_latency_monitor',
        name='vision_latency_monitor',
        output='screen',
        parameters=[{
            'measurement_window': 5.0,
            'sync_slop': 0.1,
            'deadline_ms': 50.0,
        }]
    )
    
    motor_latency_monitor = Node(
        package='amy_test',
        executable='motor_latency_monitor',
        name='motor_latency_monitor',
        output='screen',
        parameters=[{
            'measurement_window': 10.0,
            'accel_threshold': 0.1,
            'response_window_ms': 200.0,
            'deadline_ms': 20.0,
        }]
    )
    
    latency_aggregator = Node(
        package='amy_test',
        executable='latency_aggregator',
        name='latency_aggregator',
        output='screen',
        parameters=[{
            'total_deadline_ms': 100.0,
        }]
    )
    
    performance_logger = Node(
        package='amy_test',
        executable='performance_logger',
        name='performance_logger',
        output='screen',
        parameters=[{
            'log_directory': LaunchConfiguration('log_dir'),
            'log_prefix': 'rover_performance',
        }]
    )
    
    # Dashboard node (only in dashboard mode)
    latency_dashboard = Node(
        package='amy_test',
        executable='latency_dashboard',
        name='latency_dashboard',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'dashboard'"])
        )
    )
    
    # Automated test node (only in test mode)
    automated_test = Node(
        package='amy_test',
        executable='automated_latency_test',
        name='automated_latency_test',
        output='screen',
        parameters=[{
            'test_duration_s': LaunchConfiguration('test_duration'),
            'vision_deadline_ms': 50.0,
            'motor_deadline_ms': 20.0,
            'total_deadline_ms': 100.0,
            'max_miss_rate_percent': 5.0,
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'test'"])
        )
    )
    
    # Status messages
    monitoring_info = LogInfo(
        msg='Launching latency monitoring (background logging mode)',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'monitoring'"])
        )
    )
    
    dashboard_info = LogInfo(
        msg='Launching latency monitoring with live dashboard',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'dashboard'"])
        )
    )
    
    test_info = LogInfo(
        msg=['Launching automated latency test (duration: ', 
             LaunchConfiguration('test_duration'), 's)'],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'test'"])
        )
    )
    
    return LaunchDescription([
        # Arguments
        mode_arg,
        test_duration_arg,
        log_dir_arg,
        
        # Status messages
        monitoring_info,
        dashboard_info,
        test_info,
        
        # Core monitoring nodes (always launched)
        topic_rate_monitor,
        vision_latency_monitor,
        motor_latency_monitor,
        latency_aggregator,
        performance_logger,
        
        # Optional nodes (mode-dependent)
        latency_dashboard,
        automated_test,
    ])
