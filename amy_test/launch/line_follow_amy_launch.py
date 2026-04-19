#!/usr/bin/env python3
"""
Launch file for Amy line follower with optional monitoring
"""
import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _get_running_node_names() -> set[str]:
    """Check which ROS2 nodes are currently running"""
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
    # Get paths
    workspace_src_dir = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', '..')
    )
    amy_test_dir = os.path.join(workspace_src_dir, 'amy_test', 'amy_test')
    fastsam_dir = os.path.expanduser('~/robo_realsense/fastsam')
    
    # Check running nodes
    running_nodes = _get_running_node_names()
    print(f"Currently running nodes: {running_nodes}")

    # Launch arguments
    with_monitoring = DeclareLaunchArgument(
        'with_monitoring',
        default_value='false',
        description='Enable latency monitoring nodes'
    )
    
    with_dashboard = DeclareLaunchArgument(
        'with_dashboard',
        default_value='false',
        description='Enable real-time dashboard'
    )
    
    debug_plot = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug visualization'
    )
    
    debug_interval = DeclareLaunchArgument(
        'debug_interval',
        default_value='1',
        description='Update debug view every N frames (1=every frame, 2=every other frame, etc.)'
    )
    
    forward_speed = DeclareLaunchArgument(
        'speed',
        default_value='0.25',
        description='Forward speed (m/s)'
    )
    
    kp_gain = DeclareLaunchArgument(
        'kp',
        default_value='0.8',
        description='Proportional gain for steering'
    )
    
    ki_gain = DeclareLaunchArgument(
        'ki',
        default_value='0.1',
        description='Integral gain for steering (eliminates steady-state error)'
    )
    
    kd_gain = DeclareLaunchArgument(
        'kd',
        default_value='0.05',
        description='Derivative gain for steering (dampens oscillations)'
    )

    # Core nodes
    line_follower = Node(
        package='amy_test',
        executable='line_follow_amy',
        name='line_follower_amy',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'image_topic': '/camera/color/image_raw',
            'cmd_vel_topic': '/cmd_vel',
            'forward_speed': LaunchConfiguration('speed'),
            'kp': LaunchConfiguration('kp'),
            'ki': LaunchConfiguration('ki'),
            'kd': LaunchConfiguration('kd'),
            'max_turn': 1.0,
            'min_pixels': 50,
            'debug_plot': LaunchConfiguration('debug'),
            'debug_interval': LaunchConfiguration('debug_interval'),
            'publish_timing': True,
        }]
    )

    rover_node = Node(
        package='robo_rover',
        executable='rover_node',
        name='rover_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression([
                "'rover_node' not in ", repr(list(running_nodes)), " and ",
                "'/rover_node' not in ", repr(list(running_nodes))
            ])
        )
    )

    ros_stream = ExecuteProcess(
        cmd=['python3', os.path.join(fastsam_dir, 'ros_stream_with_depth.py')],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression([
                "'ros_stream_with_depth' not in ", repr(list(running_nodes)), " and ",
                "'realsense_color_publisher' not in ", repr(list(running_nodes))
            ])
        )
    )

    # Optional monitoring nodes
    monitoring_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(workspace_src_dir, 'amy_test', 'launch', 'latency_monitoring.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('with_monitoring'))
    )

    dashboard_node = Node(
        package='amy_test',
        executable='latency_dashboard',
        name='latency_dashboard',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('with_dashboard'))
    )

    return LaunchDescription([
        # Arguments
        with_monitoring,
        with_dashboard,
        debug_plot,
        debug_interval,
        forward_speed,
        kp_gain,
        ki_gain,
        kd_gain,
        
        # Core nodes
        ros_stream,
        rover_node,
        line_follower,
        
        # Optional monitoring
        monitoring_launch,
        dashboard_node,
    ])
