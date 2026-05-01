#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    workspace_src_dir = os.path.expanduser('~/code/Copperbottoms/ros_ws/src')
    fastsam_dir = os.path.expanduser('~/robo_realsense/fastsam')

    find_paper = ExecuteProcess(
        cmd=['python3', os.path.join(workspace_src_dir, 'find_paper.py')],
        output='screen',
        emulate_tty=True,
    )

    pid_paper_following = ExecuteProcess(
        cmd=['python3', os.path.join(workspace_src_dir, 'pid_paper_following.py')],
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

    ros_stream_with_depth = ExecuteProcess(
        cmd=['python3', os.path.join(fastsam_dir, 'ros_stream_with_depth.py')],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        find_paper,
        pid_paper_following,
        rover_node,
        ros_stream_with_depth,
    ])