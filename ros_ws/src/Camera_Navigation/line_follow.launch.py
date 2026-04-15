#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
	workspace_src_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
	camera_nav_dir = os.path.join(workspace_src_dir, 'Camera_Navigation')
	fastsam_dir = os.path.expanduser('~/robo_realsense/fastsam')

	line_follow = ExecuteProcess(
		cmd=['python3', os.path.join(camera_nav_dir, 'line_follow.py')],
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
		ros_stream_with_depth,
		rover_node,
		line_follow,
	])
