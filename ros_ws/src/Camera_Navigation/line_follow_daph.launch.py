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
	fastsam_dir = os.path.expanduser('~/robo_realsense/fastsam')
	running_nodes = _get_running_node_names()
	print(f"nodes already running at launch start: {running_nodes}")

	line_follow_daph = ExecuteProcess(
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
	
	launch_actions = [line_follow_daph]

	if 'rover_node' not in running_nodes and '/rover_node' not in running_nodes:
		launch_actions.insert(0, rover_node)
		
	if 'ros_stream_with_depth' not in running_nodes and '/ros_stream_with_depth' not in running_nodes:
		launch_actions.insert(0, ros_stream_with_depth)

	return LaunchDescription(launch_actions)