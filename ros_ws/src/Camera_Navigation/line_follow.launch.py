#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
	workspace_src_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
	camera_nav_dir = os.path.join(workspace_src_dir, 'Camera_Navigation')
	fastsam_dir = os.path.expanduser('~/robo_realsense/fastsam')

	"""
	To run
	ros2 launch src/Camera_Navigation/line_follow.launch.py
	"""

	# Kill prior instances to avoid stale node/process overlap from previous runs.
	kill_line_follow = ExecuteProcess(
		cmd=['bash', '-lc', "pkill -f 'Camera_Navigation/line_follow.py' || true"],
		output='screen',
		emulate_tty=True,
	)

	kill_rover = ExecuteProcess(
		cmd=['bash', '-lc', "pkill -f 'robo_rover/rover_node' || true; pkill -f ' --ros-args -r __node:=rover_node' || true"],
		output='screen',
		emulate_tty=True,
	)

	kill_realsense = ExecuteProcess(
		cmd=['bash', '-lc', "pkill -f 'fastsam/ros_stream_with_depth.py' || true"],
		output='screen',
		emulate_tty=True,
	)

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
		kill_line_follow,
		kill_rover,
		kill_realsense,
		TimerAction(
			period=1.0,
			actions=[
				rover_node,
			],
		),
		TimerAction(
			period=2.5,
			actions=[
				ros_stream_with_depth,
			],
		),
		TimerAction(
			period=6.0,
			actions=[
				line_follow,
			],
		),
	])
