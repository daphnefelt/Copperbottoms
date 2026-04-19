#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
	workspace_src_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
	camera_nav_dir = os.path.join(workspace_src_dir, 'Camera_Navigation')
	fastsam_dir = os.path.expanduser('~/robo_realsense/fastsam')
	robo_realsense_dir = os.path.expanduser('~/robo_realsense')
	cleanup_previous = LaunchConfiguration('cleanup_previous')
	camera_mode = LaunchConfiguration('camera_mode')

	"""
	To run
	ros2 launch src/Camera_Navigation/line_follow_V2.launch.py
	ros2 launch src/Camera_Navigation/line_follow_V2.launch.py cleanup_previous:=true
	"""

	# Optional cleanup of prior instances to avoid stale overlap from previous runs.
	# Disabled by default so launch does not kill other sessions/processes unexpectedly.
	cleanup_arg = DeclareLaunchArgument(
		'cleanup_previous',
		default_value='false',
		description='If true, pkill old line_follow_V2/rover/realsense processes before launch.'
	)

	camera_mode_arg = DeclareLaunchArgument(
		'camera_mode',
		default_value='color',
		description="Camera stream mode: 'color' (color-only) or 'color_depth' (legacy color+depth)."
	)

	kill_line_follow_V2 = ExecuteProcess(
		cmd=['bash', '-lc', "pkill -f '[C]amera_Navigation/line_follow_V2.py' || true"],
		condition=IfCondition(cleanup_previous),
		output='screen',
		emulate_tty=True,
	)

	kill_rover = ExecuteProcess(
		cmd=['bash', '-lc', "pkill -f '[r]obo_rover/rover_node' || true; pkill -f '[r]os2 run robo_rover rover_node --ros-args -r __node:=rover_node' || true"],
		condition=IfCondition(cleanup_previous),
		output='screen',
		emulate_tty=True,
	)

	kill_realsense = ExecuteProcess(
		cmd=['bash', '-lc', "pkill -f '[f]astsam/ros_stream_with_depth.py' || true; pkill -f '[r]obo_realsense/ros_stream.py' || true"],
		condition=IfCondition(cleanup_previous),
		output='screen',
		emulate_tty=True,
	)

	line_follow_V2 = ExecuteProcess(
		cmd=[
			'bash',
			'-lc',
			(
				f"if ros2 node list 2>/dev/null | grep -Fxq '/line_follower_v2' "
				f"; then "
				f"echo '[launch] /line_follower_v2 already running; skipping start'; "
				f"else exec python3 {os.path.join(camera_nav_dir, 'line_follow_V2.py')}; fi"
			),
		],
		output='screen',
		emulate_tty=True,
	)

	rover_node = ExecuteProcess(
		cmd=[
			'bash',
			'-lc',
			(
				"if ros2 node list 2>/dev/null | grep -Fxq '/rover_node' "
				"&& ros2 topic info /rover/armed 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then "
				"echo '[launch] /rover_node already running with /rover/armed publisher; skipping start'; "
				"else exec ros2 run robo_rover rover_node --ros-args -r __node:=rover_node; fi"
			),
		],
		output='screen',
		emulate_tty=True,
	)

	ros_stream_with_depth = ExecuteProcess(
		cmd=[
			'bash',
			'-lc',
			(
				f"if ros2 node list 2>/dev/null | grep -Fxq '/realsense_color_depth_publisher' "
				f"; then "
				f"echo '[launch] /realsense_color_depth_publisher already running; skipping start'; "
				f"else exec python3 {os.path.join(fastsam_dir, 'ros_stream_with_depth.py')}; fi"
			),
		],
		condition=IfCondition(PythonExpression(["'", camera_mode, "' == 'color_depth'"])),
		output='screen',
		emulate_tty=True,
	)

	ros_stream_color_only = ExecuteProcess(
		cmd=[
			'bash',
			'-lc',
			(
				f"if ros2 node list 2>/dev/null | grep -Fxq '/realsense_color_publisher' "
				f"; then "
				f"echo '[launch] /realsense_color_publisher already running; skipping start'; "
				f"else exec python3 {os.path.join(robo_realsense_dir, 'ros_stream.py')}; fi"
			),
		],
		condition=IfCondition(PythonExpression(["'", camera_mode, "' == 'color'"])),
		output='screen',
		emulate_tty=True,
	)

	return LaunchDescription([
		cleanup_arg,
		camera_mode_arg,
		kill_line_follow_V2,
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
				ros_stream_color_only,
				ros_stream_with_depth,
			],
		),
		TimerAction(
			period=6.0,
			actions=[
				line_follow_V2,
			],
		),
	])
