#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
	workspace_src_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
	camera_nav_dir = os.path.join(workspace_src_dir, 'Camera_Navigation')
	robo_realsense_dir = os.path.expanduser('~/robo_realsense')

	"""
	To run
	ros2 launch src/Camera_Navigation/line_follow_V2.launch.py
	"""

	# Start rover in detached mode so Ctrl+C on this launch does not stop it.
	rover_node = ExecuteProcess(
		cmd=[
			'bash',
			'-lc',
			(
				"if ros2 node list 2>/dev/null | grep -Fxq '/rover_node' "
				"&& ros2 topic info /rover/armed 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then "
				"echo '[launch] /rover_node already running; skipping start'; "
				"else nohup ros2 run robo_rover rover_node --ros-args -r __node:=rover_node "
				">/tmp/rover_node.log 2>&1 & "
				"echo '[launch] started /rover_node in background (log: /tmp/rover_node.log)'; fi"
			),
		],
		output='screen',
		emulate_tty=True,
	)

	# Color-only RealSense launcher (no depth stream), also detached.
	ros_stream_color_only = ExecuteProcess(
		cmd=[
			'bash',
			'-lc',
			(
				f"if ros2 node list 2>/dev/null | grep -Fxq '/realsense_color_publisher' "
				f"; then "
				f"echo '[launch] /realsense_color_publisher already running; skipping start'; "
				f"else nohup python3 {os.path.join(robo_realsense_dir, 'ros_stream.py')} "
				f">/tmp/realsense_color_publisher.log 2>&1 & "
				f"echo '[launch] started /realsense_color_publisher in background (log: /tmp/realsense_color_publisher.log)'; fi"
			),
		],
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

	return LaunchDescription([
		TimerAction(
			period=0.5,
			actions=[
				rover_node,
			],
		),
		TimerAction(
			period=1.0,
			actions=[
				ros_stream_color_only,
			],
		),
		TimerAction(
			period=2.0,
			actions=[
				line_follow_V2,
			],
		),
	])