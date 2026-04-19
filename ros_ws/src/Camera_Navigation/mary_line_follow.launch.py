#!/usr/bin/env python3

import os
import subprocess
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


def _is_any_running(running_names: set[str], aliases: tuple[str, ...]) -> bool:
	for alias in aliases:
		if alias in running_names:
			return True
		if alias.startswith('/') and alias[1:] in running_names:
			return True
		if (not alias.startswith('/')) and ('/' + alias) in running_names:
			return True
	return False


def generate_launch_description():
	workspace_src_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
	camera_nav_dir = os.path.join(workspace_src_dir, 'Camera_Navigation')
	fastsam_dir = os.path.expanduser('~/robo_realsense/fastsam')

	line_follow = ExecuteProcess(
		cmd=['python3', os.path.join(camera_nav_dir, 'mary_line_follow.py')],
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


	running_nodes = _get_running_node_names()
	launch_actions = []
	# line_follow.py creates this node name.
	if not _is_any_running(running_nodes, ('mary_line_follow', '/mary_line_follow')):
		launch_actions.append(line_follow)
		print("mary not running")
	else:
		print("mary already running")

	# ros_stream_with_depth.py creates this node name.
	if not _is_any_running(running_nodes, ('realsense_color_depth_publisher', '/realsense_color_depth_publisher')):
		launch_actions.append(ros_stream_with_depth)

	if not _is_any_running(running_nodes, ('rover_node', '/rover_node')):
		launch_actions.append(rover_node)


	return LaunchDescription(launch_actions)
