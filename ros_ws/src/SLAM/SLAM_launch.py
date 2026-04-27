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
	# need robo_rover, camera,  rplidar, landmark sensor node, ekf
	workspace_src_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
	SLAM_dir = os.path.join(workspace_src_dir, 'SLAM')
	fastsam_dir = os.path.expanduser('~/robo_realsense/fastsam')

	node_names = []
	processes = []

	landmark_based_slam = ExecuteProcess(
		cmd=['python3', os.path.join(SLAM_dir, 'landmark_based_slam.py')],
		output='screen',
		emulate_tty=True,
	)
	node_names.append(('ekf_slam', '/ekf_slam'))
	processes.append(landmark_based_slam)

	sensor_slam = ExecuteProcess(
		cmd=['python3', os.path.join(SLAM_dir, 'sensor.py')],
		output='screen',
		emulate_tty=True,
	)
	node_names.append(('landmark_sensor', '/landmark_sensor'))
	processes.append(sensor_slam)

	rover_node = Node(
		package='robo_rover',
		executable='rover_node',
		name='rover_node',
		output='screen',
		emulate_tty=True,
	)
	node_names.append(('rover_node', '/rover_node'))
	processes.append(rover_node)



	lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen',
        emulate_tty=True,
    )
	node_names.append(('rplidar_node', '/rplidar_node'))
	processes.append(lidar_node)
	

	ros_stream_with_depth = ExecuteProcess(
		cmd=['python3', os.path.join(fastsam_dir, 'desktop_stream.py')],
		output='screen',
		emulate_tty=True,
	)
	node_names.append(('realsense_color_depth_publisher', '/realsense_color_depth_publisher'))
	processes.append(ros_stream_with_depth)


	running_nodes = _get_running_node_names()
	launch_actions = []

	# only launch nodes that aren't already running
	for i in range(len(node_names)):
		if not _is_any_running(running_nodes, node_names[i]):
			launch_actions.append(processes[i])
		


	return LaunchDescription(launch_actions)
