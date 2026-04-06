from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package = 'robo_realsense',
			node_namespace= 'realsense',
			node_executable = 'realsense_node',
			node_name = 'camera'
		)
	])
