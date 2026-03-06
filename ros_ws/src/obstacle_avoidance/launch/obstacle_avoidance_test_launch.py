from launch import LaunchDescription
from launch_ros.actions import Node


#  ros2 run rplidar_ros rplidar_node --ros-args --remap scan_mode:=Standard --ros-args --remap serial_port:=/dev/ttyUSB0
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robo_rover',
            namespace='robo_node',
            executable='robo_node',
            name='robo_node'
        ),
        Node(
            package='rplidar_ros',
            namespace='rplidar_node',
            executable='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'scan_mode': 'Standard',
            }],
            name='rplidar_node'
        ),
        Node(
            package='obstacle_avoidance',
            executable='collision',
            name='collision_executable',
        )
    ])
