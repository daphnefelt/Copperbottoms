from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


#  ros2 run rplidar_ros rplidar_node --ros-args --remap scan_mode:=Standard --ros-args --remap serial_port:=/dev/ttyUSB0
def generate_launch_description():
    # i'm being lazy about generating the folder - mary
    rplidar_launch_file = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/home/copperbottoms/code/Copperbottoms/ros_ws/src/rplidar_ros/launch/', 'rplidar_a1_launch.py'])
    return LaunchDescription([
        rplidar_launch_file,
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
            namespace='collision',
            executable='collision',
            name='collision_executable',
        )
    ])
