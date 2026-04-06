from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


#  ros2 run rplidar_ros rplidar_node --ros-args --remap scan_mode:=Standard --ros-args --remap serial_port:=/dev/ttyUSB0
def generate_launch_description():
    rplidar_launch_file = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/home/copperbottoms/code/Copperbottoms/ros_ws/src/rplidar_ros/launch/', 'rplidar_a1_launch.py']))
    return LaunchDescription([
        rplidar_launch_file,
        Node(
            package='robo_rover',
            executable='rover_node',
            name='robo_node'
        ),
        Node(
            package='obstacle_avoidance',
            executable='collision',
            name='collision_executable',
        ),
	ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '/cmd_vel', 'geometry/Twist', '"{linear: {x: 0.2}, angular: {z: 0.0}}"'],
            output='screen'
        )
    ])
