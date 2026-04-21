from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amy_test',
            executable='tape_vision_node',
            name='tape_vision_node',
            output='screen',
            parameters=[
                {'image_topic': '/camera/color/image_raw'},
                {'debug_image_topic': '/tape_debug/image'}
            ]
        ),
        Node(
            package='amy_test',
            executable='tape_control_node',
            name='tape_control_node',
            output='screen',
            parameters=[
                {'enable_motor_control': True}
            ]
        ),
        Node(
            package='amy_test',
            executable='tape_recovery_node',
            name='tape_recovery_node',
            output='screen',
            parameters=[
                {'enable_motor_control': True}
            ]
        )
    ])
