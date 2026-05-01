#!/usr/bin/env python3

# Assumes SLAM is already running (launched via SLAM_dependency_launch.py).
# Starts the TF bridge, map relay, Nav2, and waypoint runner.

import os
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    slam_dir  = os.path.abspath(os.path.join(this_dir, '..'))
    params_file = os.path.join(this_dir, 'nav2_params.yaml')

    # Converts /slam/pose → TF map→base_link for Nav2
    tf_bridge = ExecuteProcess(
        cmd=['python3', os.path.join(slam_dir, 'slam_tf_bridge.py')],
        output='screen',
        emulate_tty=True,
    )

    # Mirrors /slam/lidar_map → /map without touching the original topic
    map_relay = Node(
        package='topic_tools',
        executable='relay',
        name='lidar_map_relay',
        arguments=['/slam/lidar_map', '/map'],
        output='screen',
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py',
            )
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false',
        }.items(),
    )

    # Waits for Nav2 to be active before sending waypoints
    waypoint_runner = ExecuteProcess(
        cmd=['python3', os.path.join(this_dir, 'run_waypoints.py')],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([tf_bridge, map_relay, nav2, waypoint_runner])


def main() -> int:
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == '__main__':
    raise SystemExit(main())
