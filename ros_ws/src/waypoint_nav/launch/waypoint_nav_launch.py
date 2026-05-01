#!/usr/bin/env python3

# Assumes SLAM is already running (launched via SLAM_dependency_launch.py).
# Starts the TF bridge, map relay, Nav2, and waypoint runner.

import os
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share = get_package_share_directory('waypoint_nav')
    params_file = os.path.join(package_share, 'nav2_params.yaml')
    run_waypoints_script = os.path.join(package_share, 'run_waypoints.py')
    map_relay_script = os.path.join(package_share, 'map_relay.py')
    workspace_root = os.path.abspath(os.path.join(package_share, '..', '..', '..', '..'))
    slam_tf_bridge_script = os.path.join(workspace_root, 'src', 'SLAM', 'slam_tf_bridge.py')

    # Converts /slam/pose → TF map→base_link for Nav2
    tf_bridge = ExecuteProcess(
        cmd=['python3', slam_tf_bridge_script],
        output='screen',
        emulate_tty=True,
    )

    # Republishes /slam/lidar_map → /map with TRANSIENT_LOCAL durability for Nav2 costmap
    map_relay = ExecuteProcess(
        cmd=['python3', map_relay_script],
        output='screen',
        emulate_tty=True,
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

    # Waits for bt_navigator to be active before sending waypoints
    waypoint_runner = ExecuteProcess(
        cmd=['python3', run_waypoints_script],
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