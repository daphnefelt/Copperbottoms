#!/usr/bin/env python3
"""
Minimal launch — no Nav2.
Starts EKF SLAM + simple waypoint driver only.
Run SLAM_dependency_launch.py first (rplidar, rf2o, camera, etc.).
"""
import os
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess

def generate_launch_description():
    this_dir = os.path.dirname(os.path.abspath(__file__))          # .../waypoint_nav/launch/
    pkg_dir  = os.path.abspath(os.path.join(this_dir, '..'))       # .../waypoint_nav/
    ws_src   = os.path.abspath(os.path.join(pkg_dir, '..'))        # .../ros_ws/src/
    slam_dir = os.path.join(ws_src, 'SLAM')

    ekf_slam = ExecuteProcess(
        cmd=['python3', os.path.join(slam_dir, 'landmark_based_slam.py')],
        output='screen',
        emulate_tty=True,
    )

    driver = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_dir, 'simple_waypoint_driver.py')],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([ekf_slam, driver])


def main() -> int:
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == '__main__':
    raise SystemExit(main())