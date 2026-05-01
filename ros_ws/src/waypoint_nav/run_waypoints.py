#!/usr/bin/env python3
import math, os
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

SLAM_OFFSET_X = 0.0
SLAM_OFFSET_Y = 0.0

def load_waypoints(filepath, every_n=4):
    poses = []
    with open(filepath) as f:
        for line in f:
            parts = list(map(float, line.strip().split()))
            x, y, yaw = parts[0], parts[1], parts[2]  # ignore timestamp if present
            poses.append((x + SLAM_OFFSET_X, y + SLAM_OFFSET_Y, yaw))
    sampled = poses[::every_n]
    if not sampled:
        return sampled
    # Skip all initial waypoints at the starting position; NavFn returns empty plan for start==goal
    start_x, start_y = sampled[0][0], sampled[0][1]
    i = 0
    while i < len(sampled) and math.hypot(sampled[i][0] - start_x, sampled[i][1] - start_y) < 0.5:
        i += 1
    return sampled[i:]

def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

def main():
    rclpy.init()
    navigator = BasicNavigator()
    # waitUntilNav2Active() waits for AMCL which we don't use — wait for bt_navigator directly
    navigator._waitForNodeToActivate('bt_navigator')

    script_dir = os.path.dirname(os.path.abspath(__file__))
    waypoints_raw = load_waypoints(os.path.join(script_dir, 'pose_history_save.txt'))

    waypoints = []
    for x, y, yaw in waypoints_raw:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        waypoints.append(pose)

    navigator.followWaypoints(waypoints)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"At waypoint {feedback.current_waypoint + 1} / {len(waypoints)}")

    print("Result:", navigator.getResult())
    rclpy.shutdown()

if __name__ == '__main__':
    main()