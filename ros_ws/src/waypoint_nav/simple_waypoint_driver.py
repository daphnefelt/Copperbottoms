#!/usr/bin/env python3
"""
Simple waypoint driver — no Nav2 required.
Reads /slam/pose, steers toward waypoints via a P-controller, publishes /cmd_vel.
"""
import math
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

WAYPOINT_TOLERANCE  = 0.3   # metres — advance when within this distance
WAYPOINT_EVERY_N    = 8     # sample every Nth pose from the history file
MAX_LINEAR_VEL      = 0.3   # m/s
KP_ANGULAR          = 1.5   # rad/s per rad of heading error
MAX_ANGULAR_VEL     = 1.2   # rad/s cap
HEADING_SLOW_THRESH = 0.5   # rad — slow down when heading error is larger than this


def load_waypoints(filepath, every_n=WAYPOINT_EVERY_N):
    poses = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = list(map(float, line.split()))
            x, y, yaw = parts[0], parts[1], parts[2]
            poses.append((x, y, yaw))
    sampled = poses[::every_n]
    if not sampled:
        return sampled
    # Skip all initial waypoints at starting position to avoid trivial zero-distance goals
    start_x, start_y = sampled[0][0], sampled[0][1]
    i = 0
    while i < len(sampled) and math.hypot(sampled[i][0] - start_x, sampled[i][1] - start_y) < 0.5:
        i += 1
    return sampled[i:]


def angle_wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class SimpleWaypointDriver(Node):
    def __init__(self, waypoints):
        super().__init__('simple_waypoint_driver')
        self._waypoints = waypoints
        self._idx = 0
        self._pose = None

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/slam/pose', self._pose_cb, 10)
        self.create_timer(0.1, self._control_loop)   # 10 Hz

        self.get_logger().info(
            f'Simple waypoint driver ready — {len(waypoints)} waypoints loaded')

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        if self._pose is None:
            self.get_logger().info(
                f'SLAM start position: x={x:.3f}  y={y:.3f}  yaw={math.degrees(yaw):.1f}°')
            for i, (wx, wy, _) in enumerate(self._waypoints[:5]):
                dist = math.hypot(wx - x, wy - y)
                bearing = math.degrees(math.atan2(wy - y, wx - x))
                self.get_logger().info(
                    f'  WP{i+1}: x={wx:.3f}  y={wy:.3f}  dist={dist:.2f}m  bearing={bearing:.1f}°')
        self._pose = (x, y, yaw)

    def _control_loop(self):
        if self._pose is None:
            return

        if self._idx >= len(self._waypoints):
            self._stop()
            self.get_logger().info('All waypoints reached — stopping.', once=True)
            return

        rx, ry, ryaw = self._pose
        wx, wy, _ = self._waypoints[self._idx]

        dist = math.hypot(wx - rx, wy - ry)
        if dist < WAYPOINT_TOLERANCE:
            self.get_logger().info(
                f'Reached WP{self._idx + 1}/{len(self._waypoints)} '
                f'robot=({rx:.2f},{ry:.2f}) yaw={math.degrees(ryaw):.1f}°')
            self._idx += 1
            if self._idx < len(self._waypoints):
                nwx, nwy, _ = self._waypoints[self._idx]
                bearing = math.degrees(math.atan2(nwy - ry, nwx - rx))
                rel_bearing = math.degrees(math.atan2(
                    math.sin(math.radians(bearing) - ryaw),
                    math.cos(math.radians(bearing) - ryaw)))
                self.get_logger().info(
                    f'  → WP{self._idx + 1}: ({nwx:.2f},{nwy:.2f}) '
                    f'dist={math.hypot(nwx-rx,nwy-ry):.2f}m '
                    f'abs_bearing={bearing:.1f}° rel={rel_bearing:+.1f}°(+L/-R)')
            return

        # Desired heading toward waypoint
        desired_yaw = math.atan2(wy - ry, wx - rx)
        heading_err = angle_wrap(desired_yaw - ryaw)

        angular_z = max(-MAX_ANGULAR_VEL,
                        min(MAX_ANGULAR_VEL, KP_ANGULAR * heading_err))

        # Reduce forward speed when heading error is large
        if abs(heading_err) > HEADING_SLOW_THRESH:
            linear_x = MAX_LINEAR_VEL * max(0.0, math.cos(heading_err))
        else:
            linear_x = MAX_LINEAR_VEL

        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_pub.publish(msg)

    def _stop(self):
        self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    waypoints_file = os.path.join(script_dir, 'pose_history_save.txt')
    waypoints = load_waypoints(waypoints_file)
    if not waypoints:
        print(f'ERROR: no waypoints loaded from {waypoints_file}')
        return
    print(f'Loaded {len(waypoints)} waypoints')
    node = SimpleWaypointDriver(waypoints)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()