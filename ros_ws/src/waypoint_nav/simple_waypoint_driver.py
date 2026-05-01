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

WAYPOINT_TOLERANCE  = 0.35  # metres — advance when within this distance
MIN_WAYPOINT_DIST   = 0.15   # metres — minimum spacing between loaded waypoints
MIN_LINEAR_VEL      = 0.25  # m/s — minimum to actually overcome friction at low battery
MAX_LINEAR_VEL      = 0.35  # m/s
KP_ANGULAR          = 0.8   # rad/s per rad of heading error
MAX_ANGULAR_VEL     = 0.8   # rad/s cap


def angle_wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class SimpleWaypointDriver(Node):
    def __init__(self, waypoints_file):
        super().__init__('simple_waypoint_driver')
        self._waypoints = []
        self._idx = 0
        self._pose = None
        self._file = open(waypoints_file, 'r')
        self._start_pos = None   # first point read, used to skip starting cluster
        self._last_wp_pos = None # (x,y) of last accepted waypoint, for distance filter

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/slam/pose', self._pose_cb, 10)
        self.create_timer(0.1, self._control_loop)
        self.create_timer(1.0, self._poll_file)

        self._poll_file()  # load whatever is already in the file
        self.get_logger().info(
            f'Simple waypoint driver ready — watching {waypoints_file}')

    def _poll_file(self):
        added = 0
        while True:
            line = self._file.readline()
            if not line:
                break
            line = line.strip()
            if not line:
                continue
            try:
                parts = list(map(float, line.split()))
                x, y, yaw = parts[0], parts[1], parts[2]
            except (ValueError, IndexError):
                continue

            if self._start_pos is None:
                self._start_pos = (x, y)

            # Skip the initial cluster at the recording start position
            if self._last_wp_pos is None:
                sx, sy = self._start_pos
                if math.hypot(x - sx, y - sy) < 0.5:
                    continue

            # Reject points too close to the last accepted waypoint
            if self._last_wp_pos is not None:
                lx, ly = self._last_wp_pos
                if math.hypot(x - lx, y - ly) < MIN_WAYPOINT_DIST:
                    continue

            self._waypoints.append((x, y, yaw))
            self._last_wp_pos = (x, y)
            added += 1

        if added > 0:
            self.get_logger().info(
                f'+{added} waypoints  total={len(self._waypoints)}  '
                f'remaining={len(self._waypoints) - self._idx}')

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
            if len(self._waypoints) > 0:
                self.get_logger().info('All waypoints reached — waiting for more.', once=True)
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

        # Always maintain at least MIN_LINEAR_VEL (robot needs forward motion to turn).
        # Scale up toward MAX_LINEAR_VEL as heading error shrinks.
        heading_factor = max(0.0, math.cos(heading_err))
        linear_x = max(MIN_LINEAR_VEL, MAX_LINEAR_VEL * heading_factor)

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
    if not os.path.exists(waypoints_file):
        print(f'ERROR: waypoints file not found: {waypoints_file}')
        return
    node = SimpleWaypointDriver(waypoints_file)
    rclpy.spin(node)
    node._file.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()