#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DriveTestNode(Node):

    def __init__(self):
        super().__init__('drive_test_node')

        # -- same variables as hallway_center_node --
        self.forward_speed    = 0.25
        self.OB_forward_speed = 0.2
        self.turn_rate        = 1

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.commands = [
            ('FORWARD',     2.0, -self.forward_speed,     0.0),
            ('STOP',        1.0,  0.0,                    0.0),
            ('BACKWARD',    2.0,  self.OB_forward_speed,  0.0),
            ('STOP',        1.0,  0.0,                    0.0),
            ('TURN LEFT',   2.0, -self.forward_speed,     self.turn_rate),
            ('STOP',        1.0,  0.0,                    0.0),
            ('TURN RIGHT',  2.0, -self.forward_speed,    -self.turn_rate),
            ('STOP',        1.0,  0.0,                    0.0),
        ]

        self.cmd_index   = 0
        self.phase_start = time.time()
        self.timer       = self.create_timer(0.1, self.tick)
        self.get_logger().info('Drive test node started.')

    def tick(self):
        if self.cmd_index >= len(self.commands):
            self.get_logger().info('Sequence complete.')
            self.vel_pub.publish(Twist())
            raise SystemExit

        label, duration, lin_x, ang_z = self.commands[self.cmd_index]
        elapsed = time.time() - self.phase_start

        if elapsed < duration:
            twist = Twist()
            twist.linear.x  = lin_x
            twist.angular.z = ang_z
            self.vel_pub.publish(twist)
            self.get_logger().info(
                f'{label}  lin.x={lin_x:+.2f}  ang.z={ang_z:+.2f}  ({elapsed:.1f}/{duration:.1f}s)',
                throttle_duration_sec=0.5
            )
        else:
            self.get_logger().info(f'{label} done.')
            self.cmd_index  += 1
            self.phase_start = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = DriveTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
