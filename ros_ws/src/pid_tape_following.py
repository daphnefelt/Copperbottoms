#!/usr/bin/env python3
# read in topic /angle_goal and use it to control the robot to follow the tape
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
from collections import deque


class PaperFollower(Node):
    def __init__(self):
        super().__init__('paper_follower')
        self.previous_errors = deque(maxlen=10)
        self.update_rate = 10.0  # Hz
        self.previous_error = None
        self.should_stop = False

        self.angle_goal_sub = self.create_subscription(
            Float64, '/angle_goal', self.angle_goal_callback, int(self.update_rate))
        self.stop_sub = self.create_subscription(
            Bool, '/tape_stop', self.stop_callback, 10)
        self.drive_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def stop_callback(self, msg: Bool):
        self.should_stop = msg.data
        if self.should_stop:
            self.get_logger().warn("Stop signal received — halting robot.")
            twist = Twist()  # all zeros = full stop
            self.drive_pub.publish(twist)

    def angle_goal_callback(self, msg):
        if self.should_stop:
            return  # ignore angle goals while stopped

        angle_goal = -msg.data
        self.get_logger().info(f"Received angle_goal: {angle_goal:.4f} rad")

        p = 0.2
        i = 0.0
        d = 0.0

        if self.previous_error is None:
            derivative_error = 0.0
        else:
            dt = 1.0 / self.update_rate
            derivative_error = (angle_goal - self.previous_error) / dt

        self.previous_errors.append(angle_goal)
        self.previous_error = angle_goal

        integral_error = sum(self.previous_errors)

        turn_amount = p * angle_goal + i * integral_error + d * derivative_error
        self.get_logger().info(
            f"turn_amount={turn_amount:.4f}  "
            f"p_term={p * angle_goal:.4f}  "
            f"i_term={i * integral_error:.4f}  "
            f"d_term={d * derivative_error:.4f}"
        )

        twist = Twist()
        twist.linear.x = 0.15
        twist.angular.z = turn_amount
        self.drive_pub.publish(twist)


def main():
    try:
        rclpy.init()
        paper_follower = PaperFollower()
        rclpy.spin(paper_follower)
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        paper_follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
