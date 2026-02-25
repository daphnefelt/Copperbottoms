# read in topic /angle_goal and use it to control the robot to follow the paper

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from collections import deque

class PaperFollower(Node):
    def __init__(self):
        super().__init__('paper_follower')
        self.previous_errors = deque(maxlen=10)  # Store last 10 errors for integral term
        self.update_rate = 10.0  # Hz
        self.angle_goal_sub = self.create_subscription(
            Float64, '/angle_goal', self.angle_goal_callback, int(self.update_rate))
        self.drive_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def angle_goal_callback(self, msg):
        angle_goal = msg.data
        self.get_logger().info(f"Received angle_goal: {angle_goal}")
        # Angle goal is with respect to the robot's current heading, so it is already the error
        p = 1.0  # proportional
        i = 0.0  # integral
        d = 0.0  # derivative

        self.previous_errors.append(angle_goal)
        integral_error = sum(self.previous_errors)
        derivative_error = (angle_goal - self.previous_errors[-2]) / (1.0 / self.update_rate) if len(self.previous_errors) > 1 else 0.0

        turn_amount = p * angle_goal + i * integral_error + d * derivative_error
        self.get_logger().info(f"Calculated turn_amount: {turn_amount}")

        # now actually command using the robo rover node
        twist = Twist()
        twist.linear.x = 0.5  # constant forward speed to make turn actually happen
        twist.angular.z = turn_amount # need to check signs here
        self.drive_pub.publish(twist)

if __name__ == '__main__':
    try:
        rclpy.init()
        paper_follower = PaperFollower()
        rclpy.spin(paper_follower)
    except rclpy.exceptions.ROSInterruptException:
        pass