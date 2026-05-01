" Dummy code based off slam map "
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist 
from time import sleep

class dummySarina(Node):
    def __init__(self):
        super().__init__('dummy_sarina')

        # Publishers / Subscribers
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.angle_pub = self.create_publisher(Float64, '/line_angle', 10)

        # If you still want to listen to /angle_goal
        self.angle_goal_sub = self.create_subscription(
            Float64,
            '/angle_goal',
            self.angle_goal_callback,
            10
        )

        # PID gains
        self.kp = 0.20
        self.ki = 0.01
        self.kd = 0.05

        # PID state
        self.prev_error = 0.0
        self.integral = 0.0
        self.dt = 0.1

        # Start the drive sequence immediately
        self.get_logger().info('Starting dummy_sarina drive sequence')
        self.loop()

    def angle_goal_callback(self, msg):
        self.get_logger().info(f"Received /angle_goal: {msg.data:.2f}")

    def loop(self):
        sequence = [
            (0.5, 0.0, 10.0, 'forward'),
            (0.3, -0.5, 4.0, 'turn right'),
            (0.5, 0.0, 20.0, 'forward'),
            (0.3, -0.5, 4.0, 'turn right'),
            (0.5, 0.0, 30.0, 'forward'),
            (0.3, -0.5, 4.0, 'turn right'),
            (0.5, 0.0, 20.0, 'forward'),
            (0.3, -0.5, 4.0, 'turn right'),
            (0.5, 0.0, 10.0, 'forward'),
        ]

        for linear_x, angular_z, duration, desc in sequence:
            self.get_logger().info(f'Publishing {desc} for {duration:.0f}s')
            twist_msg = Twist()
            twist_msg.linear.x = linear_x
            twist_msg.angular.z = angular_z
            self.pub.publish(twist_msg)
            sleep(duration)

        self.get_logger().info('Drive sequence complete, stopping robot')
        stop_msg = Twist()
        self.pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = dummySarina()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
