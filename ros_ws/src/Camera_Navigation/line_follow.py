### Dark Lane Following ###

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        image_topic = '/camera/color/image_raw'
        cmd_vel_topic = '/cmd_vel'

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.get_logger().info(
            f'Line follower node started. image_topic={image_topic}, cmd_vel_topic={cmd_vel_topic}'
        )

    def image_callback(self, msg: Image):
        # We will build this logic together step by step.
        pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LineFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


    


