import reclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageSubscriber(Node):
	def __init__(self):
		super().__init__('image_subscriber')
		self.subscription = self.create_subscription(Image, '/camera/image',
			self.listener_callback,
			10
		)

	def listener_callback(self, msg):
		self.get_logger().info('Received image frame')

def main():
	rclpy.init()
	node = ImageSubscriber()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


