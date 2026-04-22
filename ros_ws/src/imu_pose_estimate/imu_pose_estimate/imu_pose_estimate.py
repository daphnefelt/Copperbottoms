import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
import time
import threading
from pymavlink import mavutil
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Vector3
from custom_messages.msg import Slow
from custom_messages.msg import ImuBundled

class ImuPoseEstimate(Node):
    def __init__(self):
        super().__init__('rover_node')

        # Subscribers - MARY consider appending a timestamp for more accurate time estimates
        # versus late estimates
        queue_size = 100
        self.cmd_sub = self.create_subscription(
            Vector3, 'imu/gyro', self.imu_gyro, queue_size)
        self.imu_sub = self.create_subscription(ImuBundled, 'imu/imu_bundled', queue_size)
    

    def imu_accel(self, vec: Vector3):
        pass
    def imu_gyro(self,vec: Vector3):
        pass


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ImuPoseEstimate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
