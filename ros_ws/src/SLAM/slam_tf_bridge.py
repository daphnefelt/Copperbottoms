#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class SlamTfBridge(Node):
    def __init__(self):
        super().__init__('slam_tf_bridge')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            PoseWithCovarianceStamped, '/slam/pose', self.pose_cb, 10)

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SlamTfBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
