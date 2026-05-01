#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

class SlamTfBridge(Node):
    def __init__(self):
        super().__init__('slam_tf_bridge')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            PoseWithCovarianceStamped, '/slam/pose', self.pose_cb, 10)

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        slam_x = msg.pose.pose.position.x
        slam_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        slam_yaw = 2.0 * math.atan2(q.z, q.w)

        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
            odom_x = odom_to_base.transform.translation.x
            odom_y = odom_to_base.transform.translation.y
            oq = odom_to_base.transform.rotation
            odom_yaw = 2.0 * math.atan2(oq.z, oq.w)
        except Exception:
            # rf2o not ready yet — publish map→odom at current SLAM pose (odom origin = base_link)
            odom_x, odom_y, odom_yaw = 0.0, 0.0, 0.0

        # map_T_odom = map_T_base * inv(odom_T_base)
        # rotation: theta_mo = slam_yaw - odom_yaw
        # translation: p_mo = p_mb - R_mo * p_ob
        theta_mo = slam_yaw - odom_yaw
        x_mo = slam_x - (math.cos(theta_mo) * odom_x - math.sin(theta_mo) * odom_y)
        y_mo = slam_y - (math.sin(theta_mo) * odom_x + math.cos(theta_mo) * odom_y)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = x_mo
        t.transform.translation.y = y_mo
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(theta_mo / 2.0)
        t.transform.rotation.w = math.cos(theta_mo / 2.0)
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SlamTfBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()