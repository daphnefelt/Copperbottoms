#!/usr/bin/env python3
"""
Vision Latency Monitor - Measures camera capture to cmd_vel publication latency
Uses message_filters to correlate camera frames with resulting cmd_vel commands
Measures vision processing pipeline performance (expected: 30-40ms based on FIFO baseline)

Part of the latency monitoring infrastructure for RTES rover application
All monitoring code is non-invasive and contained in amy_test package
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from custom_messages.msg import LatencyStats
from message_filters import ApproximateTimeSynchronizer, Subscriber
from collections import deque
import statistics


class VisionLatencyMonitor(Node):
    """Monitor latency from camera frame to cmd_vel publication"""
    
    def __init__(self):
        super().__init__('vision_latency_monitor')
        
        # Measurement parameters
        self.declare_parameter('measurement_window', 5.0)  # seconds
        self.declare_parameter('sync_slop', 0.1)  # max time difference for sync (100ms)
        self.declare_parameter('deadline_ms', 50.0)  # vision processing deadline
        
        self.measurement_window = self.get_parameter('measurement_window').value
        self.sync_slop = self.get_parameter('sync_slop').value
        self.deadline_ms = self.get_parameter('deadline_ms').value
        
        # Storage for latency measurements
        self.latencies = deque(maxlen=1000)
        
        # Synchronized subscribers for camera and cmd_vel
        self.image_sub = Subscriber(
            self,
            Image,
            '/camera/color/image_raw'
        )
        
        self.cmd_vel_sub = Subscriber(
            self,
            Twist,
            '/cmd_vel'
        )
        
        # ApproximateTimeSynchronizer to correlate camera frames with cmd_vel
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.cmd_vel_sub],
            queue_size=10,
            slop=self.sync_slop
        )
        self.sync.registerCallback(self._synchronized_callback)
        
        # Publisher for latency statistics
        self.stats_pub = self.create_publisher(
            LatencyStats,
            '/monitoring/vision_latency',
            10
        )
        
        # Timer to compute and publish statistics
        self.stats_timer = self.create_timer(
            self.measurement_window,
            self._publish_statistics
        )
        
        self.get_logger().info(
            f'Vision Latency Monitor initialized '
            f'(window={self.measurement_window}s, deadline={self.deadline_ms}ms)'
        )
    
    def _synchronized_callback(self, image_msg, cmd_vel_msg):
        """
        Callback when camera frame and cmd_vel are approximately synchronized
        Calculate latency from camera timestamp to cmd_vel timestamp
        """
        # Extract timestamps from message headers
        camera_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        
        # cmd_vel doesn't have header, so use current time as proxy
        # In production, line_follow should stamp cmd_vel messages with camera timestamp
        current_time = self.get_clock().now()
        cmd_vel_time = current_time.nanoseconds * 1e-9
        
        # Calculate latency in milliseconds
        # This is an approximation - ideally cmd_vel should carry camera timestamp
        latency_ms = (cmd_vel_time - camera_time) * 1000.0
        
        # Sanity check - latency should be positive and reasonable (<500ms)
        if 0 < latency_ms < 500:
            self.latencies.append(latency_ms)
        else:
            self.get_logger().warn(
                f'Unrealistic latency: {latency_ms:.1f}ms '
                f'(camera: {camera_time:.3f}, cmd_vel: {cmd_vel_time:.3f})'
            )
    
    def _publish_statistics(self):
        """Compute and publish vision latency statistics"""
        if len(self.latencies) < 2:
            self.get_logger().warn('Insufficient latency samples')
            return
        
        latencies_list = list(self.latencies)
        
        # Compute statistics
        mean_latency = statistics.mean(latencies_list)
        min_latency = min(latencies_list)
        max_latency = max(latencies_list)
        stddev_latency = statistics.stdev(latencies_list) if len(latencies_list) > 1 else 0.0
        
        # Miss rate - percentage exceeding deadline
        misses = sum(1 for lat in latencies_list if lat > self.deadline_ms)
        miss_rate = (misses / len(latencies_list)) * 100.0
        
        # Create and publish LatencyStats message
        msg = LatencyStats()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.measurement_type = 'vision'
        msg.mean_latency_ms = mean_latency
        msg.min_latency_ms = min_latency
        msg.max_latency_ms = max_latency
        msg.stddev_latency_ms = stddev_latency
        msg.sample_count = len(latencies_list)
        msg.miss_rate_percent = miss_rate
        msg.deadline_ms = self.deadline_ms
        
        self.stats_pub.publish(msg)
        
        # Log statistics
        status = '✓ GOOD' if miss_rate < 5.0 else '⚠ MARGINAL' if miss_rate < 20.0 else '✗ BAD'
        self.get_logger().info(
            f'Vision latency: {mean_latency:.1f}ms ± {stddev_latency:.1f}ms '
            f'(min={min_latency:.1f}, max={max_latency:.1f}, '
            f'miss={miss_rate:.1f}%, n={len(latencies_list)}) {status}'
        )
        
        # Clear measurements for next window
        self.latencies.clear()


def main(args=None):
    rclpy.init(args=args)
    node = VisionLatencyMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
