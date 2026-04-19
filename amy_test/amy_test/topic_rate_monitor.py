#!/usr/bin/env python3
"""
Topic Rate Monitor - Non-invasive ROS 2 publication rate monitoring
Subscribes to critical topics and measures publication rates, jitter, and timing statistics
Publishes aggregated statistics to /monitoring/rate_stats

Part of the latency monitoring infrastructure for RTES rover application
All monitoring code is non-invasive and contained in amy_test package
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from custom_messages.msg import RateStats
from collections import deque
import time
import statistics


class TopicRateMonitor(Node):
    """Monitor publication rates and jitter for critical topics"""
    
    def __init__(self):
        super().__init__('topic_rate_monitor')
        
        # Measurement window (seconds) - report statistics every N seconds
        self.declare_parameter('window_duration', 5.0)
        self.window_duration = self.get_parameter('window_duration').value
        
        # Topics to monitor
        self.declare_parameter('monitor_camera', True)
        self.declare_parameter('monitor_cmd_vel', True)
        self.declare_parameter('monitor_imu', True)
        
        # Storage for timestamps (topic_name -> deque of timestamps)
        self.timestamps = {
            'camera': deque(maxlen=1000),
            'cmd_vel': deque(maxlen=1000),
            'imu_accel': deque(maxlen=1000),
        }
        
        # Subscribers for monitored topics
        if self.get_parameter('monitor_camera').value:
            self.camera_sub = self.create_subscription(
                Image,
                '/camera/color/image_raw',
                lambda msg: self._record_timestamp('camera', msg),
                10
            )
        
        if self.get_parameter('monitor_cmd_vel').value:
            self.cmd_vel_sub = self.create_subscription(
                Twist,
                '/cmd_vel',
                lambda msg: self._record_timestamp('cmd_vel', msg),
                10
            )
        
        if self.get_parameter('monitor_imu').value:
            self.imu_sub = self.create_subscription(
                Imu,
                '/imu/accel',
                lambda msg: self._record_timestamp('imu_accel', msg),
                10
            )
        
        # Publisher for aggregated statistics
        self.stats_pub = self.create_publisher(
            RateStats,
            '/monitoring/rate_stats',
            10
        )
        
        # Timer to compute and publish statistics
        self.stats_timer = self.create_timer(
            self.window_duration,
            self._publish_statistics
        )
        
        self.get_logger().info(
            f'Topic Rate Monitor initialized (window={self.window_duration}s)'
        )
    
    def _record_timestamp(self, topic_name, msg):
        """Record timestamp for a received message"""
        # Use ROS message timestamp if available, otherwise wall clock
        if hasattr(msg, 'header') and msg.header.stamp.sec > 0:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            timestamp = time.time()
        
        self.timestamps[topic_name].append(timestamp)
    
    def _compute_rate_stats(self, topic_name):
        """Compute rate statistics for a topic over the measurement window"""
        timestamps = list(self.timestamps[topic_name])
        
        if len(timestamps) < 2:
            return None
        
        # Filter to measurement window
        current_time = time.time()
        window_start = current_time - self.window_duration
        recent_timestamps = [t for t in timestamps if t >= window_start]
        
        if len(recent_timestamps) < 2:
            return None
        
        # Calculate inter-message intervals (deltas)
        deltas = [recent_timestamps[i+1] - recent_timestamps[i] 
                  for i in range(len(recent_timestamps) - 1)]
        
        if not deltas:
            return None
        
        # Rate statistics (Hz)
        rates_hz = [1.0 / delta if delta > 0 else 0.0 for delta in deltas]
        mean_rate = statistics.mean(rates_hz) if rates_hz else 0.0
        min_rate = min(rates_hz) if rates_hz else 0.0
        max_rate = max(rates_hz) if rates_hz else 0.0
        
        # Jitter statistics (ms) - variability in inter-message timing
        deltas_ms = [d * 1000.0 for d in deltas]
        mean_jitter = statistics.mean(deltas_ms) if deltas_ms else 0.0
        max_jitter = max(deltas_ms) if deltas_ms else 0.0
        
        return {
            'mean_rate_hz': mean_rate,
            'min_rate_hz': min_rate,
            'max_rate_hz': max_rate,
            'mean_jitter_ms': mean_jitter,
            'max_jitter_ms': max_jitter,
            'message_count': len(recent_timestamps),
        }
    
    def _publish_statistics(self):
        """Compute and publish rate statistics for all monitored topics"""
        for topic_name in self.timestamps.keys():
            stats = self._compute_rate_stats(topic_name)
            
            if stats is None:
                continue
            
            # Create and publish RateStats message
            msg = RateStats()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.topic_name = f'/{topic_name.replace("_", "/")}'
            msg.mean_rate_hz = stats['mean_rate_hz']
            msg.min_rate_hz = stats['min_rate_hz']
            msg.max_rate_hz = stats['max_rate_hz']
            msg.mean_jitter_ms = stats['mean_jitter_ms']
            msg.max_jitter_ms = stats['max_jitter_ms']
            msg.message_count = stats['message_count']
            msg.window_duration_s = self.window_duration
            
            self.stats_pub.publish(msg)
            
            self.get_logger().info(
                f'{topic_name}: {stats["mean_rate_hz"]:.1f} Hz '
                f'(jitter: {stats["mean_jitter_ms"]:.1f} ms, '
                f'n={stats["message_count"]})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TopicRateMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
