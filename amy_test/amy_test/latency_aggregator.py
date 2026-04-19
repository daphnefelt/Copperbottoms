#!/usr/bin/env python3
"""
Latency Aggregator - Combines latency statistics from vision and motor monitors
Calculates total end-to-end latency (camera → motor actuation)
Validates against <100ms requirement for real-time control

Part of the latency monitoring infrastructure for RTES rover application
All monitoring code is non-invasive and contained in amy_test package
"""

import rclpy
from rclpy.node import Node
from custom_messages.msg import LatencyStats
from collections import defaultdict


class LatencyAggregator(Node):
    """Aggregate latency statistics from multiple monitoring nodes"""
    
    def __init__(self):
        super().__init__('latency_aggregator')
        
        # Measurement parameters
        self.declare_parameter('total_deadline_ms', 100.0)  # end-to-end deadline
        
        self.total_deadline_ms = self.get_parameter('total_deadline_ms').value
        
        # Storage for latest statistics from each measurement type
        self.latest_stats = {}  # measurement_type -> LatencyStats
        
        # Subscribers for latency statistics
        self.vision_sub = self.create_subscription(
            LatencyStats,
            '/monitoring/vision_latency',
            lambda msg: self._stats_callback('vision', msg),
            10
        )
        
        self.motor_sub = self.create_subscription(
            LatencyStats,
            '/monitoring/motor_latency',
            lambda msg: self._stats_callback('motor', msg),
            10
        )
        
        # Publisher for total end-to-end latency
        self.total_pub = self.create_publisher(
            LatencyStats,
            '/monitoring/total_latency',
            10
        )
        
        # Timer to compute and publish aggregated statistics
        self.aggregate_timer = self.create_timer(
            5.0,  # Aggregate every 5 seconds
            self._publish_aggregated_stats
        )
        
        self.get_logger().info(
            f'Latency Aggregator initialized (total_deadline={self.total_deadline_ms}ms)'
        )
    
    def _stats_callback(self, measurement_type, msg):
        """Store latest statistics for each measurement type"""
        self.latest_stats[measurement_type] = msg
        
        # Log received statistics
        self.get_logger().debug(
            f'Received {measurement_type} stats: '
            f'{msg.mean_latency_ms:.1f}ms ± {msg.stddev_latency_ms:.1f}ms '
            f'(miss={msg.miss_rate_percent:.1f}%)'
        )
    
    def _publish_aggregated_stats(self):
        """
        Compute and publish total end-to-end latency
        Total = vision latency + motor latency
        """
        # Check if we have both vision and motor statistics
        if 'vision' not in self.latest_stats or 'motor' not in self.latest_stats:
            self.get_logger().warn(
                'Missing latency statistics - waiting for all monitors to report'
            )
            return
        
        vision_stats = self.latest_stats['vision']
        motor_stats = self.latest_stats['motor']
        
        # Calculate total latency (sum of vision and motor)
        total_mean = vision_stats.mean_latency_ms + motor_stats.mean_latency_ms
        total_min = vision_stats.min_latency_ms + motor_stats.min_latency_ms
        total_max = vision_stats.max_latency_ms + motor_stats.max_latency_ms
        
        # Combined standard deviation (assuming independence)
        # σ_total = sqrt(σ_vision² + σ_motor²)
        total_stddev = (vision_stats.stddev_latency_ms**2 + 
                       motor_stats.stddev_latency_ms**2) ** 0.5
        
        # Estimate total miss rate (conservative: max of individual miss rates)
        total_miss_rate = max(vision_stats.miss_rate_percent, motor_stats.miss_rate_percent)
        
        # Sample count (minimum of both)
        total_samples = min(vision_stats.sample_count, motor_stats.sample_count)
        
        # Create and publish aggregated LatencyStats
        msg = LatencyStats()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.measurement_type = 'total'
        msg.mean_latency_ms = total_mean
        msg.min_latency_ms = total_min
        msg.max_latency_ms = total_max
        msg.stddev_latency_ms = total_stddev
        msg.sample_count = total_samples
        msg.miss_rate_percent = total_miss_rate
        msg.deadline_ms = self.total_deadline_ms
        
        self.total_pub.publish(msg)
        
        # Log aggregated statistics with status
        if total_mean < self.total_deadline_ms:
            if total_mean < self.total_deadline_ms * 0.7:  # <70ms (good margin)
                status = '✓ EXCELLENT'
            else:  # 70-100ms (acceptable)
                status = '✓ GOOD'
        elif total_mean < self.total_deadline_ms * 1.2:  # 100-120ms (marginal)
            status = '⚠ MARGINAL'
        else:  # >120ms (bad)
            status = '✗ BAD'
        
        self.get_logger().info(
            f'\n'
            f'═══════════════════════════════════════════════════════════\n'
            f'  END-TO-END LATENCY SUMMARY\n'
            f'═══════════════════════════════════════════════════════════\n'
            f'  Vision (camera → cmd_vel): {vision_stats.mean_latency_ms:6.1f}ms '
            f'± {vision_stats.stddev_latency_ms:.1f}ms\n'
            f'  Motor (cmd_vel → actuation): {motor_stats.mean_latency_ms:6.1f}ms '
            f'± {motor_stats.stddev_latency_ms:.1f}ms\n'
            f'  ─────────────────────────────────────────────────────────\n'
            f'  TOTAL (camera → actuation): {total_mean:6.1f}ms '
            f'± {total_stddev:.1f}ms {status}\n'
            f'  Range: {total_min:.1f}ms - {total_max:.1f}ms\n'
            f'  Deadline: {self.total_deadline_ms:.1f}ms '
            f'({"PASS" if total_mean < self.total_deadline_ms else "FAIL"})\n'
            f'═══════════════════════════════════════════════════════════'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LatencyAggregator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
