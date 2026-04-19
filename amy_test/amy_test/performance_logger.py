#!/usr/bin/env python3
"""
Performance Logger - Logs latency and rate statistics to CSV files
Creates timestamped CSV files for offline analysis and performance tracking
Useful for long-term performance validation and regression testing

Part of the latency monitoring infrastructure for RTES rover application
All monitoring code is non-invasive and contained in amy_test package
"""

import rclpy
from rclpy.node import Node
from custom_messages.msg import LatencyStats, RateStats
import csv
import os
from datetime import datetime


class PerformanceLogger(Node):
    """Log performance statistics to CSV files for offline analysis"""
    
    def __init__(self):
        super().__init__('performance_logger')
        
        # Logging parameters
        self.declare_parameter('log_directory', '~/performance_logs')
        self.declare_parameter('log_prefix', 'rover_performance')
        
        log_dir = os.path.expanduser(self.get_parameter('log_directory').value)
        log_prefix = self.get_parameter('log_prefix').value
        
        # Create log directory if it doesn't exist
        os.makedirs(log_dir, exist_ok=True)
        
        # Create timestamped log files
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        self.latency_log_path = os.path.join(
            log_dir, 
            f'{log_prefix}_latency_{timestamp}.csv'
        )
        self.rate_log_path = os.path.join(
            log_dir, 
            f'{log_prefix}_rate_{timestamp}.csv'
        )
        
        # Initialize CSV files with headers
        self._init_latency_log()
        self._init_rate_log()
        
        # Subscribers for statistics
        self.vision_latency_sub = self.create_subscription(
            LatencyStats,
            '/monitoring/vision_latency',
            lambda msg: self._log_latency('vision', msg),
            10
        )
        
        self.motor_latency_sub = self.create_subscription(
            LatencyStats,
            '/monitoring/motor_latency',
            lambda msg: self._log_latency('motor', msg),
            10
        )
        
        self.total_latency_sub = self.create_subscription(
            LatencyStats,
            '/monitoring/total_latency',
            lambda msg: self._log_latency('total', msg),
            10
        )
        
        self.rate_stats_sub = self.create_subscription(
            RateStats,
            '/monitoring/rate_stats',
            self._log_rate_stats,
            10
        )
        
        self.get_logger().info(
            f'Performance Logger initialized\n'
            f'  Latency log: {self.latency_log_path}\n'
            f'  Rate log: {self.rate_log_path}'
        )
    
    def _init_latency_log(self):
        """Initialize latency CSV file with header"""
        with open(self.latency_log_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp',
                'measurement_type',
                'mean_latency_ms',
                'min_latency_ms',
                'max_latency_ms',
                'stddev_latency_ms',
                'sample_count',
                'miss_rate_percent',
                'deadline_ms'
            ])
    
    def _init_rate_log(self):
        """Initialize rate CSV file with header"""
        with open(self.rate_log_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp',
                'topic_name',
                'mean_rate_hz',
                'min_rate_hz',
                'max_rate_hz',
                'mean_jitter_ms',
                'max_jitter_ms',
                'message_count',
                'window_duration_s'
            ])
    
    def _log_latency(self, measurement_type, msg):
        """Append latency statistics to CSV"""
        timestamp = datetime.now().isoformat()
        
        with open(self.latency_log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp,
                measurement_type,
                f'{msg.mean_latency_ms:.2f}',
                f'{msg.min_latency_ms:.2f}',
                f'{msg.max_latency_ms:.2f}',
                f'{msg.stddev_latency_ms:.2f}',
                msg.sample_count,
                f'{msg.miss_rate_percent:.2f}',
                f'{msg.deadline_ms:.2f}'
            ])
        
        self.get_logger().debug(
            f'Logged {measurement_type} latency: {msg.mean_latency_ms:.1f}ms'
        )
    
    def _log_rate_stats(self, msg):
        """Append rate statistics to CSV"""
        timestamp = datetime.now().isoformat()
        
        with open(self.rate_log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp,
                msg.topic_name,
                f'{msg.mean_rate_hz:.2f}',
                f'{msg.min_rate_hz:.2f}',
                f'{msg.max_rate_hz:.2f}',
                f'{msg.mean_jitter_ms:.2f}',
                f'{msg.max_jitter_ms:.2f}',
                msg.message_count,
                f'{msg.window_duration_s:.2f}'
            ])
        
        self.get_logger().debug(
            f'Logged rate stats for {msg.topic_name}: {msg.mean_rate_hz:.1f} Hz'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PerformanceLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Performance logging stopped')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
