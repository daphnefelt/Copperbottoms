#!/usr/bin/env python3
"""
Motor Latency Monitor - Measures cmd_vel to motor actuation latency
Detects IMU acceleration changes following cmd_vel commands to estimate motor response time
Expected latency: 10-15ms (MAVLink transmission + ArduPilot processing + servo response)

Part of the latency monitoring infrastructure for RTES rover application
All monitoring code is non-invasive and contained in amy_test package
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from custom_messages.msg import LatencyStats
from collections import deque
import time
import statistics
import math


class MotorLatencyMonitor(Node):
    """Monitor latency from cmd_vel to observable motor actuation via IMU"""
    
    def __init__(self):
        super().__init__('motor_latency_monitor')
        
        # Measurement parameters
        self.declare_parameter('measurement_window', 10.0)  # seconds
        self.declare_parameter('accel_threshold', 0.1)  # m/s² threshold for detection
        self.declare_parameter('response_window_ms', 200.0)  # max response time to check
        self.declare_parameter('deadline_ms', 20.0)  # motor actuation deadline
        
        self.measurement_window = self.get_parameter('measurement_window').value
        self.accel_threshold = self.get_parameter('accel_threshold').value
        self.response_window = self.get_parameter('response_window_ms').value / 1000.0
        self.deadline_ms = self.get_parameter('deadline_ms').value
        
        # Storage for measurements
        self.latencies = deque(maxlen=1000)
        self.recent_cmd_vels = deque(maxlen=100)  # (timestamp, linear_x, angular_z)
        self.recent_accels = deque(maxlen=500)  # (timestamp, accel_magnitude)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Vector3,
            '/imu/accel',
            self._imu_callback,
            10
        )
        
        # Publisher for latency statistics
        self.stats_pub = self.create_publisher(
            LatencyStats,
            '/monitoring/motor_latency',
            10
        )
        
        # Timer to compute and publish statistics
        self.stats_timer = self.create_timer(
            self.measurement_window,
            self._publish_statistics
        )
        
        # Baseline acceleration (updated with moving average)
        self.baseline_accel = 0.0
        self.baseline_alpha = 0.95  # Low-pass filter coefficient
        
        self.get_logger().info(
            f'Motor Latency Monitor initialized '
            f'(window={self.measurement_window}s, '
            f'accel_threshold={self.accel_threshold} m/s², '
            f'deadline={self.deadline_ms}ms)'
        )
    
    def _cmd_vel_callback(self, msg):
        """Record cmd_vel commands with timestamps"""
        timestamp = time.time()
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Only record significant velocity changes
        if abs(linear_x) > 0.01 or abs(angular_z) > 0.01:
            self.recent_cmd_vels.append((timestamp, linear_x, angular_z))
    
    def _imu_callback(self, msg):
        """
        Record IMU acceleration and correlate with recent cmd_vel commands
        Look for acceleration changes following velocity commands
        """
        timestamp = time.time()
        
        # Calculate acceleration magnitude
        accel_mag = math.sqrt(msg.x**2 + msg.y**2 + msg.z**2)
        
        # Update baseline acceleration (low-pass filter)
        self.baseline_accel = (self.baseline_alpha * self.baseline_accel + 
                               (1 - self.baseline_alpha) * accel_mag)
        
        # Detect significant acceleration change from baseline
        accel_change = abs(accel_mag - self.baseline_accel)
        
        self.recent_accels.append((timestamp, accel_mag))
        
        # Look for recent cmd_vel that might have caused this acceleration
        if accel_change > self.accel_threshold:
            self._correlate_with_cmd_vel(timestamp, accel_change)
    
    def _correlate_with_cmd_vel(self, accel_timestamp, accel_change):
        """
        Find the most recent cmd_vel within response window
        Calculate latency from cmd_vel to acceleration detection
        """
        # Look backward in time for cmd_vel within response window
        matching_cmd_vels = [
            (ts, lin_x, ang_z) 
            for ts, lin_x, ang_z in self.recent_cmd_vels
            if 0 < (accel_timestamp - ts) < self.response_window
        ]
        
        if not matching_cmd_vels:
            return
        
        # Use the most recent cmd_vel (closest in time)
        cmd_vel_time, linear_x, angular_z = max(matching_cmd_vels, key=lambda x: x[0])
        
        # Calculate latency in milliseconds
        latency_ms = (accel_timestamp - cmd_vel_time) * 1000.0
        
        # Sanity check - reasonable latency range
        if 0 < latency_ms < self.response_window * 1000:
            self.latencies.append(latency_ms)
            
            self.get_logger().debug(
                f'Motor response detected: {latency_ms:.1f}ms '
                f'(cmd_vel: x={linear_x:.2f}, z={angular_z:.2f}, '
                f'accel_change={accel_change:.2f} m/s²)'
            )
    
    def _publish_statistics(self):
        """Compute and publish motor latency statistics"""
        if len(self.latencies) < 2:
            self.get_logger().warn(
                'Insufficient motor latency samples - '
                'try driving rover with varying velocity commands'
            )
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
        msg.measurement_type = 'motor'
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
            f'Motor latency: {mean_latency:.1f}ms ± {stddev_latency:.1f}ms '
            f'(min={min_latency:.1f}, max={max_latency:.1f}, '
            f'miss={miss_rate:.1f}%, n={len(latencies_list)}) {status}'
        )
        
        # Clear measurements for next window
        self.latencies.clear()


def main(args=None):
    rclpy.init(args=args)
    node = MotorLatencyMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
