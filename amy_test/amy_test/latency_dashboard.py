#!/usr/bin/env python3
"""
Latency Dashboard - Real-time console dashboard for monitoring performance
Displays live latency and rate statistics in a formatted console view
Useful for real-time debugging and tuning during development

Part of the latency monitoring infrastructure for RTES rover application
All monitoring code is non-invasive and contained in amy_test package
"""

import rclpy
from rclpy.node import Node
from custom_messages.msg import LatencyStats, RateStats
import os
from datetime import datetime


class LatencyDashboard(Node):
    """Display real-time latency and rate statistics in console dashboard"""
    
    def __init__(self):
        super().__init__('latency_dashboard')
        
        # Storage for latest statistics
        self.latency_stats = {}  # measurement_type -> LatencyStats
        self.rate_stats = {}  # topic_name -> RateStats
        
        # Subscribers for all statistics
        self.vision_latency_sub = self.create_subscription(
            LatencyStats,
            '/monitoring/vision_latency',
            lambda msg: self._update_latency('vision', msg),
            10
        )
        
        self.motor_latency_sub = self.create_subscription(
            LatencyStats,
            '/monitoring/motor_latency',
            lambda msg: self._update_latency('motor', msg),
            10
        )
        
        self.total_latency_sub = self.create_subscription(
            LatencyStats,
            '/monitoring/total_latency',
            lambda msg: self._update_latency('total', msg),
            10
        )
        
        self.rate_stats_sub = self.create_subscription(
            RateStats,
            '/monitoring/rate_stats',
            self._update_rate_stats,
            10
        )
        
        # Timer to refresh dashboard display
        self.refresh_timer = self.create_timer(
            2.0,  # Refresh every 2 seconds
            self._refresh_dashboard
        )
        
        self.get_logger().info('Latency Dashboard initialized - displaying real-time stats')
    
    def _update_latency(self, measurement_type, msg):
        """Update stored latency statistics"""
        self.latency_stats[measurement_type] = msg
    
    def _update_rate_stats(self, msg):
        """Update stored rate statistics"""
        self.rate_stats[msg.topic_name] = msg
    
    def _format_latency_row(self, label, stats, width=60):
        """Format a single latency statistics row"""
        if stats is None:
            return f"  {label:20s} : No data"
        
        # Status indicator
        if stats.mean_latency_ms < stats.deadline_ms:
            status = '✓'
        elif stats.mean_latency_ms < stats.deadline_ms * 1.2:
            status = '⚠'
        else:
            status = '✗'
        
        return (
            f"  {label:20s} : {stats.mean_latency_ms:6.1f}ms "
            f"± {stats.stddev_latency_ms:5.1f}ms "
            f"[{stats.min_latency_ms:5.1f} - {stats.max_latency_ms:5.1f}] "
            f"miss={stats.miss_rate_percent:5.1f}% {status}"
        )
    
    def _format_rate_row(self, topic, stats):
        """Format a single rate statistics row"""
        if stats is None:
            return f"  {topic:20s} : No data"
        
        return (
            f"  {topic:25s} : {stats.mean_rate_hz:6.1f} Hz "
            f"(jitter: {stats.mean_jitter_ms:5.1f}ms, n={stats.message_count})"
        )
    
    def _refresh_dashboard(self):
        """Clear screen and redraw dashboard with latest statistics"""
        # Clear screen (ANSI escape code)
        os.system('clear' if os.name != 'nt' else 'cls')
        
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        # Build dashboard display
        dashboard = []
        dashboard.append('╔' + '═' * 78 + '╗')
        dashboard.append('║' + ' ' * 20 + 'ROVER LATENCY MONITORING DASHBOARD' + ' ' * 24 + '║')
        dashboard.append('║' + f' {timestamp:^76s} ' + '║')
        dashboard.append('╠' + '═' * 78 + '╣')
        
        # Latency Statistics Section
        dashboard.append('║ LATENCY STATISTICS' + ' ' * 59 + '║')
        dashboard.append('║' + '─' * 78 + '║')
        
        vision_stats = self.latency_stats.get('vision')
        motor_stats = self.latency_stats.get('motor')
        total_stats = self.latency_stats.get('total')
        
        dashboard.append('║' + self._format_latency_row('Vision (cam→cmd_vel)', vision_stats).ljust(78) + '║')
        dashboard.append('║' + self._format_latency_row('Motor (cmd_vel→act)', motor_stats).ljust(78) + '║')
        dashboard.append('║' + '─' * 78 + '║')
        dashboard.append('║' + self._format_latency_row('TOTAL (cam→act)', total_stats).ljust(78) + '║')
        
        # Total latency status
        if total_stats:
            if total_stats.mean_latency_ms < 70:
                status_msg = '✓ EXCELLENT - Well within deadline'
            elif total_stats.mean_latency_ms < 100:
                status_msg = '✓ GOOD - Meeting deadline'
            elif total_stats.mean_latency_ms < 120:
                status_msg = '⚠ MARGINAL - Near deadline limit'
            else:
                status_msg = '✗ BAD - Exceeding deadline!'
            
            dashboard.append('║  Status: ' + status_msg.ljust(68) + '║')
        
        dashboard.append('╠' + '═' * 78 + '╣')
        
        # Topic Rate Statistics Section
        dashboard.append('║ TOPIC RATES' + ' ' * 66 + '║')
        dashboard.append('║' + '─' * 78 + '║')
        
        # Sort topics for consistent display
        sorted_topics = sorted(self.rate_stats.keys())
        for topic_name in sorted_topics:
            stats = self.rate_stats[topic_name]
            dashboard.append('║' + self._format_rate_row(topic_name, stats).ljust(78) + '║')
        
        if not self.rate_stats:
            dashboard.append('║  No rate data available' + ' ' * 54 + '║')
        
        dashboard.append('╚' + '═' * 78 + '╝')
        
        # Print dashboard
        print('\n'.join(dashboard))
        print('\nPress Ctrl+C to exit')


def main(args=None):
    rclpy.init(args=args)
    node = LatencyDashboard()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nDashboard stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
