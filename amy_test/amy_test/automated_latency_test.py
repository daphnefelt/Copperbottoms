#!/usr/bin/env python3
"""
Automated Latency Test - Validates system latency against requirements
Runs a structured test sequence and generates pass/fail report
Useful for regression testing and automated validation

Part of the latency monitoring infrastructure for RTES rover application
All monitoring code is non-invasive and contained in amy_test package
"""

import rclpy
from rclpy.node import Node
from custom_messages.msg import LatencyStats, RateStats
import time
from datetime import datetime


class AutomatedLatencyTest(Node):
    """Automated test to validate latency requirements"""
    
    def __init__(self):
        super().__init__('automated_latency_test')
        
        # Test parameters
        self.declare_parameter('test_duration_s', 30.0)  # seconds to collect data
        self.declare_parameter('vision_deadline_ms', 50.0)
        self.declare_parameter('motor_deadline_ms', 20.0)
        self.declare_parameter('total_deadline_ms', 100.0)
        self.declare_parameter('max_miss_rate_percent', 5.0)
        
        self.test_duration = self.get_parameter('test_duration_s').value
        self.vision_deadline = self.get_parameter('vision_deadline_ms').value
        self.motor_deadline = self.get_parameter('motor_deadline_ms').value
        self.total_deadline = self.get_parameter('total_deadline_ms').value
        self.max_miss_rate = self.get_parameter('max_miss_rate_percent').value
        
        # Test results storage
        self.vision_results = []
        self.motor_results = []
        self.total_results = []
        self.rate_results = {}
        
        # Test state
        self.test_start_time = None
        self.test_complete = False
        
        # Subscribers for statistics
        self.vision_sub = self.create_subscription(
            LatencyStats,
            '/monitoring/vision_latency',
            lambda msg: self._collect_latency('vision', msg),
            10
        )
        
        self.motor_sub = self.create_subscription(
            LatencyStats,
            '/monitoring/motor_latency',
            lambda msg: self._collect_latency('motor', msg),
            10
        )
        
        self.total_sub = self.create_subscription(
            LatencyStats,
            '/monitoring/total_latency',
            lambda msg: self._collect_latency('total', msg),
            10
        )
        
        self.rate_sub = self.create_subscription(
            RateStats,
            '/monitoring/rate_stats',
            self._collect_rate_stats,
            10
        )
        
        # Timer to check test completion
        self.check_timer = self.create_timer(1.0, self._check_test_status)
        
        self.get_logger().info(
            f'\n'
            f'═══════════════════════════════════════════════════════════\n'
            f'  AUTOMATED LATENCY TEST\n'
            f'═══════════════════════════════════════════════════════════\n'
            f'  Test duration: {self.test_duration}s\n'
            f'  Vision deadline: {self.vision_deadline}ms\n'
            f'  Motor deadline: {self.motor_deadline}ms\n'
            f'  Total deadline: {self.total_deadline}ms\n'
            f'  Max miss rate: {self.max_miss_rate}%\n'
            f'═══════════════════════════════════════════════════════════\n'
            f'  Starting test...'
        )
        
        self.test_start_time = time.time()
    
    def _collect_latency(self, measurement_type, msg):
        """Collect latency statistics during test"""
        if self.test_complete:
            return
        
        if measurement_type == 'vision':
            self.vision_results.append(msg)
        elif measurement_type == 'motor':
            self.motor_results.append(msg)
        elif measurement_type == 'total':
            self.total_results.append(msg)
    
    def _collect_rate_stats(self, msg):
        """Collect rate statistics during test"""
        if self.test_complete:
            return
        
        if msg.topic_name not in self.rate_results:
            self.rate_results[msg.topic_name] = []
        self.rate_results[msg.topic_name].append(msg)
    
    def _check_test_status(self):
        """Check if test duration has elapsed and generate report"""
        if self.test_complete:
            return
        
        elapsed = time.time() - self.test_start_time
        
        if elapsed >= self.test_duration:
            self.test_complete = True
            self._generate_test_report()
            # Shutdown after report
            self.get_logger().info('Test complete - shutting down')
            rclpy.shutdown()
    
    def _evaluate_latency_requirement(self, results, deadline, label):
        """Evaluate latency results against requirement"""
        if not results:
            return {
                'label': label,
                'pass': False,
                'reason': 'No data collected',
                'mean': 0.0,
                'max': 0.0,
                'miss_rate': 0.0
            }
        
        # Calculate average statistics
        mean_latencies = [r.mean_latency_ms for r in results]
        max_latencies = [r.max_latency_ms for r in results]
        miss_rates = [r.miss_rate_percent for r in results]
        
        avg_mean = sum(mean_latencies) / len(mean_latencies)
        avg_max = sum(max_latencies) / len(max_latencies)
        avg_miss_rate = sum(miss_rates) / len(miss_rates)
        
        # Pass criteria
        mean_pass = avg_mean < deadline
        miss_rate_pass = avg_miss_rate < self.max_miss_rate
        
        passed = mean_pass and miss_rate_pass
        
        reason = []
        if not mean_pass:
            reason.append(f'mean latency {avg_mean:.1f}ms > {deadline}ms')
        if not miss_rate_pass:
            reason.append(f'miss rate {avg_miss_rate:.1f}% > {self.max_miss_rate}%')
        
        return {
            'label': label,
            'pass': passed,
            'reason': ', '.join(reason) if reason else 'OK',
            'mean': avg_mean,
            'max': avg_max,
            'miss_rate': avg_miss_rate,
            'deadline': deadline
        }
    
    def _generate_test_report(self):
        """Generate final test report with pass/fail results"""
        self.get_logger().info('Generating test report...')
        
        # Evaluate requirements
        vision_eval = self._evaluate_latency_requirement(
            self.vision_results, self.vision_deadline, 'Vision (camera → cmd_vel)'
        )
        
        motor_eval = self._evaluate_latency_requirement(
            self.motor_results, self.motor_deadline, 'Motor (cmd_vel → actuation)'
        )
        
        total_eval = self._evaluate_latency_requirement(
            self.total_results, self.total_deadline, 'Total (camera → actuation)'
        )
        
        # Overall test result
        all_tests_passed = (vision_eval['pass'] and 
                           motor_eval['pass'] and 
                           total_eval['pass'])
        
        # Print report
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        report = []
        report.append('\n' + '═' * 80)
        report.append('  AUTOMATED LATENCY TEST REPORT')
        report.append('═' * 80)
        report.append(f'  Test completed: {timestamp}')
        report.append(f'  Test duration: {self.test_duration}s')
        report.append('═' * 80)
        report.append('')
        report.append('LATENCY REQUIREMENTS:')
        report.append('─' * 80)
        
        for eval_result in [vision_eval, motor_eval, total_eval]:
            status = '✓ PASS' if eval_result['pass'] else '✗ FAIL'
            report.append(f"  {eval_result['label']:35s} {status}")
            report.append(f"    Mean: {eval_result['mean']:6.1f}ms (deadline: {eval_result['deadline']:.1f}ms)")
            report.append(f"    Max:  {eval_result['max']:6.1f}ms")
            report.append(f"    Miss: {eval_result['miss_rate']:6.1f}% (max: {self.max_miss_rate}%)")
            if not eval_result['pass']:
                report.append(f"    Reason: {eval_result['reason']}")
            report.append('')
        
        report.append('─' * 80)
        report.append('TOPIC RATES:')
        report.append('─' * 80)
        
        for topic_name, results in sorted(self.rate_results.items()):
            if results:
                avg_rate = sum(r.mean_rate_hz for r in results) / len(results)
                report.append(f"  {topic_name:35s} {avg_rate:6.1f} Hz")
        
        report.append('═' * 80)
        
        if all_tests_passed:
            report.append('  OVERALL RESULT: ✓ ALL TESTS PASSED')
        else:
            report.append('  OVERALL RESULT: ✗ SOME TESTS FAILED')
        
        report.append('═' * 80)
        
        # Print to log
        for line in report:
            self.get_logger().info(line)
        
        # Save report to file
        report_path = f'/tmp/latency_test_report_{datetime.now().strftime("%Y%m%d_%H%M%S")}.txt'
        with open(report_path, 'w') as f:
            f.write('\n'.join(report))
        
        self.get_logger().info(f'\nReport saved to: {report_path}')


def main(args=None):
    rclpy.init(args=args)
    node = AutomatedLatencyTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.test_complete:
            node.get_logger().warn('Test interrupted before completion')
        node.destroy_node()


if __name__ == '__main__':
    main()
