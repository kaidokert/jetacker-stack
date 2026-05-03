#!/usr/bin/env python3
"""
TF Gate Checker Node - Persistent service for TF transform checks.

CRITICAL: This node ONLY has TF buffer + listener subscriptions.
No other subscriptions allowed to prevent executor contention.

Architecture:
- Runs continuously as a persistent service
- TF buffer stays warm (no startup penalty)
- Isolated from other subscriptions (prevents TF tree fragmentation)
- Exposes /check_tf service for transform checks

Usage:
    ros2 service call /check_tf reset_interfaces/srv/CheckTF \
      "{parent_frame: 'odom', child_frame: 'base_link', timeout: 30.0}"
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from tf2_ros import Buffer, TransformListener
import time

# Import custom service (will be available after overlay build)
from reset_interfaces.srv import CheckTF


class TFGateCheckerNode(Node):
    """
    Persistent TF gate checker service.

    CRITICAL: ONLY TF buffer + listener subscriptions.
    No clock, odom, amcl, map, joint_states, controller_state, or action clients.
    """

    def __init__(self):
        super().__init__('tf_gate_checker')

        # === TF Buffer + Listener (ONLY SUBSCRIPTIONS) ===
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # === Service ===
        self.check_tf_service = self.create_service(
            CheckTF, '/check_tf', self.check_tf_callback)

        self.get_logger().info('TF Gate Checker Node initialized')
        self.get_logger().info('Service: /check_tf (parent_frame, child_frame, timeout)')

        # Warmup period - let TF buffer populate before accepting calls
        self.get_logger().info('Warming up TF buffer (10s)...')
        self._warmup_tf_buffer()
        self.get_logger().info('TF buffer ready')

    def _warmup_tf_buffer(self):
        """
        Warmup TF buffer after node initialization.

        Spins for 10 seconds to let TF listener receive initial transforms.
        This prevents first service call from failing due to empty buffer.
        """
        warmup_start = time.time()
        warmup_duration = 10.0

        sample_count = 0
        while time.time() - warmup_start < warmup_duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            sample_count += 1
            time.sleep(0.1)

        self.get_logger().info(f'Warmup complete: {sample_count} spin cycles')

    def check_tf_callback(self, request, response):
        """
        Service callback for TF transform checks.

        Checks if transform from parent_frame to child_frame exists and is fresh.

        Args:
            request: CheckTF request with parent_frame, child_frame, timeout
            response: CheckTF response with success, message, duration

        Returns:
            response: Populated response object
        """
        parent = request.parent_frame
        child = request.child_frame
        timeout = request.timeout

        self.get_logger().info(
            f'Checking TF transform {parent}→{child} (timeout={timeout:.1f}s)')

        start_time = time.time()
        consecutive_success = 0
        required_consecutive = 3  # Require 3 consecutive successes for stability

        # Check transform availability and freshness
        deadline = start_time + timeout

        while time.time() < deadline:
            try:
                # Lookup transform (latest available)
                transform = self.tf_buffer.lookup_transform(
                    parent, child,
                    Time(),  # Latest available
                    timeout=Duration(seconds=0.5)
                )

                # Check transform freshness (age < 1 second)
                now = self.get_clock().now()
                age = now - Time.from_msg(transform.header.stamp)
                age_sec = age.nanoseconds / 1e9

                if age_sec < 1.0:
                    consecutive_success += 1
                    if consecutive_success >= required_consecutive:
                        duration = time.time() - start_time
                        response.success = True
                        response.message = f'Transform {parent}→{child} available and fresh'
                        response.duration = duration
                        self.get_logger().info(
                            f'✓ TF transform {parent}→{child} ({duration:.1f}s)')
                        return response
                else:
                    consecutive_success = 0
                    self.get_logger().debug(
                        f'Transform {parent}→{child} too old: {age_sec:.2f}s')

            except Exception as e:
                consecutive_success = 0
                self.get_logger().warn(
                    f'Transform {parent}→{child} lookup failed: {e}')

            # Brief sleep before retry
            time.sleep(0.5)

        # Timeout - transform not available
        duration = time.time() - start_time
        response.success = False
        response.message = f'Transform {parent}→{child} timeout ({timeout:.1f}s)'
        response.duration = duration
        self.get_logger().error(f'✗ TF transform {parent}→{child} timeout')
        return response


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = TFGateCheckerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
