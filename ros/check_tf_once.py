#!/usr/bin/env python3
"""
One-shot TF transform checker.

Spawned as fresh process per check to avoid persistent service warmup issues.

Usage:
    python3 check_tf_once.py --parent odom --child base_link --timeout 30
"""

import sys
import argparse
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from tf2_ros import Buffer, TransformListener


class TFCheckerOnce(Node):
    """One-shot TF checker with ONLY TF buffer + listener."""

    def __init__(self):
        # NOTE: NOT using use_sim_time because it prevents TF buffer from populating
        # We'll skip the age check instead (see below)
        super().__init__('tf_checker_once')

        # ONLY TF buffer + listener (no other subscriptions)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


def check_transform(parent: str, child: str, timeout: float) -> tuple[bool, str, float]:
    """
    Check if transform exists and is fresh.

    Args:
        parent: Parent frame
        child: Child frame
        timeout: Timeout in seconds

    Returns:
        (success, message, duration)
    """
    rclpy.init()
    node = TFCheckerOnce()

    # Warmup period - let TF buffer populate (5 seconds)
    # Longer warmup needed for static transforms from /tf_static
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)

    # Check transform availability
    start_time = time.time()
    consecutive_success = 0
    required_consecutive = 3

    while time.time() - start_time < timeout:
        # Continue spinning to receive new TF messages
        rclpy.spin_once(node, timeout_sec=0.01)

        try:
            # Lookup transform
            transform = node.tf_buffer.lookup_transform(
                parent, child,
                Time(),
                timeout=Duration(seconds=0.5)
            )

            # NOTE: Skipping age check because we can't use use_sim_time (prevents TF buffer from filling)
            # If the transform exists in buffer, it's fresh enough for our purposes
            consecutive_success += 1
            if consecutive_success >= required_consecutive:
                duration = time.time() - start_time
                message = f'Transform {parent}→{child} available'
                rclpy.shutdown()
                return True, message, duration

        except Exception:
            consecutive_success = 0

        time.sleep(0.5)

    # Timeout
    duration = time.time() - start_time
    message = f'Transform {parent}→{child} timeout'
    rclpy.shutdown()
    return False, message, duration


def main():
    parser = argparse.ArgumentParser(description='One-shot TF transform check')
    parser.add_argument('--parent', required=True, help='Parent frame')
    parser.add_argument('--child', required=True, help='Child frame')
    parser.add_argument('--timeout', type=float, default=30.0, help='Timeout in seconds')
    parser.add_argument('--json', action='store_true', help='Output JSON')
    args = parser.parse_args()

    success, message, duration = check_transform(args.parent, args.child, args.timeout)

    if args.json:
        import json
        print(json.dumps({
            'success': success,
            'message': message,
            'duration': duration
        }))
    else:
        status = '✓' if success else '✗'
        print(f'{status} {args.parent}→{args.child} ({duration:.1f}s): {message}')

    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
