#!/usr/bin/env python3
"""
Reset Gates Node - Container-side readiness gate checker.

This ROS2 node provides gate check methods for verifying system readiness
during reset sequences. Uses rclpy subscriptions (NO CLI tools) for reliability.

Architecture:
- Single node instance checks multiple gates sequentially
- Persistent TF buffer and subscriptions across gates in same invocation
- Returns consolidated JSON result for all gate checks
- All subscriptions created in __init__() per project rules

Usage:
    python3 reset_gates.py --gates clock_monotonic tf_static:base_link --timeout 30 --json
"""

import sys
import time
import json
import argparse
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.duration import Duration

from builtin_interfaces.msg import Time as TimeMsg
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from controller_manager_msgs.msg import ControllerState
from nav2_msgs.action import NavigateToPose


class ResetGatesNode(Node):
    """
    ROS2 node for checking multiple readiness gates in sequence.

    Created once per invocation, checks multiple gates, then exits.
    All subscriptions created in __init__() and persist for entire session.
    """

    def __init__(self):
        super().__init__('reset_gates')

        # === Subscriptions (created once, reused across all gate checks) ===

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self._clock_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_callback, 10)

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_callback, 10)

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 10)

        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_states_callback, 10)

        self.controller_state_sub = self.create_subscription(
            ControllerState,
            '/tricycle_steering_controller/controller_state',
            self._controller_state_callback, 10)

        # Subscribe to /tf_static for static frame checking
        self.tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self._tf_static_callback, 10)

        # === TF Buffer (created once, reused) ===
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # === Action Clients (created once) ===
        self.nav_action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # === Message storage (last received) ===
        self.last_clock: Optional[TimeMsg] = None
        self.last_clock_time: Optional[float] = None
        self.clock_samples: List[Tuple[float, float]] = []  # (sim_time, wall_time)

        self.last_odom: Optional[Odometry] = None
        self.last_odom_time: Optional[float] = None

        self.last_amcl: Optional[PoseWithCovarianceStamped] = None
        self.last_amcl_time: Optional[float] = None

        self.last_map: Optional[OccupancyGrid] = None
        self.last_map_time: Optional[float] = None

        self.last_joint_states: Optional[JointState] = None
        self.last_joint_states_time: Optional[float] = None

        self.last_controller_state: Optional[ControllerState] = None
        self.last_controller_state_time: Optional[float] = None

        # Track static frames from /tf_static topic
        self.static_frames: set = set()

        self.get_logger().info('ResetGatesNode initialized')

    # === Callbacks (store messages with timestamps) ===

    def _clock_callback(self, msg: Clock):
        """Store clock messages for monotonic checking."""
        now = time.time()
        self.last_clock = msg.clock
        self.last_clock_time = now

        sim_time = msg.clock.sec + msg.clock.nanosec / 1e9
        self.clock_samples.append((sim_time, now))

        # Keep last 10 samples
        if len(self.clock_samples) > 10:
            self.clock_samples.pop(0)

    def _odom_callback(self, msg: Odometry):
        """Store odometry messages."""
        self.last_odom = msg
        self.last_odom_time = time.time()

    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        """Store AMCL pose messages."""
        self.last_amcl = msg
        self.last_amcl_time = time.time()

    def _map_callback(self, msg: OccupancyGrid):
        """Store map messages."""
        self.last_map = msg
        self.last_map_time = time.time()

    def _joint_states_callback(self, msg: JointState):
        """Store joint state messages."""
        self.last_joint_states = msg
        self.last_joint_states_time = time.time()

    def _controller_state_callback(self, msg: ControllerState):
        """Store controller state messages."""
        self.last_controller_state = msg
        self.last_controller_state_time = time.time()

    def _tf_static_callback(self, msg: TFMessage):
        """Track static frames from /tf_static topic."""
        for transform in msg.transforms:
            self.static_frames.add(transform.child_frame_id)
            self.static_frames.add(transform.header.frame_id)

    # === Gate Check Methods ===

    def check_clock_monotonic(self, timeout: float = 30.0) -> bool:
        """
        Wait for /clock topic to publish monotonically increasing time.

        Verifies:
        - /clock topic is active
        - Time is increasing (not stuck or going backward)
        - At least 3 consecutive samples with t[i+1] > t[i]

        Args:
            timeout: Maximum wait time in seconds

        Returns:
            True if clock is monotonic within timeout
        """
        self.get_logger().info(f'Checking clock_monotonic (timeout={timeout}s)...')
        start_time = time.time()

        while time.time() - start_time < timeout:
            # Spin to process callbacks
            rclpy.spin_once(self, timeout_sec=0.5)

            # Need at least 3 samples
            if len(self.clock_samples) >= 3:
                # Check last 3 samples are monotonic
                recent = self.clock_samples[-3:]
                is_monotonic = all(
                    recent[i+1][0] > recent[i][0]
                    for i in range(len(recent) - 1)
                )

                if is_monotonic:
                    self.get_logger().info('✓ clock_monotonic')
                    return True

            time.sleep(0.5)

        self.get_logger().error(f'✗ clock_monotonic timeout ({timeout}s)')
        return False

    def check_tf_static(self, *expected_frames, timeout: float = 30.0) -> bool:
        """
        Wait for /tf_static to contain expected static frames.

        Subscribes to /tf_static topic and checks for specific frame IDs.

        Args:
            expected_frames: Frame IDs to check (e.g., 'base_link', 'base_footprint')
            timeout: Maximum wait time

        Returns:
            True if all expected frames present in /tf_static within timeout
        """
        self.get_logger().info(
            f'Checking tf_static for frames: {expected_frames} (timeout={timeout}s)...')

        # Default expected frames if none specified
        if not expected_frames:
            expected_frames = ('base_link', 'base_footprint')

        start_time = time.time()

        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)

            # Check if all expected frames are in static_frames set
            missing = [f for f in expected_frames if f not in self.static_frames]

            if not missing:
                self.get_logger().info(f'✓ tf_static (frames: {expected_frames})')
                return True

            time.sleep(0.5)

        # Log missing frames on timeout
        missing = [f for f in expected_frames if f not in self.static_frames]
        self.get_logger().error(
            f'✗ tf_static timeout ({timeout}s) - missing frames: {missing}')
        return False

    def check_tf_transform(self, parent: str, child: str, timeout: float = 30.0) -> bool:
        """
        Wait for TF transform from parent to child to be available and fresh.

        Verifies:
        - Transform exists in TF tree
        - Transform age < 1.0 second (actively published)
        - 3 consecutive successful lookups (stability)

        Args:
            parent: Parent frame (e.g., 'odom', 'map')
            child: Child frame (e.g., 'base_link', 'odom')
            timeout: Maximum wait time

        Returns:
            True if transform available and stable within timeout
        """
        self.get_logger().info(
            f'Checking tf_transform {parent}→{child} (timeout={timeout}s)...')
        start_time = time.time()
        consecutive_success = 0

        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

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
                    if consecutive_success >= 3:
                        self.get_logger().info(f'✓ tf_transform {parent}→{child}')
                        return True
                else:
                    consecutive_success = 0
                    self.get_logger().debug(
                        f'Transform {parent}→{child} too old: {age_sec:.2f}s')

            except Exception as e:
                consecutive_success = 0
                self.get_logger().warn(
                    f'Transform {parent}→{child} lookup failed: {e}')

            time.sleep(0.5)

        self.get_logger().error(
            f'✗ tf_transform {parent}→{child} timeout ({timeout}s)')
        return False

    def check_topic_active(self, topic_name: str, max_age: float = 2.0,
                          timeout: float = 30.0) -> bool:
        """
        Wait for topic to be actively publishing (recent message received).

        Verifies:
        - Topic has been received at least once
        - Last message age < max_age seconds

        Args:
            topic_name: Topic to check (e.g., '/odometry/filtered')
            max_age: Maximum message age in seconds
            timeout: Maximum wait time

        Returns:
            True if topic is active within timeout
        """
        self.get_logger().info(
            f'Checking topic_active {topic_name} (timeout={timeout}s)...')
        start_time = time.time()

        # Map topic names to stored messages
        topic_map = {
            '/clock': (lambda: self.last_clock, lambda: self.last_clock_time),
            '/odometry/filtered': (lambda: self.last_odom, lambda: self.last_odom_time),
            '/amcl_pose': (lambda: self.last_amcl, lambda: self.last_amcl_time),
            '/map': (lambda: self.last_map, lambda: self.last_map_time),
            '/joint_states': (lambda: self.last_joint_states,
                             lambda: self.last_joint_states_time),
        }

        if topic_name not in topic_map:
            self.get_logger().error(f'Unknown topic: {topic_name}')
            return False

        msg_getter, time_getter = topic_map[topic_name]

        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)

            msg = msg_getter()
            msg_time = time_getter()

            if msg is not None and msg_time is not None:
                age = time.time() - msg_time
                if age < max_age:
                    self.get_logger().info(f'✓ topic_active {topic_name}')
                    return True

            time.sleep(0.5)

        self.get_logger().error(f'✗ topic_active {topic_name} timeout ({timeout}s)')
        return False

    def check_controller_active(self, controller_name: str = 'tricycle_steering_controller',
                               timeout: float = 30.0) -> bool:
        """
        Wait for controller to be in 'active' state.

        Uses proper message field access (not string search).

        Args:
            controller_name: Controller name (default: tricycle_steering_controller)
            timeout: Maximum wait time

        Returns:
            True if controller state is 'active' within timeout
        """
        self.get_logger().info(
            f'Checking controller_active {controller_name} (timeout={timeout}s)...')
        start_time = time.time()

        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)

            if self.last_controller_state is not None:
                # Access .state field directly
                # ControllerState.state is a string: 'active', 'inactive', 'unconfigured'
                if hasattr(self.last_controller_state, 'state'):
                    if self.last_controller_state.state == 'active':
                        self.get_logger().info(f'✓ controller_active {controller_name}')
                        return True
                else:
                    # Fallback for unknown message format
                    self.get_logger().warning(
                        f'ControllerState message has no .state field, using string search')
                    state_str = str(self.last_controller_state)
                    if 'active' in state_str.lower():
                        self.get_logger().info(f'✓ controller_active {controller_name}')
                        return True

            time.sleep(0.5)

        self.get_logger().error(
            f'✗ controller_active {controller_name} timeout ({timeout}s)')
        return False

    def check_amcl_publishing(self, timeout: float = 60.0) -> bool:
        """
        Wait for AMCL to start publishing poses.

        AMCL has set_initial_pose: true in config, will auto-initialize.
        This gate just verifies /amcl_pose topic is publishing.

        Args:
            timeout: Maximum wait time

        Returns:
            True if AMCL publishes /amcl_pose within timeout
        """
        self.get_logger().info(
            f'Checking amcl_publishing (timeout={timeout}s)...')
        self.get_logger().info(
            'AMCL has set_initial_pose:true, will auto-initialize from config')

        start_time = time.time()

        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)

            if self.last_amcl is not None and self.last_amcl_time is not None:
                age = time.time() - self.last_amcl_time
                if age < 2.0:  # Recent message
                    self.get_logger().info('✓ amcl_publishing')
                    return True

            time.sleep(0.5)

        self.get_logger().error(f'✗ amcl_publishing timeout ({timeout}s)')
        return False

    def check_amcl_converged(self, covariance_threshold: float = 0.5,
                            timeout: float = 60.0) -> bool:
        """
        Wait for AMCL pose to converge (low covariance).

        Verifies:
        - /amcl_pose actively publishing
        - Covariance trace < threshold
        - 3 consecutive successful checks

        Args:
            covariance_threshold: Max trace of position covariance matrix
            timeout: Maximum wait time

        Returns:
            True if AMCL converged within timeout
        """
        self.get_logger().info(
            f'Checking amcl_converged: expecting covariance trace < {covariance_threshold}, '
            f'3 consecutive checks, timeout={timeout}s')
        self.get_logger().info(
            f'  Requirement: /amcl_pose publishing with position uncertainty < {covariance_threshold}')
        self.get_logger().info(
            f'  Note: AMCL requires robot movement to converge particle filter')

        start_time = time.time()
        consecutive_success = 0
        last_logged_trace = None
        log_interval = 5.0  # Log status every 5 seconds
        last_log_time = start_time

        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            elapsed = time.time() - start_time

            if self.last_amcl is not None and self.last_amcl_time is not None:
                age = time.time() - self.last_amcl_time
                if age < 2.0:
                    # Check covariance trace (xx + yy)
                    cov = self.last_amcl.pose.covariance
                    trace = cov[0] + cov[7]  # cov[0,0] + cov[1,1]

                    # Log status every 5 seconds
                    if time.time() - last_log_time >= log_interval:
                        self.get_logger().info(
                            f'  [{elapsed:.1f}s] AMCL covariance trace: {trace:.3f} '
                            f'(threshold: {covariance_threshold}, consecutive: {consecutive_success}/3)')
                        last_log_time = time.time()

                    if trace < covariance_threshold:
                        consecutive_success += 1
                        if consecutive_success >= 3:
                            self.get_logger().info(
                                f'✓ amcl_converged (trace={trace:.3f} < {covariance_threshold})')
                            return True
                    else:
                        consecutive_success = 0
                        last_logged_trace = trace
                else:
                    # /amcl_pose is stale
                    if time.time() - last_log_time >= log_interval:
                        self.get_logger().info(
                            f'  [{elapsed:.1f}s] /amcl_pose is stale (age={age:.1f}s > 2.0s)')
                        last_log_time = time.time()
            else:
                # No /amcl_pose received yet
                if time.time() - last_log_time >= log_interval:
                    self.get_logger().info(
                        f'  [{elapsed:.1f}s] Waiting for /amcl_pose messages...')
                    last_log_time = time.time()

            time.sleep(1.0)

        # Timeout - report final state
        if self.last_amcl is not None and self.last_amcl_time is not None:
            age = time.time() - self.last_amcl_time
            cov = self.last_amcl.pose.covariance
            trace = cov[0] + cov[7]
            self.get_logger().error(
                f'✗ amcl_converged TIMEOUT ({timeout}s): '
                f'Final covariance trace={trace:.3f} (required: <{covariance_threshold}), '
                f'consecutive={consecutive_success}/3, '
                f'pose_age={age:.1f}s')
            self.get_logger().error(
                f'  FAILURE REASON: Covariance too high - AMCL needs robot movement to converge')
        elif self.last_amcl is not None:
            self.get_logger().error(
                f'✗ amcl_converged TIMEOUT ({timeout}s): '
                f'FAILURE REASON: /amcl_pose messages stopped (last seen >2s ago)')
        else:
            self.get_logger().error(
                f'✗ amcl_converged TIMEOUT ({timeout}s): '
                f'FAILURE REASON: No /amcl_pose messages received')
        return False

    def check_nav2_action_server(self, timeout: float = 120.0) -> bool:
        """
        Wait for /navigate_to_pose action server to be ready.

        Uses ActionClient.wait_for_server() to check availability.
        This verifies lifecycle manager has activated bt_navigator.

        Args:
            timeout: Maximum wait time (120s for lifecycle activation)

        Returns:
            True if action server ready within timeout
        """
        self.get_logger().info(
            f'Checking nav2_action_server (timeout={timeout}s)...')
        self.get_logger().info(
            'Waiting for lifecycle manager to activate bt_navigator...')

        # Spin while waiting for server
        server_ready = self.nav_action_client.wait_for_server(timeout_sec=timeout)

        if server_ready:
            self.get_logger().info('✓ nav2_action_server')
            return True
        else:
            self.get_logger().error(f'✗ nav2_action_server timeout ({timeout}s)')
            return False


def main():
    """
    Command-line interface for checking multiple gates in sequence.

    Checks multiple gates in single invocation, returns consolidated result.

    Usage:
        python reset_gates.py \
          --gates clock_monotonic tf_static:base_link tf_transform:odom:base_link \
          --timeout 30 \
          --json

    Exit codes:
        0 = all gates passed
        1 = one or more gates failed
    """
    parser = argparse.ArgumentParser(description='ROS2 Reset Gates Checker')
    parser.add_argument('--gates', nargs='+', required=True,
                       help='Gate checks to perform (gate or gate:arg1:arg2)')
    parser.add_argument('--timeout', type=float, default=30.0,
                       help='Timeout per gate in seconds')
    parser.add_argument('--json', action='store_true',
                       help='Output JSON result')

    args = parser.parse_args()

    rclpy.init()
    node = ResetGatesNode()

    # Warmup period to let all subscriptions discover publishers
    # TF listener + all other subscriptions need DDS discovery time
    # With many subscriptions, DDS discovery can take 10+ seconds
    node.get_logger().info('Warming up subscriptions (8s)...')
    for _ in range(80):  # 8 second warmup (80 x 0.1s) - OPT-1 (5s failed, trying 8s)
        rclpy.spin_once(node, timeout_sec=0.1)

    # Map gate names to methods
    gate_methods = {
        'clock_monotonic': node.check_clock_monotonic,
        'tf_static': node.check_tf_static,
        'tf_transform': node.check_tf_transform,
        'topic_active': node.check_topic_active,
        'controller_active': node.check_controller_active,
        'amcl_publishing': node.check_amcl_publishing,
        'amcl_converged': node.check_amcl_converged,
        'nav2_action_server': node.check_nav2_action_server,
    }

    # Check all gates in sequence
    results = []
    all_success = True
    start_time = time.time()

    for gate_spec in args.gates:
        # Parse gate specification: "gate_name" or "gate_name:arg1:arg2"
        parts = gate_spec.split(':')
        gate_name = parts[0]
        gate_args = parts[1:] if len(parts) > 1 else []

        if gate_name not in gate_methods:
            print(f"Unknown gate: {gate_name}", file=sys.stderr)
            results.append({
                'name': gate_name,
                'args': gate_args,
                'success': False,
                'error': 'unknown_gate'
            })
            all_success = False
            continue

        # Call gate check method
        method = gate_methods[gate_name]
        gate_start = time.time()

        try:
            success = method(*gate_args, timeout=args.timeout)
            duration = time.time() - gate_start

            results.append({
                'name': gate_name,
                'args': gate_args,
                'success': success,
                'duration': duration
            })

            if not success:
                all_success = False

        except Exception as e:
            duration = time.time() - gate_start
            results.append({
                'name': gate_name,
                'args': gate_args,
                'success': False,
                'duration': duration,
                'error': str(e)
            })
            all_success = False

    total_duration = time.time() - start_time

    # Output result
    if args.json:
        result = {
            'success': all_success,
            'gates': results,
            'total_duration': total_duration
        }
        print(json.dumps(result))
    else:
        # Human-readable output
        for r in results:
            status = '✓' if r['success'] else '✗'
            gate_desc = f"{r['name']}:{':'.join(r['args'])}" if r['args'] else r['name']
            print(f"{status} {gate_desc} ({r.get('duration', 0):.1f}s)")
        print(f"\nTotal: {total_duration:.1f}s, Success: {all_success}")

    rclpy.shutdown()
    return 0 if all_success else 1


if __name__ == '__main__':
    sys.exit(main())
