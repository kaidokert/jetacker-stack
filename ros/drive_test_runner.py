#!/usr/bin/env python3
"""
Drive Test Runner - ROS2-native test harness

Runs INSIDE the container to avoid fragile docker compose exec patterns.
Host just needs one command: docker compose exec test-drive python3 /workspace/ros/drive_test_runner.py

This node:
- Subscribes to /test_drive/* topics
- Calls /stackReset and /driveDistance services
- Waits for test completion using ROS2 spin
- Returns exit code 0 on success, 1 on failure
"""

import sys
import argparse
import json
import subprocess
import time
import logging
import os
from pathlib import Path
from datetime import datetime
from logging.handlers import RotatingFileHandler
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration
from std_srvs.srv import Trigger
from scenario_execution_interfaces.srv import ExecuteScenario
from std_msgs.msg import Bool, String
from tf2_ros import TransformListener, Buffer
from tf2_msgs.msg import TFMessage
from typing import Tuple, Optional

# ============================================================================
# Structured Event Logging
# ============================================================================
# Create logs directory
os.makedirs('/workspace/logs/events', exist_ok=True)

# Configure structured event logger
event_logger = logging.getLogger('drive_test_events')
event_logger.setLevel(logging.DEBUG)

# File handler with rotation (10MB files, 5 backups)
file_handler = RotatingFileHandler(
    '/workspace/logs/events/drive_test_events.log',
    maxBytes=10*1024*1024,
    backupCount=5
)

# Structured format: timestamp | level | cycle | event | message
formatter = logging.Formatter(
    '%(asctime)s | %(levelname)-8s | cycle=%(cycle)s | event=%(event)s | %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
file_handler.setFormatter(formatter)
event_logger.addHandler(file_handler)

# Also log to console if verbose
console_handler = logging.StreamHandler()
console_handler.setFormatter(formatter)
console_handler.setLevel(logging.INFO)
event_logger.addHandler(console_handler)

# ============================================================================
# Timeout constants (seconds)
DEFAULT_OVERALL_TIMEOUT_SECONDS = 30.0       # Default timeout for complete drive test
SERVICE_WAIT_TIMEOUT_SECONDS = 30.0          # Timeout waiting for service to become available
SERVICE_CALL_TIMEOUT_SECONDS = 10.0          # Timeout for service call to return
SPIN_POLL_TIMEOUT_SECONDS = 0.2              # rclpy.spin_once() timeout in main loop
RESULT_WAIT_TIMEOUT_SECONDS = 0.1            # rclpy.spin_once() timeout waiting for result message

# TF validation timeouts (after reset/restart, frames take time to appear)
EKF_READINESS_TIMEOUT_SECONDS = 10.0         # Timeout waiting for EKF to publish filtered odometry
TF_FRAMES_EXIST_TIMEOUT_SECONDS = 10.0       # Timeout waiting for required frames to appear in TF tree
TF_CONNECTIVITY_TIMEOUT_SECONDS = 10.0       # Timeout waiting for TF tree to be fully connected
TF_CONNECTIVITY_LOG_DELAY_SECONDS = 2.0      # Delay before logging connectivity wait (avoid spam)
TF_VALIDATION_TIMEOUT_SECONDS = 5.0          # Timeout for final transform lookup validation
TF_RATE_DIAGNOSTIC_DURATION_SECONDS = 2.0    # Duration to sample TF publishing rate for diagnostics
TF_RATE_DIAGNOSTIC_FAST_SECONDS = 0.5        # Shortened duration for warm-reset cycles


class DriveTestRunner(Node):
    """Single-shot test runner that executes one drive test and reports results"""

    def __init__(self, instruction_file: str = 'calibrate_1m',
                 json_output: bool = False, verbose: bool = True,
                 record: bool = False, bag_output_dir: str = '/workspace/logs/rosbags'):
        super().__init__('drive_test_runner')

        self.instruction_file = instruction_file
        self.json_output = json_output
        self.verbose = verbose
        self.record = record
        self.bag_output_dir = Path(bag_output_dir)
        self.bag_recorder = None
        self.bag_path = None
        self.instance_id = None

        # QoS matching test_drive node
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE

        # State from topics
        self.executing = None
        self.has_error = None
        self.result = None

        # Subscriptions
        self.sub_exec = self.create_subscription(
            Bool, '/test_drive/executing', self.on_exec, qos)
        self.sub_err = self.create_subscription(
            Bool, '/test_drive/error', self.on_err, qos)
        self.sub_res = self.create_subscription(
            String, '/test_drive/result', self.on_res, qos)

        # Service clients
        self.reset_cli = self.create_client(Trigger, '/stackReset')
        self.testdrive_cli = self.create_client(ExecuteScenario, '/testDrive')

        # TF listener for transform validation (must be created at startup, not ad-hoc)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_exec(self, msg: Bool):
        self.executing = msg.data

    def on_err(self, msg: Bool):
        self.has_error = msg.data

    def on_res(self, msg: String):
        self.result = msg.data

    def log(self, msg: str):
        """Print message if verbose"""
        if self.verbose and not self.json_output:
            print(msg, flush=True)

    def wait_service(self, cli, name: str, timeout_s: float = SERVICE_WAIT_TIMEOUT_SECONDS):
        """Wait for service to be available"""
        self.log(f"Waiting for service {name}...")
        if not cli.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(f"Service {name} not available after {timeout_s}s")
        self.log(f"  {name} ready")

    def call_trigger(self, cli, name: str, timeout_s: float = SERVICE_CALL_TIMEOUT_SECONDS) -> str:
        """Call a Trigger service and return message"""
        self.log(f"Calling {name}...")
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_s)

        if fut.result() is None:
            raise RuntimeError(f"Service call {name} timed out")

        if not fut.result().success:
            raise RuntimeError(f"Service {name} rejected: {fut.result().message}")

        return fut.result().message

    def set_instance_id_parameter(self, instance_id: str, timeout_s: float = 5.0) -> bool:
        """
        Set instance_id parameter on test_drive node.

        This enables save_map instructions to use the same instance_id as rosbag recordings,
        keeping maps and rosbags correlated in the same directory structure.

        Args:
            instance_id: Instance identifier (e.g., "test_cycle001_20260201_200610")
            timeout_s: Timeout for parameter setting

        Returns:
            True if parameter set successfully
        """
        self.log(f"Setting instance_id parameter: {instance_id}")

        try:
            # Use subprocess to call ros2 param set
            # This avoids needing to create a SetParameters client
            cmd = f'source /opt/ros/jazzy/setup.bash && ros2 param set /test_drive instance_id "{instance_id}"'
            result = subprocess.run(
                cmd,
                shell=True,
                executable='/bin/bash',
                capture_output=True,
                text=True,
                timeout=timeout_s
            )

            if result.returncode == 0:
                self.log(f"  instance_id parameter set successfully")
                return True
            else:
                self.log(f"  Warning: Failed to set parameter: {result.stderr[:200]}")
                return False

        except Exception as e:
            self.log(f"  Warning: Failed to set parameter: {e}")
            return False

    def call_testdrive(self, instruction_file: str, timeout_s: float = SERVICE_CALL_TIMEOUT_SECONDS) -> bool:
        """Call /testDrive service with instruction file"""
        self.log(f"Calling /testDrive with {instruction_file}...")

        req = ExecuteScenario.Request()
        req.scenario.scenario_file = instruction_file

        fut = self.testdrive_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_s)

        if fut.result() is None:
            raise RuntimeError(f"Service call /testDrive timed out")

        if not fut.result().result:
            raise RuntimeError(f"Service /testDrive rejected request (check node logs)")

        return True

    def verify_tf_frames_exist(self, timeout_s: float = TF_FRAMES_EXIST_TIMEOUT_SECONDS) -> Tuple[bool, Optional[str]]:
        """
        Wait for required TF frames to exist in the tree.

        After resets, frames may take time to appear:
        - robot_state_publisher needs to publish URDF-based frames (base_link, etc.)
        - EKF needs to start publishing odom->base_link transform

        Args:
            timeout_s: Maximum time to wait for frames to appear

        Returns:
            (success, error_message): success=True if all frames exist, error_message=None on success
        """
        self.log("Waiting for TF frames to appear...")

        required_frames = ['odom', 'base_link']  # Jetacker uses base_link as primary base frame
        start_time = self.get_clock().now()
        frame_appearance_times = {}
        last_logged_state = None

        while (self.get_clock().now() - start_time).nanoseconds < timeout_s * 1e9:
            # Get all frames currently in TF tree
            all_frames = self.tf_buffer.all_frames_as_string()
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9

            # Track when each frame first appears
            for frame in required_frames:
                if frame in all_frames and frame not in frame_appearance_times:
                    frame_appearance_times[frame] = elapsed
                    self.log(f"    ✓ Frame '{frame}' appeared after {elapsed:.2f}s")

            # Check which required frames exist
            missing_frames = [f for f in required_frames if f not in all_frames]

            if not missing_frames:
                elapsed_total = (self.get_clock().now() - start_time).nanoseconds / 1e9
                self.log(f"  ✓ All required frames exist after {elapsed_total:.2f}s: {', '.join(required_frames)}")
                return True, None

            # Log progress only when state changes (avoid spam)
            existing = [f for f in required_frames if f in all_frames]
            current_state = (tuple(existing), tuple(missing_frames))
            if current_state != last_logged_state and existing:
                self.log(f"    Waiting... Found: {', '.join(existing)}, Missing: {', '.join(missing_frames)}")
                last_logged_state = current_state

            rclpy.spin_once(self, timeout_sec=0.5)

        # Timeout - some frames still missing
        missing_frames = [f for f in required_frames if f not in self.tf_buffer.all_frames_as_string()]
        existing = [f for f in required_frames if f in self.tf_buffer.all_frames_as_string()]

        error_msg = f"Frames missing after {timeout_s}s: {', '.join(missing_frames)}"
        if existing:
            error_msg += f" (found: {', '.join(existing)})"
        if frame_appearance_times:
            timing_info = ', '.join([f"{k}@{v:.2f}s" for k, v in frame_appearance_times.items()])
            error_msg += f" [Timing: {timing_info}]"

        self.log(f"  ✗ Frame check FAILED: {error_msg}")
        return False, error_msg

    def verify_tf_tree(self, timeout_s: float = TF_VALIDATION_TIMEOUT_SECONDS) -> Tuple[bool, Optional[str]]:
        """
        Verify that odom->base_link transform exists.

        Args:
            timeout_s: Maximum time to wait for transform

        Returns:
            (success, error_message): success=True if transform exists, error_message=None on success
        """
        self.log("Verifying TF tree: odom->base_link...")

        try:
            # Use instance TF buffer (already subscribed and receiving data)
            transform = self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=timeout_s)
            )

            self.log("  ✓ TF tree healthy: odom->base_link exists")
            return True, None

        except Exception as e:
            error_msg = f"{type(e).__name__}: {str(e)}"
            self.log(f"  ✗ TF tree BROKEN: {error_msg}")
            return False, error_msg

    def verify_ekf_publishing(self, timeout_s: float = EKF_READINESS_TIMEOUT_SECONDS) -> Tuple[bool, Optional[str]]:
        """
        Wait for EKF to start publishing filtered odometry after reset.

        After Gazebo resets or node restarts, EKF clears its TF buffer and needs to:
        1. Receive controller odometry messages
        2. Start filtering and publishing

        This check ensures the publishing chain is complete before TF validation.

        Args:
            timeout_s: Maximum time to wait for EKF to start publishing

        Returns:
            (success, error_message): success=True if EKF publishing, error_message=None on success
        """
        from nav_msgs.msg import Odometry

        self.log("Waiting for EKF to start publishing filtered odometry...")

        msg_received = False

        def callback(msg):
            nonlocal msg_received
            msg_received = True

        # Temporary subscription to check if EKF is publishing
        sub = self.create_subscription(Odometry, '/odometry/filtered', callback, 10)

        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds < timeout_s * 1e9:
            if msg_received:
                self.destroy_subscription(sub)
                self.log("  ✓ EKF publishing filtered odometry")
                return True, None
            rclpy.spin_once(self, timeout_sec=0.1)

        # Timeout - EKF not publishing
        self.destroy_subscription(sub)
        error_msg = f"EKF not publishing after {timeout_s}s timeout"
        self.log(f"  ✗ EKF check FAILED: {error_msg}")
        return False, error_msg

    def check_tf_publishing_rate(self, sample_duration: float = TF_RATE_DIAGNOSTIC_DURATION_SECONDS) -> dict:
        """
        Monitor /tf publishing rate for diagnostics.

        Args:
            sample_duration: How long to sample (seconds)

        Returns:
            dict with: message_count, rate_hz, frames_seen
        """
        from collections import defaultdict

        message_count = 0
        frames = defaultdict(int)

        def tf_callback(msg):
            nonlocal message_count
            message_count += 1
            for transform in msg.transforms:
                frame_pair = f"{transform.header.frame_id} → {transform.child_frame_id}"
                frames[frame_pair] += 1

        # Temporary subscription for monitoring
        sub = self.create_subscription(TFMessage, '/tf', tf_callback, 10)

        # Sample for specified duration
        start = self.get_clock().now()
        while (self.get_clock().now() - start).nanoseconds < sample_duration * 1e9:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Cleanup
        self.destroy_subscription(sub)

        rate_hz = message_count / sample_duration if sample_duration > 0 else 0

        return {
            'message_count': message_count,
            'rate_hz': rate_hz,
            'frames_seen': dict(frames)
        }

    def start_recording(self, cycle_num: int = None, test_timeout_s: float = None) -> bool:
        """
        Start in-process rosbag recording via BagRecorder.

        Args:
            cycle_num: Cycle number for naming
            test_timeout_s: Unused (kept for API compatibility)

        Returns:
            True if recording started successfully
        """
        if not self.record:
            return True

        from bag_recorder import BagRecorder, _drive_record_topics, make_bag_path

        # Generate bag name with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        cycle_str = f"_cycle{cycle_num:03d}" if cycle_num is not None else ""
        bag_name = f"test{cycle_str}_{timestamp}"

        bag_path = make_bag_path(str(self.bag_output_dir), cycle_num, prefix='test')
        self.bag_path = Path(bag_path)
        self.instance_id = bag_name

        self.log(f"Starting rosbag recording: {bag_name}")

        try:
            self.bag_recorder = BagRecorder(self, bag_path, _drive_record_topics())
            self.log(f"  Recording to: {self.bag_path}")
            return True
        except Exception as e:
            self.log(f"  Warning: Failed to start recording: {e}")
            self.bag_recorder = None
            return False

    def stop_recording(self) -> bool:
        """
        Stop in-process rosbag recording.

        Returns:
            True if stopped successfully
        """
        if not self.record or self.bag_recorder is None:
            return True

        self.log("Stopping rosbag recording...")

        try:
            self.bag_recorder.close()
            self.log("  Recording stopped")
            return True
        except Exception as e:
            self.log(f"  Warning: Error stopping recording: {e}")
            return False
        finally:
            self.bag_recorder = None

    def run(self, overall_timeout_s: float = DEFAULT_OVERALL_TIMEOUT_SECONDS, cycle_num: int = None,
            fast_preflight: bool = False) -> dict:
        """
        Execute one drive test cycle.

        Args:
            overall_timeout_s: Maximum time to wait for test completion
            cycle_num: Optional cycle number for rosbag naming

        Returns:
            dict with keys: success (bool), message (str), result (dict or None), bag_path (str or None)
        """
        # Track rosbag recording status
        self.rosbag_recording_ok = True  # Will be set to False if recording fails

        # Create cycle context for logging
        extra = {'cycle': cycle_num or 0, 'event': ''}

        extra['event'] = 'test_start'
        event_logger.info(f'Test execution started: {self.instruction_file}', extra=extra)

        try:
            # Wait for services
            self.wait_service(self.reset_cli, '/stackReset', timeout_s=SERVICE_WAIT_TIMEOUT_SECONDS)
            self.wait_service(self.testdrive_cli, '/testDrive', timeout_s=SERVICE_WAIT_TIMEOUT_SECONDS)

            extra['event'] = 'services_ready'
            event_logger.info('All services available', extra=extra)

            # Wait for EKF to start publishing (ensures full publishing chain is ready)
            extra['event'] = 'ekf_readiness'
            ekf_success, ekf_error = self.verify_ekf_publishing(timeout_s=EKF_READINESS_TIMEOUT_SECONDS)
            if not ekf_success:
                event_logger.error(f'EKF readiness check FAILED: {ekf_error}', extra=extra)
                return {
                    'success': False,
                    'message': f'EKF not ready: {ekf_error}',
                    'result': None,
                    'bag_path': str(self.bag_path) if self.bag_path else None
                }
            event_logger.info('EKF publishing filtered odometry', extra=extra)

            # Wait for required TF frames to exist AND be connected
            extra['event'] = 'tf_frames_check'
            frames_success, frames_error = self.verify_tf_frames_exist(timeout_s=TF_FRAMES_EXIST_TIMEOUT_SECONDS)
            if not frames_success:
                event_logger.error(f'TF frames check FAILED: {frames_error}', extra=extra)
                return {
                    'success': False,
                    'message': f'TF frames missing: {frames_error}',
                    'result': None,
                    'bag_path': str(self.bag_path) if self.bag_path else None
                }
            event_logger.info('All required TF frames exist', extra=extra)

            # Extra validation: Wait for frames to be connected (not just existing)
            # This catches the case where EKF publishes odometry but not TF yet
            extra['event'] = 'tf_connectivity_check'
            self.log("Waiting for TF tree connectivity (odom->base_link)...")
            connectivity_start = self.get_clock().now()
            connectivity_timeout = TF_CONNECTIVITY_TIMEOUT_SECONDS
            connected = False

            while (self.get_clock().now() - connectivity_start).nanoseconds < connectivity_timeout * 1e9:
                try:
                    # Try to query the transform to verify connection exists
                    self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=1.0))
                    elapsed = (self.get_clock().now() - connectivity_start).nanoseconds / 1e9
                    self.log(f"  ✓ TF tree connected after {elapsed:.2f}s")
                    event_logger.info(f'TF tree connectivity verified after {elapsed:.2f}s', extra=extra)
                    connected = True
                    break
                except Exception as e:
                    # Not connected yet, keep waiting
                    elapsed = (self.get_clock().now() - connectivity_start).nanoseconds / 1e9
                    if elapsed > TF_CONNECTIVITY_LOG_DELAY_SECONDS:  # Only log after delay to avoid spam
                        self.log(f"    Still waiting for connectivity... ({elapsed:.1f}s elapsed)")
                    rclpy.spin_once(self, timeout_sec=0.5)

            if not connected:
                error_msg = "TF tree not connected after 10s (frames exist but EKF not publishing transforms)"
                event_logger.error(f'TF connectivity check FAILED: {error_msg}', extra=extra)
                return {
                    'success': False,
                    'message': f'TF connectivity timeout: {error_msg}',
                    'result': None,
                    'bag_path': str(self.bag_path) if self.bag_path else None
                }

            # Verify TF tree before running test
            extra['event'] = 'tf_validation'
            tf_success, tf_error = self.verify_tf_tree(timeout_s=TF_VALIDATION_TIMEOUT_SECONDS)
            if not tf_success:
                event_logger.error(f'TF tree validation FAILED: {tf_error}', extra=extra)
                return {
                    'success': False,
                    'message': f'TF tree validation failed: {tf_error}',
                    'result': None,
                    'bag_path': str(self.bag_path) if self.bag_path else None
                }
            event_logger.info('TF tree validated successfully', extra=extra)

            # Diagnostic: Check TF publishing rate
            extra['event'] = 'tf_diagnostics'
            tf_diag_duration = TF_RATE_DIAGNOSTIC_FAST_SECONDS if fast_preflight else TF_RATE_DIAGNOSTIC_DURATION_SECONDS
            tf_rate_info = self.check_tf_publishing_rate(sample_duration=tf_diag_duration)
            event_logger.info(
                f'TF publishing rate: {tf_rate_info["rate_hz"]:.1f} Hz, '
                f'{tf_rate_info["message_count"]} messages in {tf_diag_duration}s',
                extra=extra
            )

            # Reset state
            try:
                reset_msg = self.call_trigger(self.reset_cli, '/stackReset', timeout_s=SERVICE_CALL_TIMEOUT_SECONDS)
                self.log(f"  Reset: {reset_msg}")
                extra['event'] = 'reset_complete'
                event_logger.info(f'Reset complete: {reset_msg}', extra=extra)
            except Exception as e:
                self.log(f"  Warning: Reset failed (continuing): {e}")
                extra['event'] = 'reset_warning'
                event_logger.warning(f'Reset failed (continuing): {e}', extra=extra)

            # Start rosbag recording BEFORE test starts
            recording_started = self.start_recording(cycle_num, test_timeout_s=overall_timeout_s)

            # Set instance_id parameter on test_drive node for map save correlation
            # If recording is disabled, generate instance_id from cycle_num
            if self.instance_id is None:
                # Generate instance_id even if not recording (for save_map instructions)
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                cycle_str = f"_cycle{cycle_num:03d}" if cycle_num is not None else ""
                self.instance_id = f"test{cycle_str}_{timestamp}"

            self.set_instance_id_parameter(self.instance_id)

            # Start drive test with instruction file
            self.call_testdrive(self.instruction_file, timeout_s=SERVICE_CALL_TIMEOUT_SECONDS)
            self.log(f"  Started: {self.instruction_file}")

            extra['event'] = 'test_drive_start'
            event_logger.info(f'Started test with rosbag={recording_started}', extra=extra)

            # Wait for test to complete
            self.log("Waiting for test to complete...")
            deadline = self.get_clock().now().nanoseconds + int(overall_timeout_s * 1e9)
            started = False
            last_dot = self.get_clock().now().nanoseconds

            while self.get_clock().now().nanoseconds < deadline:
                rclpy.spin_once(self, timeout_sec=SPIN_POLL_TIMEOUT_SECONDS)

                # Print progress dots every 0.5s
                now = self.get_clock().now().nanoseconds
                if self.verbose and not self.json_output and (now - last_dot) > 0.5e9:
                    print(".", end="", flush=True)
                    last_dot = now

                # State machine: wait for executing=true, then wait for executing=false
                if self.executing is True:
                    started = True

                # Wait for completion: executing=false AND has_error is set (ensure all messages arrived)
                if started and self.executing is False and self.has_error is not None:
                    # Test completed - give a tiny bit more time for result message to arrive
                    for _ in range(5):
                        rclpy.spin_once(self, timeout_sec=RESULT_WAIT_TIMEOUT_SECONDS)
                        if self.result is not None:
                            break

                    if self.verbose and not self.json_output:
                        print()  # Newline after progress dots

                    # Parse result JSON
                    result_data = None
                    if self.result:
                        try:
                            result_data = json.loads(self.result)
                        except json.JSONDecodeError:
                            result_data = {'raw': self.result}

                    if self.has_error:
                        extra['event'] = 'test_failed'
                        event_logger.error(f'Test failed: {self.result or "unknown error"}', extra=extra)
                        return {
                            'success': False,
                            'message': f"Test failed: {self.result or 'unknown error'}",
                            'result': result_data,
                            'bag_path': str(self.bag_path) if self.bag_path else None
                        }
                    else:
                        extra['event'] = 'test_complete'
                        event_logger.info('Test completed successfully', extra=extra)

                        # Check if rosbag recording will succeed (it runs in finally block)
                        # We can't check it here, so we'll return and the finally block will update if needed
                        test_result = {
                            'success': True,
                            'message': 'Test completed successfully',
                            'result': result_data,
                            'bag_path': str(self.bag_path) if self.bag_path else None
                        }
                        return test_result

            # Timeout
            extra['event'] = 'test_timeout'
            if not started:
                event_logger.error('Timeout: test never started', extra=extra)
                raise RuntimeError("Timeout: test never started")
            else:
                event_logger.error(f'Timeout: test did not complete in {overall_timeout_s}s', extra=extra)
                raise RuntimeError(f"Timeout: test did not complete in {overall_timeout_s}s")

        except Exception as e:
            extra['event'] = 'test_error'
            event_logger.exception('Test execution failed with exception', extra=extra)
            raise
        finally:
            # Always stop recording, even on timeout/error
            recording_ok = self.stop_recording()
            self.rosbag_recording_ok = recording_ok
            if not recording_ok:
                print("ERROR: Rosbag recording did not close cleanly", file=sys.stderr)
                print(f"  Bag path: {self.bag_path}", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description='Run a single drive test')
    parser.add_argument('--instruction', type=str, default='calibrate_1m',
                        help='Instruction file to execute (default: calibrate_1m)')
    parser.add_argument('--timeout', type=float, default=DEFAULT_OVERALL_TIMEOUT_SECONDS,
                        help=f'Overall test timeout in seconds (default: {DEFAULT_OVERALL_TIMEOUT_SECONDS:.0f})')
    parser.add_argument('--json', action='store_true',
                        help='Output results as JSON')
    parser.add_argument('--quiet', action='store_true',
                        help='Suppress progress messages')
    parser.add_argument('--record', action='store_true',
                        help='Record rosbag2 during test')
    parser.add_argument('--cycle-num', type=int, default=None,
                        help='Cycle number for rosbag naming')
    parser.add_argument('--bag-output-dir', type=str, default='/workspace/logs/rosbags',
                        help='Output directory for rosbag files (default: /workspace/logs/rosbags)')
    parser.add_argument('--fast-preflight', action='store_true',
                        help='Shorten TF diagnostics for warm-reset cycles')
    args = parser.parse_args()

    rclpy.init()
    node = DriveTestRunner(
        instruction_file=args.instruction,
        json_output=args.json,
        verbose=not args.quiet,
        record=args.record,
        bag_output_dir=args.bag_output_dir
    )

    try:
        result = node.run(overall_timeout_s=args.timeout, cycle_num=args.cycle_num,
                          fast_preflight=args.fast_preflight)

        # Check rosbag status and override result if recording failed
        if args.record and hasattr(node, 'rosbag_recording_ok') and not node.rosbag_recording_ok:
            result['success'] = False
            result['message'] = 'Test completed but rosbag recording failed'

        if args.json:
            print(json.dumps(result, indent=2))
        else:
            if result['success']:
                print(f"\nSUCCESS: {result['message']}")
                if result['result']:
                    print(f"  Distance: {result['result'].get('final_distance', 'N/A'):.3f}m")
                    print(f"  Error: {result['result'].get('error', 'N/A'):.4f}m")
            else:
                print(f"\nFAILED: {result['message']}")

        return 0 if result['success'] else 1

    except Exception as e:
        if args.json:
            print(json.dumps({'success': False, 'message': str(e), 'result': None}))
        else:
            print(f"\nERROR: {e}", file=sys.stderr)
        return 1

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
