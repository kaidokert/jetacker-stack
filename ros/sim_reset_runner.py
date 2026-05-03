#!/usr/bin/env python3
"""
Simulation Reset Runner - Container-internal reset orchestrator

Runs INSIDE a container (typically gazebo or test-drive) to reset simulation state.
This replaces fragile shell-based reset sequences with native ROS2/Gazebo APIs.

Performs:
1. Gazebo world reset (physics, poses, velocities)
2. Robot-specific resets (EKF for slam_bot, odometry topics)
3. Test-drive internal state reset
4. Wait for topics to stabilize

Usage:
    python3 /workspace/ros/sim_reset_runner.py --stack jetacker
    python3 /workspace/ros/sim_reset_runner.py --stack slam_bot --json
"""

import sys
import argparse
import json
import time
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import Trigger, Empty

# Timeout constants (seconds)
SERVICE_WAIT_TIMEOUT_SECONDS = 10.0       # Timeout waiting for service to become available
SERVICE_CALL_TIMEOUT_SECONDS = 5.0        # Timeout for service call to return
GAZEBO_RESET_TIMEOUT_SECONDS = 5.0        # Timeout for Gazebo world reset
GAZEBO_CMD_BUFFER_SECONDS = 2.0           # Extra time for gz subprocess to complete
SETTLE_TIME_JETACKER_SECONDS = 3.0        # Stabilization wait for Jetacker stack
SETTLE_TIME_SLAM_BOT_SECONDS = 2.0        # Stabilization wait for slam_bot stack


class SimResetRunner(Node):
    """Orchestrates simulation reset across multiple components"""

    def __init__(self, stack_name: str, json_output: bool = False, verbose: bool = True, nav2_mode: bool = False):
        super().__init__('sim_reset_runner')

        self.stack_name = stack_name
        self.json_output = json_output
        self.verbose = verbose
        self.nav2_mode = nav2_mode

        # Determine world name and services based on stack
        if stack_name == 'jetacker':
            self.world_name = 'jetacker_world'
            self.gazebo_service = 'jetacker-gazebo'
            self.needs_ekf_reset = False
        elif stack_name == 'slam_bot':
            self.world_name = 'slam_world'
            self.gazebo_service = 'gazebo'
            self.needs_ekf_reset = True
        else:
            raise ValueError(f"Unknown stack: {stack_name}")

        # Service clients
        self.reset_cli = self.create_client(Trigger, '/stackReset')
        if self.needs_ekf_reset:
            self.ekf_reset_cli = self.create_client(Empty, '/set_pose')

    def log(self, msg: str):
        """Print message if verbose"""
        if self.verbose and not self.json_output:
            print(msg, flush=True)

    def wait_service(self, cli, name: str, timeout_s: float = SERVICE_WAIT_TIMEOUT_SECONDS) -> bool:
        """Wait for service to be available"""
        if not cli.wait_for_service(timeout_sec=timeout_s):
            self.log(f"  Warning: Service {name} not available")
            return False
        return True

    def call_gz_service(self, timeout_s: float = GAZEBO_RESET_TIMEOUT_SECONDS) -> bool:
        """Call Gazebo world reset service using gz command"""
        self.log("  [1/3] Resetting Gazebo world...")

        cmd = [
            'gz', 'service',
            '-s', f'/world/{self.world_name}/control',
            '--reqtype', 'gz.msgs.WorldControl',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(int(timeout_s * 1000)),
            '--req', 'reset: {all: true}'
        ]

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout_s + GAZEBO_CMD_BUFFER_SECONDS,
                check=False
            )

            if result.returncode == 0:
                self.log("    Gazebo world reset complete")
                return True
            else:
                self.log(f"    Warning: Gazebo reset failed: {result.stderr}")
                return False

        except subprocess.TimeoutExpired:
            self.log("    Warning: Gazebo reset timed out")
            return False
        except FileNotFoundError:
            self.log("    Warning: gz command not found (might not be in this container)")
            return False

    def reset_ekf(self) -> bool:
        """Reset EKF localization (slam_bot only)"""
        if not self.needs_ekf_reset:
            return True

        self.log("  [2/3] Resetting EKF localization...")

        if not self.wait_service(self.ekf_reset_cli, '/set_pose', timeout_s=SERVICE_CALL_TIMEOUT_SECONDS):
            return False

        try:
            fut = self.ekf_reset_cli.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=SERVICE_CALL_TIMEOUT_SECONDS)

            if fut.result() is None:
                self.log("    Warning: EKF reset timed out")
                return False

            self.log("    EKF reset complete")
            return True

        except Exception as e:
            self.log(f"    Warning: EKF reset failed: {e}")
            return False

    def reset_test_drive(self) -> bool:
        """Reset test-drive internal state"""
        self.log("  [3/3] Resetting test-drive state...")

        if not self.wait_service(self.reset_cli, '/stackReset', timeout_s=SERVICE_WAIT_TIMEOUT_SECONDS):
            return False

        try:
            fut = self.reset_cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=SERVICE_CALL_TIMEOUT_SECONDS)

            if fut.result() is None:
                self.log("    Warning: test-drive reset timed out")
                return False

            if not fut.result().success:
                self.log(f"    Warning: test-drive reset failed: {fut.result().message}")
                return False

            self.log("    Test-drive reset complete")
            return True

        except Exception as e:
            self.log(f"    Warning: test-drive reset failed: {e}")
            return False

    def wait_for_controller_ready(self, controller_name: str, timeout_s: float = 20.0) -> bool:
        """
        Wait for controller to be ready (subscription discoverable + actively publishing).

        Polls controller state topic to verify controller is alive and processing.

        Args:
            controller_name: Name of controller (e.g. 'tricycle_steering_controller')
            timeout_s: Maximum time to wait

        Returns:
            True if controller is ready, False if timeout
        """
        import subprocess as sp

        state_topic = f"/{controller_name}/controller_state"
        self.log(f"  Waiting for controller '{controller_name}' to be ready...")

        start_time = time.time()
        successful_reads = 0

        while time.time() - start_time < timeout_s:
            elapsed = time.time() - start_time

            # Try to read controller state topic (proves controller is publishing)
            cmd = f"timeout 2 ros2 topic echo {state_topic} --once --field header.stamp 2>/dev/null"
            try:
                result = sp.run(cmd, shell=True, capture_output=True, text=True, timeout=3.0)

                if result.returncode == 0 and 'sec:' in result.stdout:
                    successful_reads += 1
                    if successful_reads >= 2:
                        self.log(f"    Controller ready ({elapsed:.1f}s)")
                        return True
                    time.sleep(0.3)  # Brief wait before next check
                else:
                    time.sleep(0.5)  # Wait longer if not ready

            except sp.TimeoutExpired:
                pass

        self.log(f"    Warning: Controller ready check timed out ({timeout_s:.0f}s)")
        return False

    def wait_for_nav2_ready(self, timeout_s: float = 120.0) -> bool:
        """
        Wait for Nav2 stack to be ready by checking AMCL pose publication.

        Waits for AMCL to be consistently publishing poses, which indicates
        the Nav2 lifecycle manager has completed activation of all nodes.

        Args:
            timeout_s: Maximum time to wait (default 120s for full Nav2 bringup)

        Returns:
            True if Nav2 ready, False if timeout
        """
        import subprocess as sp

        self.log(f"  Waiting for Nav2 stack (AMCL pose publication)...")
        start_time = time.time()
        successful_checks = 0
        required_checks = 3  # Need 3 consecutive successful checks

        while time.time() - start_time < timeout_s:
            elapsed = time.time() - start_time

            # Check if AMCL is publishing (indicates Nav2 stack is active)
            cmd = "timeout 3 ros2 topic echo /amcl_pose --once --field header.stamp 2>/dev/null"

            try:
                result = sp.run(cmd, shell=True, capture_output=True, text=True, timeout=4.0)

                if result.returncode == 0 and 'sec:' in result.stdout:
                    successful_checks += 1
                    if successful_checks >= required_checks:
                        self.log(f"    Nav2 stack ready ({elapsed:.1f}s)")
                        return True
                    time.sleep(1.0)  # Brief wait between checks
                else:
                    successful_checks = 0  # Reset counter if check fails
                    time.sleep(2.0)  # Wait longer if not ready

            except sp.TimeoutExpired:
                successful_checks = 0
                time.sleep(2.0)

        self.log(f"    Warning: Nav2 ready check timed out ({timeout_s:.0f}s)")
        return False

    def wait_for_stability(self, settle_time: float = 1.5):
        """Wait for topics to stabilize after reset"""
        self.log(f"  Waiting {settle_time}s for stabilization...")
        time.sleep(settle_time)

    def run_gazebo_only(self) -> dict:
        """
        Execute ONLY Gazebo world reset (fast).

        Returns:
            dict with keys: success (bool), message (str), duration (float)
        """
        start_time = time.time()

        self.log(f"\n{'='*50}")
        self.log(f"  Gazebo World Reset ({self.stack_name})")
        self.log(f"{'='*50}\n")

        success = self.call_gz_service(timeout_s=GAZEBO_RESET_TIMEOUT_SECONDS)
        duration = time.time() - start_time

        if success:
            self.log(f"\n{'='*50}")
            self.log(f"  Gazebo reset complete ({duration:.1f}s)")
            self.log(f"{'='*50}\n")
            return {
                'success': True,
                'message': f'Gazebo reset complete in {duration:.1f}s',
                'duration': duration
            }
        else:
            return {
                'success': False,
                'message': 'Gazebo reset failed',
                'duration': duration
            }

    def run_wait_and_reset(self) -> dict:
        """
        Wait for controller ready, then reset ROS nodes.

        Performs:
        1. Wait for controller to be ready (if jetacker)
        2. Reset EKF (if slam_bot)
        3. Reset test-drive internal state
        4. Wait for Nav2 to be ready (if nav2_mode)
        5. Final stabilization wait

        Returns:
            dict with keys: success (bool), message (str), duration (float), steps (dict)
        """
        start_time = time.time()

        mode_str = f"{self.stack_name} + Nav2" if self.nav2_mode else self.stack_name
        self.log(f"\n{'='*50}")
        self.log(f"  Wait & Reset ({mode_str})")
        self.log(f"{'='*50}\n")

        steps = {
            'controller_ready': False,
            'ekf_reset': True if not self.needs_ekf_reset else False,
            'test_drive_reset': False,
            'nav2_ready': True if not self.nav2_mode else False,
            'stabilized': False
        }

        # Step 1: Wait for controller to be ready (jetacker only)
        if self.stack_name == 'jetacker':
            self.log("  [1/3] Waiting for controller ready...")
            steps['controller_ready'] = self.wait_for_controller_ready(
                'tricycle_steering_controller', timeout_s=20.0)
        else:
            self.log("  [1/3] Waiting for controller ready...")
            steps['controller_ready'] = self.wait_for_controller_ready(
                'diff_drive_controller', timeout_s=15.0)

        # Step 2: Reset EKF (slam_bot only)
        if self.needs_ekf_reset:
            self.log("  [2/3] Resetting EKF localization...")
            steps['ekf_reset'] = self.reset_ekf()

        # Step 3: Reset test-drive
        self.log(f"  [{3 if not self.needs_ekf_reset else 3}/3] Resetting test-drive state...")
        steps['test_drive_reset'] = self.reset_test_drive()

        # Step 4: Wait for Nav2 to be ready (if nav2_mode)
        if self.nav2_mode:
            step_num = 4 if not self.needs_ekf_reset else 4
            self.log(f"  [{step_num}/{step_num}] Waiting for Nav2 lifecycle manager...")
            steps['nav2_ready'] = self.wait_for_nav2_ready(timeout_s=120.0)

        # Step 5: Final stabilization
        settle_time = SETTLE_TIME_JETACKER_SECONDS if self.stack_name == 'jetacker' else SETTLE_TIME_SLAM_BOT_SECONDS
        self.wait_for_stability(settle_time)
        steps['stabilized'] = True

        duration = time.time() - start_time

        # Success if test-drive reset succeeded (and Nav2 ready if nav2_mode)
        # Controller ready check is best-effort - even if it times out, the test may still work
        # (EKF is optional for slam_bot)
        all_critical_steps_passed = steps['test_drive_reset'] and steps['nav2_ready']

        if all_critical_steps_passed:
            self.log(f"\n{'='*50}")
            self.log(f"  Wait & reset complete ({duration:.1f}s)")
            self.log(f"{'='*50}\n")
            return {
                'success': True,
                'message': f'Wait & reset complete in {duration:.1f}s',
                'duration': duration,
                'steps': steps
            }
        else:
            self.log(f"\n{'='*50}")
            self.log(f"  Wait & reset completed with warnings ({duration:.1f}s)")
            self.log(f"{'='*50}\n")
            return {
                'success': False,
                'message': 'Wait & reset completed with warnings',
                'duration': duration,
                'steps': steps
            }

    def run(self) -> dict:
        """
        Execute full reset sequence.

        Returns:
            dict with keys: success (bool), message (str), steps (dict)
        """
        start_time = time.time()

        self.log(f"\n{'='*50}")
        self.log(f"  Simulation Reset ({self.stack_name})")
        self.log(f"{'='*50}\n")

        # Track which steps succeeded
        steps = {
            'gazebo_reset': False,
            'ekf_reset': True if not self.needs_ekf_reset else False,
            'test_drive_reset': False,
            'stabilized': False
        }

        # Step 1: Gazebo world reset
        steps['gazebo_reset'] = self.call_gz_service(timeout_s=GAZEBO_RESET_TIMEOUT_SECONDS)

        # Step 2: Robot-specific resets
        if self.needs_ekf_reset:
            steps['ekf_reset'] = self.reset_ekf()

        # Step 3: Test-drive reset
        steps['test_drive_reset'] = self.reset_test_drive()

        # Step 4: Wait for stabilization
        settle_time = SETTLE_TIME_JETACKER_SECONDS if self.stack_name == 'jetacker' else SETTLE_TIME_SLAM_BOT_SECONDS
        self.wait_for_stability(settle_time)
        steps['stabilized'] = True

        duration = time.time() - start_time

        # Determine overall success
        all_critical_steps_passed = (
            steps['gazebo_reset'] and
            steps['ekf_reset'] and
            steps['test_drive_reset']
        )

        if all_critical_steps_passed:
            self.log(f"\n{'='*50}")
            self.log(f"  Reset complete ({duration:.1f}s)")
            self.log(f"{'='*50}\n")
            return {
                'success': True,
                'message': f'Reset complete in {duration:.1f}s',
                'duration': duration,
                'steps': steps
            }
        else:
            self.log(f"\n{'='*50}")
            self.log(f"  Reset completed with warnings ({duration:.1f}s)")
            self.log(f"{'='*50}\n")
            return {
                'success': False,
                'message': 'Reset completed with warnings',
                'duration': duration,
                'steps': steps
            }


def detect_stack() -> str:
    """
    Auto-detect which stack is running by checking container names.

    Returns:
        'jetacker', 'slam_bot', or None
    """
    try:
        # Check for jetacker-gazebo
        result = subprocess.run(
            ['docker', 'ps', '--filter', 'name=jetacker-gazebo', '--format', '{{.Names}}'],
            capture_output=True,
            text=True,
            check=False
        )
        if 'jetacker-gazebo' in result.stdout:
            return 'jetacker'

        # Check for slam gazebo (just "gazebo" without prefix)
        result = subprocess.run(
            ['docker', 'ps', '--filter', 'name=^gazebo$', '--format', '{{.Names}}'],
            capture_output=True,
            text=True,
            check=False
        )
        if result.stdout.strip() == 'gazebo':
            return 'slam_bot'

        return None

    except Exception:
        return None


def main():
    parser = argparse.ArgumentParser(description='Reset simulation state')
    parser.add_argument('--stack', choices=['jetacker', 'slam_bot'],
                        help='Stack name (auto-detected if not specified)')
    parser.add_argument('--json', action='store_true',
                        help='Output results as JSON')
    parser.add_argument('--quiet', action='store_true',
                        help='Suppress progress messages')
    parser.add_argument('--nav2', action='store_true',
                        help='Enable Nav2 mode (wait for Nav2 lifecycle manager)')

    # Mode selection (mutually exclusive)
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument('--gazebo-only', action='store_true',
                           help='Only reset Gazebo world (no node waiting/reset)')
    mode_group.add_argument('--wait-and-reset', action='store_true',
                           help='Wait for controller ready, then reset nodes')

    args = parser.parse_args()

    # Auto-detect stack if not specified
    stack_name = args.stack
    if not stack_name:
        stack_name = detect_stack()
        if not stack_name:
            print("ERROR: Could not auto-detect stack. Use --stack to specify.", file=sys.stderr)
            return 1

    rclpy.init()
    node = SimResetRunner(stack_name, json_output=args.json, verbose=not args.quiet, nav2_mode=args.nav2)

    try:
        # Select mode
        if args.gazebo_only:
            result = node.run_gazebo_only()
        elif args.wait_and_reset:
            result = node.run_wait_and_reset()
        else:
            # Default: full reset (backward compatible)
            result = node.run()

        if args.json:
            print(json.dumps(result, indent=2))
        # else: already printed progress

        return 0 if result['success'] else 1

    except Exception as e:
        if args.json:
            print(json.dumps({
                'success': False,
                'message': str(e),
                'duration': 0,
                'steps': {}
            }))
        else:
            print(f"\nERROR: {e}", file=sys.stderr)
        return 1

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
