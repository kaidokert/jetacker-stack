#!/usr/bin/env python3
"""
Nav2 Waypoint Test Runner (YAML-driven) — same CLI and behavior as
test_nav2_waypoints.py but derives all stack topology from stack.py / stacks.yaml
instead of hardcoded stack_definitions.py.

Reset modes:
  --warm-reset: Fast teleport reset (~5s) — pause, teleport, zero joints, unpause
  (default):    Full orchestrator reset (~50s) — stop, gazebo reset, restart, gates

Usage:
    # Single test
    python drive_nav2.py --waypoints gym_loop_nav2 --cycles 1

    # Multi-cycle testing with warm reset
    python drive_nav2.py --waypoints gym_loop_nav2 --cycles 10 --warm-reset

    # Stop on first failure
    python drive_nav2.py --cycles 10 --stop-on-error
"""

import subprocess
import sys
import argparse
import json
import math
import time
from pathlib import Path
from typing import Optional, Tuple

import yaml

# Import shared functions
sys.path.insert(0, str(Path(__file__).parent / 'tools'))
from subprocess_utils import docker_exec as _docker_exec
from drive_utils import (
    detect_stack,
    verify_controller_loaded,
    RED, GREEN, YELLOW, BLUE, CYAN, NC,
)

# Import new declarative reset orchestrator
from reset_orchestrator import ResetOrchestrator

# Import YAML-driven stack topology
from stack import load_manifest, build_reset_definition, get_reset_conflicts, soft_reset


# Timeout constants
DEFAULT_TEST_TIMEOUT_SECONDS = 120.0
SUBPROCESS_TIMEOUT_BUFFER_SECONDS = 10.0


def run_nav2_sim_reset(stack='jetacker', quiet=False, use_amcl=True):
    """
    Reset simulation for Nav2 testing using declarative orchestrator.

    Uses deterministic reset orchestrator with:
    - Dependency-aware node ordering
    - Explicit readiness gates
    - Gazebo world reset without container restart

    Stack topology derived from stacks.yaml via stack.py.

    Args:
        stack: Stack name ('jetacker' or 'slam_bot')
        quiet: Suppress progress output
        use_amcl: Use AMCL localization (True) or odometry-only (False)

    Returns:
        (success, result_dict)
    """
    if stack != 'jetacker':
        return False, {'success': False, 'message': f'Only jetacker stack supported, got: {stack}'}

    try:
        manifest = load_manifest()
        stack_name = 'nav2' if use_amcl else 'nav2_odom'

        stack_definition = build_reset_definition(manifest, 'jetacker', stack_name)
        conflicts = get_reset_conflicts(manifest, 'jetacker', stack_name)

        orchestrator = ResetOrchestrator(
            stack_definition,
            stack_name=stack_name,
            conflicting_services=conflicts,
        )

        start_time = time.time()

        # Run full deterministic reset
        success = orchestrator.reset_stack_full()

        duration = time.time() - start_time

        if success:
            return True, {
                'success': True,
                'message': 'Reset complete',
                'duration': duration
            }
        else:
            return False, {
                'success': False,
                'message': 'Orchestrator reset failed (check logs)',
                'duration': duration
            }

    except Exception as e:
        return False, {
            'success': False,
            'message': f'Reset orchestrator exception: {str(e)}'
        }


def run_warm_reset(quiet=False, cycle_num=0):
    """Reset simulation using warm teleport reset (stack.py soft-reset).

    Much faster than full orchestrator reset (~5s vs ~50s).
    Teleports model to origin, zeros joints, resets EKF + AMCL + costmaps.

    Returns:
        (success, result_dict)
    """
    start_time = time.time()
    try:
        soft_reset()
        duration = time.time() - start_time
        return True, {
            'success': True,
            'message': 'Warm reset complete',
            'duration': duration,
        }
    except SystemExit:
        duration = time.time() - start_time
        return False, {
            'success': False,
            'message': 'Warm reset failed',
            'duration': duration,
        }


def _load_waypoint_tolerances(waypoint_file: str) -> Optional[Tuple[float, float]]:
    """Load tolerance_xy and tolerance_yaw from a waypoint YAML file.

    Returns (tolerance_xy, tolerance_yaw) or None if not found.
    """
    yaml_path = Path(__file__).parent / 'driving_instructions' / f'{waypoint_file}.yaml'
    if not yaml_path.exists():
        return None
    try:
        with open(yaml_path) as f:
            data = yaml.safe_load(f)
        waypoints = data.get('waypoints', [])
        if not waypoints:
            return None
        last_wp = waypoints[-1]
        tol_xy = last_wp.get('tolerance_xy')
        tol_yaw = last_wp.get('tolerance_yaw')
        if tol_xy is not None and tol_yaw is not None:
            return (float(tol_xy), float(tol_yaw))
    except Exception as e:
        print(f"{YELLOW}  WARN: Failed to load tolerances from {yaml_path}: {e}{NC}",
              file=sys.stderr)
    return None


def _format_param_value(value) -> str:
    """Format a parameter value for the ROS2 SetParameters service.

    ROS2 ParameterType: BOOL=1, INTEGER=2, DOUBLE=3, STRING=4.
    """
    if isinstance(value, bool):
        return f"type: 1, bool_value: {str(value).lower()}"
    elif isinstance(value, int):
        return f"type: 2, integer_value: {value}"
    elif isinstance(value, float):
        return f"type: 3, double_value: {value}"
    else:
        return f"type: 4, string_value: '{value}'"


def _set_goal_checker_tolerances(tolerance_xy: float, tolerance_yaw: float,
                                  quiet: bool = False) -> bool:
    """Program StoppedGoalChecker tolerances on controller_server.

    Sets xy_goal_tolerance and yaw_goal_tolerance via SetParameters service.
    """
    params = [
        ('general_goal_checker.xy_goal_tolerance', tolerance_xy),
        ('general_goal_checker.yaw_goal_tolerance', tolerance_yaw),
    ]
    param_entries = []
    for param_name, value in params:
        val_field = _format_param_value(value)
        param_entries.append(
            f"{{name: '{param_name}', value: {{{val_field}}}}}")
    params_yaml = "{parameters: [" + ", ".join(param_entries) + "]}"

    ros_cmd = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"ros2 service call /controller_server/set_parameters "
        f"rcl_interfaces/srv/SetParameters \"{params_yaml}\""
    )
    try:
        result = _docker_exec(ros_cmd, timeout=30)
        if result.returncode != 0:
            print(f"{YELLOW}  WARN: Failed to set goal checker tolerances{NC}",
                  file=sys.stderr)
            return False
        if 'successful=False' in result.stdout:
            print(f"{YELLOW}  WARN: Goal checker tolerances rejected: "
                  f"{result.stdout.strip()[:200]}{NC}", file=sys.stderr)
            return False
        if not quiet:
            print(f"{CYAN}  Goal checker tolerances: xy={tolerance_xy}, "
                  f"yaw={tolerance_yaw}{NC}", file=sys.stderr)
        return True
    except subprocess.TimeoutExpired:
        print(f"{YELLOW}  WARN: Goal checker tolerance set timed out{NC}",
              file=sys.stderr)
        return False


def run_nav2_test(waypoints_file='gym_loop_nav2',
                  timeout=DEFAULT_TEST_TIMEOUT_SECONDS, quiet=False,
                  record=False, cycle_num=None, skip_init=False, use_amcl=True,
                  trust_nav2=True) -> Tuple[bool, dict]:
    """
    Run Nav2 waypoint test using container-internal runner.

    Programs StoppedGoalChecker tolerances from waypoint YAML before running.

    Args:
        waypoints_file: Waypoints YAML file to execute
        timeout: Test timeout in seconds
        quiet: Suppress progress output
        record: Enable rosbag2 recording
        cycle_num: Cycle number for rosbag naming
        skip_init: Skip initial pose setting (for manual stack restart testing)
        use_amcl: Use AMCL localization (True) or odometry-only (False)
        trust_nav2: Trust Nav2 STATUS_SUCCEEDED for goal detection (default: True)

    Returns:
        (success, result_dict)
    """
    # Program StoppedGoalChecker with per-test tolerances from waypoint YAML
    if trust_nav2:
        tolerances = _load_waypoint_tolerances(waypoints_file)
        if tolerances:
            _set_goal_checker_tolerances(tolerances[0], tolerances[1], quiet=quiet)

    runner_args = ['--waypoints', waypoints_file, '--timeout', str(timeout), '--json']
    if quiet:
        runner_args.append('--quiet')
    if record:
        runner_args.append('--record')
    if cycle_num is not None:
        runner_args.extend(['--cycle-num', str(cycle_num)])
    if skip_init:
        runner_args.append('--skip-init')
    if not use_amcl:
        runner_args.append('--no-amcl')
    if trust_nav2:
        runner_args.append('--trust-nav2')

    runner_cmd = ' '.join(['python3', '/workspace/ros/nav2_test_runner.py'] + runner_args)
    inner_cmd = f'source /opt/ros/jazzy/setup.bash && {runner_cmd}'

    try:
        result = _docker_exec(inner_cmd,
                              timeout=timeout + SUBPROCESS_TIMEOUT_BUFFER_SECONDS)

        # Print stderr (debug output) if present
        if result.stderr:
            print(result.stderr, file=sys.stderr)

        # Parse JSON output (skip RTPS/DDS C++ error lines that leak to stdout)
        stdout = result.stdout
        try:
            json_start = stdout.index('{')
            data = json.loads(stdout[json_start:])
            return data.get('success', False), data
        except (json.JSONDecodeError, ValueError):
            return False, {'success': False, 'message': 'Failed to parse test output'}

    except subprocess.TimeoutExpired:
        return False, {'success': False, 'message': 'Test command timed out'}
    except Exception as e:
        return False, {'success': False, 'message': str(e)}


def run_cycles(args) -> dict:
    """
    Run multiple test cycles with reset between each.

    Returns:
        Statistics dict
    """
    # Detect stack
    stack = detect_stack()
    if not stack:
        print(f"{RED}ERROR: No robot stack detected{NC}", file=sys.stderr)
        print(f"{YELLOW}Start a stack first: python stack.py start jetacker{NC}")
        sys.exit(1)

    # Print header
    if not args.json:
        print(f"\n{BLUE}{'='*60}{NC}")
        print(f"{BLUE}  Nav2 Waypoint Test - Multi-Cycle Runner{NC}")
        print(f"{BLUE}{'='*60}{NC}")
        print(f"{CYAN}  Stack: {stack}{NC}")
        print(f"{CYAN}  Waypoints: {args.waypoints}{NC}")
        print(f"{CYAN}  Cycles: {args.cycles}{NC}")
        print(f"{CYAN}  Timeout: {args.timeout}s per test{NC}")
        print(f"{CYAN}  Reset: {'warm (~5s)' if args.warm_reset else 'full (~50s)'}{NC}")
        if args.stop_on_error:
            print(f"{CYAN}  Stop on error: enabled{NC}")
        print(f"{BLUE}{'='*60}{NC}\n")

    # Track results
    results = []
    start_time = time.time()

    # Run cycles
    for cycle in range(1, args.cycles + 1):
        cycle_start = time.time()

        if not args.json:
            print(f"{BLUE}{'='*60}{NC}")
            print(f"{BLUE}  Cycle {cycle}/{args.cycles}{NC}")
            print(f"{BLUE}{'='*60}{NC}")

        # Step 1: Reset simulation (including Nav2 stack)
        if not args.skip_init:
            if not args.json:
                reset_type = "warm" if args.warm_reset else "full"
                print(f"\n{YELLOW}[1/2] Resetting simulation ({reset_type})...{NC}")

            if args.warm_reset:
                reset_success, reset_data = run_warm_reset(quiet=args.json or args.quiet, cycle_num=cycle)
            else:
                reset_success, reset_data = run_nav2_sim_reset(stack, quiet=args.json or args.quiet, use_amcl=not args.no_amcl)

            if not args.json and not args.quiet:
                if reset_success:
                    print(f"{GREEN}  [OK] Reset complete ({reset_data.get('duration', 0):.1f}s){NC}")
                else:
                    print(f"{RED}  [FAIL] Reset failed: {reset_data.get('message', 'unknown')}{NC}")
        else:
            if not args.json:
                print(f"\n{YELLOW}[SKIP] Skipping reset (--skip-init){NC}")
            reset_success = True
            reset_data = {'success': True, 'message': 'Skipped'}

        # Step 2: Run Nav2 test
        if not args.json:
            step_num = "1/1" if args.skip_init else "2/2"
            print(f"\n{YELLOW}[{step_num}] Running Nav2 waypoint test...{NC}")

        test_success, test_data = run_nav2_test(
            waypoints_file=args.waypoints,
            timeout=args.timeout,
            quiet=args.json or args.quiet,
            record=args.record,
            cycle_num=cycle,
            skip_init=args.skip_init,
            use_amcl=not args.no_amcl,
            trust_nav2=not args.no_trust_nav2
        )

        if not args.json and not args.quiet:
            if test_success:
                waypoints_completed = test_data.get('waypoints_completed', 0)
                waypoints_total = test_data.get('waypoints_total', 0)
                duration = test_data.get('duration', 0)
                print(f"{GREEN}  [OK] Test passed{NC}")
                print(f"    Waypoints: {waypoints_completed}/{waypoints_total}")
                print(f"    Duration: {duration:.1f}s")
                # Display episode metrics if present
                metrics = test_data.get('metrics')
                if metrics:
                    steer = metrics.get('steering')
                    cmdvel = metrics.get('cmd_vel')
                    track = metrics.get('tracking')
                    if steer:
                        steer_extra = ''
                        if 'cmd_rms_rate' in steer:
                            steer_extra = f", cmd_RMS={steer['cmd_rms_rate']:.3f}, cmd_flip={steer['cmd_flip_rate_hz']:.1f} Hz"
                        print(f"    Steer: RMS_rate={steer['rms_rate']:.3f} rad/s, flip={steer['flip_rate_hz']:.1f} Hz{steer_extra}")
                    if cmdvel:
                        print(f"    CmdVel: TV(v)={cmdvel['tv_linear']:.6f}, TV(w)={cmdvel['tv_angular']:.6f}")
                    if track:
                        print(f"    XTrack: RMS={track['rms_xtrack_m']*1000:.1f}mm, max={track['max_xtrack_m']*1000:.1f}mm")
                        print(f"    Heading: RMS_dev={math.degrees(track['rms_heading_dev_rad']):.2f} deg")
                    wheel = metrics.get('wheel_vel')
                    if wheel:
                        print(f"    WheelVel: TV(L)={wheel['tv_left']:.6f}, TV(R)={wheel['tv_right']:.6f}, TV(avg)={wheel['tv_combined']:.6f}")
            else:
                failure_reason = test_data.get('failure_reason', test_data.get('message', 'unknown'))
                print(f"{RED}  [FAIL] Test failed: {failure_reason}{NC}")

        # Record cycle result
        cycle_duration = time.time() - cycle_start
        cycle_result = {
            'cycle': cycle,
            'reset_success': reset_success,
            'test_success': test_success,
            'duration': cycle_duration,
            'test_data': test_data,
            'reset_data': reset_data
        }
        results.append(cycle_result)

        # Print cycle summary
        if not args.json:
            overall = reset_success and test_success
            status = f"{GREEN}PASS{NC}" if overall else f"{RED}FAIL{NC}"
            print(f"\n[{status}] - Cycle {cycle} completed in {cycle_duration:.1f}s\n")

        # Check stop-on-error
        if args.stop_on_error and not test_success:
            if not args.json:
                print(f"{YELLOW}Stopping due to test failure (--stop-on-error){NC}\n")
            break

    # Calculate statistics
    total_duration = time.time() - start_time
    total_cycles = len(results)
    passed = sum(1 for r in results if r['reset_success'] and r['test_success'])
    failed = total_cycles - passed
    success_rate = (passed / total_cycles * 100) if total_cycles > 0 else 0

    stats = {
        'stack': stack,
        'waypoints': args.waypoints,
        'total_cycles': total_cycles,
        'passed': passed,
        'failed': failed,
        'success_rate': success_rate,
        'total_duration': total_duration,
        'avg_cycle_duration': total_duration / total_cycles if total_cycles > 0 else 0,
        'results': results
    }

    return stats


def print_summary(stats: dict):
    """Print human-readable summary"""
    print(f"\n{BLUE}{'='*60}{NC}")
    print(f"{BLUE}  Nav2 Waypoint Test Summary{NC}")
    print(f"{BLUE}{'='*60}{NC}")
    print(f"  Stack: {stats['stack']}")
    print(f"  Waypoints: {stats['waypoints']}")
    print(f"  Total Cycles: {stats['total_cycles']}")
    print(f"  Passed: {GREEN}{stats['passed']}{NC}")
    print(f"  Failed: {RED}{stats['failed']}{NC}")
    print(f"  Success Rate: {stats['success_rate']:.1f}%")
    print(f"  Total Duration: {stats['total_duration']:.1f}s")
    print(f"  Avg Cycle Duration: {stats['avg_cycle_duration']:.1f}s")
    print(f"{BLUE}{'='*60}{NC}\n")

    # Overall result
    if stats['failed'] == 0:
        print(f"{GREEN}{'='*60}{NC}")
        print(f"{GREEN}  ALL TESTS PASSED{NC}")
        print(f"{GREEN}{'='*60}{NC}\n")
    else:
        print(f"{RED}{'='*60}{NC}")
        print(f"{RED}  SOME TESTS FAILED{NC}")
        print(f"{RED}{'='*60}{NC}\n")


def main():
    parser = argparse.ArgumentParser(
        description='Run Nav2 waypoint tests with multi-cycle support (YAML-driven)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Single test
  python drive_nav2.py --waypoints gym_loop_nav2 --cycles 1

  # 10 cycles with reset
  python drive_nav2.py --waypoints gym_loop_nav2 --cycles 10

  # JSON output for automation
  python drive_nav2.py --cycles 5 --json

  # Stop on first failure
  python drive_nav2.py --cycles 10 --stop-on-error

  # With rosbag recording
  python drive_nav2.py --cycles 5 --record
        """
    )
    parser.add_argument('--waypoints', type=str, default='gym_loop_nav2',
                        help='YAML waypoints file to execute (default: gym_loop_nav2)')
    parser.add_argument('--cycles', type=int, default=1,
                        help='Number of test cycles to run (default: 1)')
    parser.add_argument('--timeout', type=float, default=DEFAULT_TEST_TIMEOUT_SECONDS,
                        help=f'Test timeout in seconds per cycle (default: {DEFAULT_TEST_TIMEOUT_SECONDS:.0f})')
    parser.add_argument('--json', action='store_true',
                        help='Output results as JSON')
    parser.add_argument('--quiet', action='store_true',
                        help='Suppress progress messages')
    parser.add_argument('--stop-on-error', action='store_true',
                        help='Stop testing after first failure')
    parser.add_argument('--record', action='store_true',
                        help='Record rosbag2 during each test')
    parser.add_argument('--no-amcl', action='store_true',
                        help='Use odometry-only (no AMCL localization) - bypasses AMCL with static map->odom transform')
    parser.add_argument('--warm-reset', action='store_true',
                        help='Use fast teleport reset (~5s) instead of full orchestrator (~50s)')
    parser.add_argument('--skip-init', action='store_true',
                        help='Skip initial pose and simulation reset (for manual stack restart testing)')
    parser.add_argument('--no-trust-nav2', action='store_true',
                        help='Use custom tolerance check instead of Nav2 STATUS_SUCCEEDED for goal detection')
    args = parser.parse_args()

    try:
        stats = run_cycles(args)

        if args.json:
            print(json.dumps(stats, indent=2))
        else:
            print_summary(stats)

        # Exit code: 0 if all passed, 1 if any failed
        return 0 if stats['failed'] == 0 else 1

    except KeyboardInterrupt:
        if not args.json:
            print(f"\n{YELLOW}Interrupted by user{NC}")
        return 130
    except Exception as e:
        if args.json:
            print(json.dumps({'success': False, 'error': str(e)}))
        else:
            print(f"{RED}ERROR: {e}{NC}", file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
