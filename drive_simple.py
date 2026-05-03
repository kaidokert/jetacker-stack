#!/usr/bin/env python3
"""
Simple Drive Test Runner (YAML-driven) — same CLI and behavior as
test_drive_simple.py but derives all stack topology from stack.py / stacks.yaml
instead of hardcoded node lists.

Reset modes:
  --warm-reset: Fast teleport reset (~5s) — pause, teleport, zero joints, unpause
  (default):    Full orchestrator reset (~50s) — stop, gazebo reset, restart, gates

Usage:
    # Single test
    python drive_simple.py

    # 10 cycles with warm reset
    python drive_simple.py --cycles 10 --warm-reset

    # JSON output
    python drive_simple.py --cycles 5 --json

    # Stop on first failure
    python drive_simple.py --cycles 10 --stop-on-error
"""

import subprocess
import sys
import argparse
import json
import time
import logging
import math
from pathlib import Path
from typing import Optional, Tuple
from logging.handlers import RotatingFileHandler


# Import shared functions
sys.path.insert(0, str(Path(__file__).parent / 'tools'))
from drive_utils import (
    detect_stack,
    verify_controller_loaded,
    run_robot_reset,
    RED, GREEN, YELLOW, BLUE, CYAN, NC,
)

# Import new declarative reset orchestrator
from reset_orchestrator import ResetOrchestrator

# Import YAML-driven stack topology
from stack import load_manifest, build_reset_definition, get_reset_conflicts, soft_reset


# ============================================================================
# Event logging (shared with container-side drive_test_runner.py)
# ============================================================================
event_logger = logging.getLogger('drive_test_events')
event_logger.setLevel(logging.DEBUG)

# Use same log file as container (mounted volume at ./logs/events)
log_file = Path('./logs/events/drive_test_events.log')
log_file.parent.mkdir(parents=True, exist_ok=True)

file_handler = RotatingFileHandler(
    str(log_file),
    maxBytes=10*1024*1024,
    backupCount=5
)

formatter = logging.Formatter(
    '%(asctime)s | %(levelname)-8s | cycle=%(cycle)s | event=%(event)s | %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
file_handler.setFormatter(formatter)
event_logger.addHandler(file_handler)

# ============================================================================
# Timeout constants (seconds)
# ============================================================================
DEFAULT_TEST_TIMEOUT_SECONDS = 120.0
SUBPROCESS_TIMEOUT_BUFFER_SECONDS = 10.0


def run_sim_reset(stack='jetacker', quiet=False, cycle_num=0):
    """
    Reset simulation using declarative orchestrator.

    Uses ResetOrchestrator with the base stack definition from stacks.yaml.
    4-phase reset: stop all → gazebo reset → restart bridges → start with gates.

    Args:
        stack: Stack name ('jetacker' or 'slam_bot')
        quiet: Suppress progress output
        cycle_num: Current cycle number for logging

    Returns:
        (success, result_dict)
    """
    if stack not in ('jetacker', 'slam_bot'):
        return False, {'success': False, 'message': f'Unsupported stack: {stack}'}

    try:
        manifest = load_manifest()
        stack_definition = build_reset_definition(manifest, stack, 'base')
        conflicts = get_reset_conflicts(manifest, stack, 'base')

        orchestrator = ResetOrchestrator(
            stack_definition,
            stack_name='base',
            conflicting_services=conflicts,
        )

        start_time = time.time()
        success = orchestrator.reset_stack_full()
        duration = time.time() - start_time

        if not success:
            return False, {
                'success': False,
                'message': 'Orchestrator reset failed (check logs)',
                'duration': duration
            }

        # Post-reset: verify controller is active
        extra = {'cycle': cycle_num, 'event': 'controller_verification'}
        event_logger.info(f'Verifying controller: {stack}/tricycle_steering_controller', extra=extra)

        ctrl_ok, ctrl_info = verify_controller_loaded(stack, 'tricycle_steering_controller')
        if not ctrl_ok:
            event_logger.error(f'Controller verification FAILED: {ctrl_info}', extra=extra)
            return False, {
                'success': False,
                'message': f'Controller verification failed: {ctrl_info}',
                'duration': duration
            }

        event_logger.info(f'Controller verified: {ctrl_info}', extra=extra)
        if not quiet:
            print(f"{GREEN}      Controller active: {ctrl_info['controller']}{NC}")

        return True, {
            'success': True,
            'message': 'Reset complete',
            'duration': duration,
            'controller': ctrl_info
        }

    except Exception as e:
        return False, {
            'success': False,
            'message': f'Reset orchestrator exception: {str(e)}'
        }


def run_warm_reset(quiet=False, cycle_num=0):
    """Reset simulation using warm teleport reset (stack.py soft-reset).

    Much faster than full orchestrator reset (~5s vs ~50s).
    Teleports model to origin, zeros joints, resets EKF.

    Returns:
        (success, result_dict)
    """
    start_time = time.time()
    try:
        # soft_reset() calls sys.exit(1) on failure
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


def run_drive_test(instruction='calibrate_1m',
                   timeout=DEFAULT_TEST_TIMEOUT_SECONDS, quiet=False,
                   record=False, cycle_num=None,
                   fast_preflight=False) -> Tuple[bool, dict]:
    """
    Run drive test using container-internal runner.

    Args:
        instruction: Instruction YAML file to execute
        timeout: Test timeout in seconds
        quiet: Suppress progress output
        record: Enable rosbag2 recording
        cycle_num: Cycle number for rosbag naming
        fast_preflight: Shorten TF diagnostics (warm-reset cycles)

    Returns:
        (success, result_dict)
    """
    runner_args = ['--instruction', instruction, '--timeout', str(timeout), '--json']
    if quiet:
        runner_args.append('--quiet')
    if record:
        runner_args.append('--record')
    if fast_preflight:
        runner_args.append('--fast-preflight')
    if cycle_num is not None:
        runner_args.extend(['--cycle-num', str(cycle_num)])

    runner_cmd = ' '.join(['python3', '/workspace/ros/drive_test_runner.py'] + runner_args)
    cmd = [
        'docker', 'compose', 'exec', '-T', 'test-drive',
        'bash', '-c',
        f'source /opt/ros/jazzy/setup.bash && {runner_cmd}'
    ]

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=False,
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
        print(f"{BLUE}  Drive Test - Multi-Cycle Runner{NC}")
        print(f"{BLUE}{'='*60}{NC}")
        print(f"{CYAN}  Stack: {stack}{NC}")
        print(f"{CYAN}  Cycles: {args.cycles}{NC}")
        print(f"{CYAN}  Timeout: {args.timeout}s per test{NC}")
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

        # Step 1: Reset robot (simulation or physical)
        if args.no_reset:
            reset_success, reset_data = True, {'success': True, 'message': 'Reset skipped (--no-reset)'}
            if not args.json:
                print(f"\n{YELLOW}[1/2] Reset skipped (--no-reset){NC}")
        else:
            if not args.json:
                if args.warm_reset:
                    reset_type = "warm"
                elif stack == 'jetacker_real':
                    reset_type = "robot"
                else:
                    reset_type = "simulation"
                print(f"\n{YELLOW}[1/2] Resetting ({reset_type})...{NC}")

            # Dispatch to correct reset function
            if stack == 'jetacker_real':
                reset_success, reset_data = run_robot_reset(stack, quiet=args.json or args.quiet, cycle_num=cycle)
            elif args.warm_reset:
                reset_success, reset_data = run_warm_reset(quiet=args.json or args.quiet, cycle_num=cycle)
            else:
                reset_success, reset_data = run_sim_reset(stack, quiet=args.json or args.quiet, cycle_num=cycle)

        if not args.json and not args.quiet:
            if reset_success:
                print(f"{GREEN}  [OK] Reset complete ({reset_data.get('duration', 0):.1f}s){NC}")
            else:
                print(f"{RED}  [FAIL] Reset failed: {reset_data.get('message', 'unknown')}{NC}")

        # Step 2: Run test
        if not args.json:
            print(f"\n{YELLOW}[2/2] Running drive test...{NC}")

        test_success, test_data = run_drive_test(
            instruction=args.instruction if args.instruction else 'calibrate_1m',
            timeout=args.timeout,
            quiet=args.json or args.quiet,
            record=args.record,
            cycle_num=cycle,
            fast_preflight=args.warm_reset,
        )

        if not args.json and not args.quiet:
            if test_success:
                result = test_data.get('result', {})
                checkpoints = result.get('checkpoints', []) if result else []
                print(f"{GREEN}  [OK] Test passed{NC}")
                for cp in checkpoints:
                    if cp['type'] == 'drive':
                        print(f"    Drive: {cp['distance']:.3f}m (error={cp['error']:.4f}m) heading={cp['heading']:.1f}")
                    elif cp['type'] == 'rotate':
                        print(f"    Rotate: {cp['rotation']:.1f} (error={cp['error']:.2f}) heading={cp['heading']:.1f}")
                # Display episode metrics if present
                metrics = result.get('metrics') if result else None
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
                print(f"{RED}  [FAIL] Test failed: {test_data.get('message', 'unknown')}{NC}")

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
    print(f"{BLUE}  Test Summary{NC}")
    print(f"{BLUE}{'='*60}{NC}")
    print(f"  Stack: {stats['stack']}")
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
        description='Run drive tests with multi-cycle support (YAML-driven)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Single test
  python drive_simple.py

  # 10 cycles with reset
  python drive_simple.py --cycles 10

  # JSON output for automation
  python drive_simple.py --cycles 5 --json

  # Stop on first failure
  python drive_simple.py --cycles 10 --stop-on-error
        """
    )
    parser.add_argument('--instruction', type=str, default=None,
                        help='YAML instruction file to load (e.g., calibrate_1m)')
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
    parser.add_argument('--no-reset', action='store_true',
                        help='Skip reset step (useful for single hw runs)')
    parser.add_argument('--warm-reset', action='store_true',
                        help='Use fast teleport reset (~5s) instead of full orchestrator (~50s)')
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
