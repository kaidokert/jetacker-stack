#!/usr/bin/env python3
"""
Nav2 Test Runner - Container-internal test runner for Nav2 waypoint navigation

Wraps nav2_waypoint_follower.py with:
- Argument parsing
- Rosbag2 recording
- JSON output
- Quiet mode
- Cycle numbering

Mirrors drive_test_runner.py pattern for consistency.

Usage:
    python3 nav2_test_runner.py --waypoints gym_loop_nav2 --json
    python3 nav2_test_runner.py --waypoints gym_loop_nav2 --record --cycle-num 1
"""

import argparse
import json
import sys
import yaml
from pathlib import Path

# Import waypoint follower
from nav2_waypoint_follower import execute_waypoint_test


def load_waypoints(waypoints_file):
    """
    Load waypoints (and optional obstacles) from YAML file.

    Args:
        waypoints_file: Filename (without .yaml extension) or full path

    Returns:
        Tuple (waypoints, obstacles) where obstacles is a list of dicts or empty.
    """
    # Try as filename first
    if not waypoints_file.endswith('.yaml'):
        yaml_path = Path('driving_instructions') / f'{waypoints_file}.yaml'
    else:
        yaml_path = Path(waypoints_file)

    if not yaml_path.exists():
        raise FileNotFoundError(f'Waypoints file not found: {yaml_path}')

    with open(yaml_path) as f:
        data = yaml.safe_load(f)

    waypoints = data.get('waypoints', [])
    obstacles = data.get('obstacles', []) or []

    if not waypoints:
        raise ValueError(f'No waypoints found in {yaml_path}')

    return waypoints, obstacles


def main():
    parser = argparse.ArgumentParser(
        description='Nav2 Test Runner - Container-internal test execution',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run test with JSON output
  python3 nav2_test_runner.py --waypoints gym_loop_nav2 --json

  # Run test with rosbag recording
  python3 nav2_test_runner.py --waypoints gym_loop_nav2 --record --cycle-num 1

  # Run test quietly
  python3 nav2_test_runner.py --waypoints gym_loop_nav2 --quiet --json
        """
    )

    parser.add_argument('--waypoints', type=str, required=True,
                       help='Waypoints YAML file (e.g., gym_loop_nav2)')
    parser.add_argument('--timeout', type=float, default=120.0,
                       help='Overall test timeout in seconds (default: 120.0)')
    parser.add_argument('--json', action='store_true',
                       help='Output results as JSON')
    parser.add_argument('--quiet', action='store_true',
                       help='Suppress info logging')
    parser.add_argument('--record', action='store_true',
                       help='Record rosbag2 during test')
    parser.add_argument('--cycle-num', type=int, default=None,
                       help='Cycle number for rosbag naming')
    parser.add_argument('--skip-init', action='store_true',
                       help='Skip initial pose setting (for manual stack restart testing)')
    parser.add_argument('--use-amcl', action='store_true', default=True,
                       help='Use AMCL localization (default: True)')
    parser.add_argument('--no-amcl', dest='use_amcl', action='store_false',
                       help='Use odometry-only mode (no AMCL localization)')
    parser.add_argument('--trust-nav2', action='store_true',
                       help='Trust Nav2 STATUS_SUCCEEDED for goal detection')

    args = parser.parse_args()

    try:
        # Load waypoints and any obstacle drops
        waypoints, obstacles = load_waypoints(args.waypoints)

        if not args.json and not args.quiet:
            print(f'Loaded {len(waypoints)} waypoints from {args.waypoints}')
            if obstacles:
                print(f'Obstacles: {len(obstacles)} scheduled drop(s)')
            print(f'Timeout: {args.timeout}s')
            print()

        # Execute test (recording handled in-process by Nav2WaypointFollower)
        results = execute_waypoint_test(
            waypoints,
            timeout=args.timeout,
            quiet=args.quiet,
            skip_init=args.skip_init,
            use_amcl=args.use_amcl,
            record=args.record,
            bag_dir='/workspace/logs/rosbags',
            cycle_num=args.cycle_num,
            trust_nav2=args.trust_nav2,
            obstacles=obstacles,
        )

        # Output results
        if args.json:
            print(json.dumps(results, indent=2))
        else:
            # Human-readable output
            print()
            print('=' * 80)
            print('  Nav2 Waypoint Test Results')
            print('=' * 80)
            print(f'  Success: {results["success"]}')
            print(f'  Waypoints completed: {results["waypoints_completed"]}/{results["waypoints_total"]}')
            print(f'  Total duration: {results["duration"]:.1f}s')

            if results['failure_reason']:
                print(f'  Failure reason: {results["failure_reason"]}')

            print()
            print('  Per-Waypoint Results:')
            print('  ' + '-' * 76)

            for i, wp_result in enumerate(results['waypoint_results'], 1):
                success_str = 'OK' if wp_result['success'] else 'FAIL'
                name = wp_result['name']
                duration = wp_result['duration']

                if wp_result['success']:
                    error_xy_mm = wp_result['error_xy'] * 1000 if wp_result['error_xy'] else 0.0
                    error_yaw_deg = (wp_result['error_yaw'] * 180 / 3.14159 if wp_result['error_yaw'] else 0.0)
                    print(f'  [{success_str}] WP{i}: {name}')
                    print(f'       Duration: {duration:.1f}s, Error: {error_xy_mm:.1f}mm, {error_yaw_deg:.2f}°')
                else:
                    reason = wp_result.get('reason', 'unknown')
                    print(f'  [{success_str}] WP{i}: {name} - {reason}')

            print('=' * 80)
            print()

        # Exit code
        return 0 if results['success'] else 1

    except FileNotFoundError as e:
        if args.json:
            print(json.dumps({'success': False, 'error': str(e)}))
        else:
            print(f'ERROR: {e}', file=sys.stderr)
        return 1

    except Exception as e:
        if args.json:
            print(json.dumps({'success': False, 'error': str(e)}))
        else:
            print(f'ERROR: {e}', file=sys.stderr)
            import traceback
            traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
