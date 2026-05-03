#!/usr/bin/env python3
"""
Cross-validate Nav2 MPPI params across all matrix tests (M1–M8).

Runs each test sequentially with warm reset, prints pass/fail summary table.
Params sourced from Optuna trial, CLI overrides, or both.

Usage:
    python run_matrix_suite.py --study mppi_pareto_pfc_pac_m8_v5 --trial 8 --no-amcl
    python run_matrix_suite.py --set PathFollowCritic.cost_weight=47.6 --tests M1,M4 --no-amcl
    python run_matrix_suite.py --study my_study --trial 14 --tests M1,M8 --reps 3 --no-amcl
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

from tune_common import (
    ALL_TESTS,
    check_stack_health,
    compute_jitter_score,
    extract_cycle_data,
    load_dotenv,
    load_tuning_params,
    resolve_storage,
    run_test,
    set_all_params,
    set_locked_params,
)

DEFAULT_PARAMS_FILE = Path(__file__).parent / 'config' / 'tuning_params_pareto_v13.yaml'


def load_trial_params(study_name: str, trial_number: int,
                      storage: str = None) -> dict:
    """Load param values from a completed Optuna trial."""
    import optuna
    load_dotenv()
    resolved = resolve_storage(storage, study_name)
    study = optuna.load_study(study_name=study_name, storage=resolved)
    trial = study.trials[trial_number]
    return dict(trial.params)


def parse_set_args(set_args: list[str]) -> dict:
    """Parse --set Key=Value arguments into a dict."""
    out = {}
    for arg in (set_args or []):
        if '=' not in arg:
            print(f"ERROR: --set requires Key=Value format, got: {arg}",
                  file=sys.stderr)
            sys.exit(1)
        key, _, val = arg.partition('=')
        # Auto-detect type
        try:
            val = int(val)
        except ValueError:
            try:
                val = float(val)
            except ValueError:
                pass  # keep as string
        out[key.strip()] = val
    return out


def main():
    parser = argparse.ArgumentParser(
        description='Cross-validate MPPI params across Nav2 matrix tests',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"Available tests: {', '.join(ALL_TESTS.keys())}")

    # Param sources
    parser.add_argument('--study', type=str, default=None,
                        help='Optuna study name')
    parser.add_argument('--trial', type=int, default=None,
                        help='Optuna trial number (requires --study)')
    parser.add_argument('--storage', type=str, default=None,
                        help='Optuna storage URL (default: $OPTUNA_STORAGE)')
    parser.add_argument('--set', dest='set_args', action='append', default=None,
                        help='Manual param override Key=Value (repeatable)')
    parser.add_argument('--params-file', type=str, default=None,
                        help=f'Tuning params YAML (default: {DEFAULT_PARAMS_FILE.name})')

    # Test selection
    parser.add_argument('--tests', type=str, default=None,
                        help='Comma-separated test IDs (default: all M1–M8)')
    parser.add_argument('--reps', type=int, default=1,
                        help='Reps per test (default: 1)')

    # Flags
    parser.add_argument('--no-amcl', action='store_true',
                        help='Odometry only (no AMCL)')
    parser.add_argument('--no-trust-nav2', action='store_true',
                        help='Use custom goal checker instead of Nav2 status')
    parser.add_argument('--skip-locked', action='store_true',
                        help='Skip setting locked params from YAML')
    parser.add_argument('--json', dest='json_output', action='store_true',
                        help='Machine-readable JSON output')

    args = parser.parse_args()

    # Validate
    if args.trial is not None and not args.study:
        print("ERROR: --trial requires --study", file=sys.stderr)
        return 1
    if args.study and args.trial is None:
        print("ERROR: --study requires --trial", file=sys.stderr)
        return 1
    if not args.study and not args.set_args:
        print("ERROR: Need --study/--trial or --set to specify params",
              file=sys.stderr)
        return 1

    # Resolve tests
    if args.tests:
        test_ids = [t.strip().upper() for t in args.tests.split(',')]
        for t in test_ids:
            if t not in ALL_TESTS:
                print(f"ERROR: Unknown test '{t}'. "
                      f"Available: {','.join(ALL_TESTS.keys())}",
                      file=sys.stderr)
                return 1
    else:
        test_ids = list(ALL_TESTS.keys())

    # Load tuning params YAML (for ros_paths and locked values)
    params_path = Path(args.params_file) if args.params_file else DEFAULT_PARAMS_FILE
    if not params_path.exists():
        print(f"ERROR: Params file not found: {params_path}", file=sys.stderr)
        return 1
    all_params = load_tuning_params(max_tier=99, path=params_path)

    # Build param values: Optuna trial → --set overrides
    trial_values = {}
    source_label = ''

    if args.study:
        trial_values = load_trial_params(args.study, args.trial, args.storage)
        source_label = f'T{args.trial} from {args.study}'

    if args.set_args:
        overrides = parse_set_args(args.set_args)
        trial_values.update(overrides)
        if source_label:
            source_label += f' + {len(overrides)} override(s)'
        else:
            source_label = f'{len(overrides)} manual param(s)'

    # Check stack health
    if not check_stack_health():
        print("ERROR: Stack is not healthy. Run: python stack.py status",
              file=sys.stderr)
        return 1

    # Set locked params from YAML
    if not args.skip_locked:
        if not set_locked_params(all_params):
            print("ERROR: Failed to set locked params", file=sys.stderr)
            return 1

    # Set trial params (only those present in all_params)
    params_to_set = {k: v for k, v in trial_values.items() if k in all_params}
    if params_to_set:
        if not set_all_params(params_to_set, all_params):
            print("ERROR: Failed to set trial params", file=sys.stderr)
            return 1

    # Header
    n_tests = len(test_ids)
    print(f"\n{'='*66}", file=sys.stderr)
    print(f"  Nav2 Matrix Suite — {n_tests} tests × {args.reps} rep",
          file=sys.stderr)
    print(f"  Source: {source_label}", file=sys.stderr)
    print(f"{'='*66}", file=sys.stderr)
    for k, v in sorted(trial_values.items()):
        if isinstance(v, float):
            print(f"  {k} = {v:.4g}", file=sys.stderr)
        else:
            print(f"  {k} = {v}", file=sys.stderr)
    print(f"{'='*66}", file=sys.stderr)

    # Run tests
    use_amcl = not args.no_amcl
    trust_nav2 = not args.no_trust_nav2
    results = []  # list of dicts per test

    for test_id in test_ids:
        waypoint_file, timeout, _weight = ALL_TESTS[test_id]
        print(f"\n[{test_id}] {waypoint_file} (timeout={timeout}s)",
              file=sys.stderr)

        test_result = {
            'test_id': test_id,
            'waypoint_file': waypoint_file,
            'reps': [],
            'pass_count': 0,
            'total_time': 0.0,
        }

        for rep in range(args.reps):
            t0 = time.time()
            data = run_test(waypoint_file, timeout, use_amcl=use_amcl,
                            trust_nav2=trust_nav2)
            wall_time = time.time() - t0

            success, collision, duration, metrics, test_data = \
                extract_cycle_data(data, fallback_duration=wall_time)

            # Extract errors
            wp_results = test_data.get('waypoint_results', [])
            error_xy = None
            error_yaw = None
            if wp_results:
                last_wp = wp_results[-1]
                error_xy = last_wp.get('error_xy')
                error_yaw = last_wp.get('error_yaw')

            cmdvel = metrics.get('cmd_vel', {})
            rev_count = cmdvel.get('reversal_count', 0)

            passed = success and not collision
            if passed:
                test_result['pass_count'] += 1
            test_result['total_time'] += wall_time

            rep_data = {
                'passed': passed,
                'collision': collision,
                'duration': duration,
                'wall_time': wall_time,
                'error_xy': error_xy,
                'error_yaw': error_yaw,
                'reversals': rev_count,
            }
            test_result['reps'].append(rep_data)

            # Print rep line
            status = 'PASS' if passed else ('COLL' if collision else 'FAIL')
            yaw_str = f'{error_yaw:.3f}' if error_yaw is not None else '?'
            xy_mm = f'{error_xy * 1000:.0f}mm' if error_xy is not None else '?'
            print(f"  rep {rep+1}: [{status}] {duration:.1f}s "
                  f"yaw={yaw_str} xy={xy_mm} rev={rev_count}",
                  file=sys.stderr)

        print(f"  → {test_result['pass_count']}/{args.reps} in "
              f"{test_result['total_time']:.1f}s", file=sys.stderr)
        results.append(test_result)

    # Summary
    total_pass = sum(r['pass_count'] for r in results)
    total_reps = len(results) * args.reps
    total_time = sum(r['total_time'] for r in results)

    print(f"\n{'='*66}", file=sys.stderr)
    print(f"  Summary", file=sys.stderr)
    print(f"{'='*66}", file=sys.stderr)
    print(f"  {'Test':<6} {'Pass':>6} {'Time':>8} {'Yaw':>8} "
          f"{'XY':>8} {'Rev':>4}", file=sys.stderr)
    print(f"  {'─'*42}", file=sys.stderr)

    for r in results:
        tid = r['test_id']
        pc = r['pass_count']
        tt = r['total_time']

        # Average metrics from passing reps (or all if none passed)
        scored = [rep for rep in r['reps'] if rep['passed']] or r['reps']
        avg_yaw = _avg([rep['error_yaw'] for rep in scored
                        if rep['error_yaw'] is not None])
        avg_xy = _avg([rep['error_xy'] for rep in scored
                       if rep['error_xy'] is not None])
        avg_rev = _avg([rep['reversals'] for rep in scored])

        yaw_str = f'{avg_yaw:.3f}' if avg_yaw is not None else '?'
        xy_str = f'{avg_xy * 1000:.0f}mm' if avg_xy is not None else '?'
        rev_str = f'{avg_rev:.0f}' if avg_rev is not None else '?'

        print(f"  {tid:<6} {pc}/{args.reps:>3} {tt:>7.1f}s {yaw_str:>8} "
              f"{xy_str:>8} {rev_str:>4}", file=sys.stderr)

    print(f"  {'─'*42}", file=sys.stderr)
    print(f"  TOTAL  {total_pass}/{total_reps:>3} {total_time:>7.1f}s",
          file=sys.stderr)
    print(f"{'='*66}", file=sys.stderr)

    # JSON output
    if args.json_output:
        print(json.dumps({
            'source': source_label,
            'params': trial_values,
            'tests': [{
                'test_id': r['test_id'],
                'pass_count': r['pass_count'],
                'reps': args.reps,
                'total_time': round(r['total_time'], 1),
                'rep_details': r['reps'],
            } for r in results],
            'total_pass': total_pass,
            'total_reps': total_reps,
            'total_time': round(total_time, 1),
        }, indent=2))

    return 0 if total_pass == total_reps else 1


def _avg(vals: list) -> float | None:
    """Average of non-None values, or None if empty."""
    filtered = [v for v in vals if v is not None]
    return sum(filtered) / len(filtered) if filtered else None


if __name__ == '__main__':
    sys.exit(main())
