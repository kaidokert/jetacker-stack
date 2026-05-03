#!/usr/bin/env python3
"""
Optuna MPPI Tuning Harness — "Shotgun" composite full-matrix scorer.

Runs multiple Nav2 matrix tests per trial with a weighted composite score.
TPE sampler, graduated penalties for failures, collision continues.

Architecture:
    tune_mppi.py (Optuna objective)
      ├─ tune_common.py (shared infrastructure)
      ├─ load_tuning_params() from config/tuning_params.yaml
      ├─ set_all_params() — batched ros2 param set via docker exec
      ├─ drive_nav2.py --waypoints <test> --warm-reset --json --cycles 1
      ├─ Compute composite score (weighted sum across tests)
      └─ Report to Optuna

Usage:
    python tune_mppi.py --trials 100 --tier 1
    python tune_mppi.py --trials 200 --tier 2 --resume
    python tune_mppi.py --trials 1 --baseline-only --tier 1
    python tune_mppi.py --trials 100 --tier 1 --full-reset-interval 50
    python tune_mppi.py --trials 100 --tier 1 --stack jetacker:nav2
    python tune_mppi.py --trials 50 --tier 1 --tests M1,M2

See also: tune_mppi_focused.py for single-test multi-objective "laser" tuning.
"""

import argparse
import atexit
import json
import sys
import time
from pathlib import Path

import optuna

from tune_common import (
    ALL_TESTS,
    DEFAULT_TESTS,
    abort_on_mismatch,
    acquire_tuning_lock,
    add_common_args,
    apply_params_filter,
    build_baseline,
    check_stack_health,
    DEFAULT_JITTER_WEIGHTS,
    compute_jitter_score,
    create_or_load_study,
    dump_all_params,
    extract_cycle_data,
    full_restart_stack,
    get_locked_params,
    get_samplable_params,
    handle_stack_crash,
    load_dotenv,
    load_tuning_params,
    log_trial_params,
    print_param_summary,
    record_trial_dumps,
    resolve_storage,
    restore_baseline,
    run_test,
    sample_trial_values,
    save_study_snapshot,
    set_all_params,
    set_locked_params,
    store_trial_data,
    verify_trial_params,
)

# ---------------------------------------------------------------------------
# Constants (shotgun-specific)
# ---------------------------------------------------------------------------

COLLISION_PENALTY = 8000.0  # Per-test collision penalty (>> max timeout penalty of 500)


# ---------------------------------------------------------------------------
# Scoring
# ---------------------------------------------------------------------------

def compute_test_score(metrics: dict, duration: float,
                       score_weights: dict = None,
                       jitter_weights: dict = None) -> float:
    """Score a single successful test from its metrics. Lower is better."""
    sw = score_weights or {}
    jitter = compute_jitter_score(metrics, jitter_weights)

    track = metrics.get('tracking', {})
    xtrack_mm = track.get('rms_xtrack_m', 0) * 1000

    return (sw.get('jitter', 5.0) * jitter
            + sw.get('duration', 0.1) * duration
            + sw.get('xtrack', 0.05) * xtrack_mm)


def compute_failure_penalty(error_xy: float | None, error_yaw: float | None,
                            tolerance_xy: float = 0.25,
                            tolerance_yaw: float = 0.25) -> float:
    """Graduated penalty for a failed test based on how close it got.

    Returns a value in [100, 500]:
      - 100 = just barely missed (at tolerance boundary)
      - 500 = nowhere close (3x+ tolerance away) or no pose data
    """
    if error_xy is None:
        return 500.0  # no pose data at all

    # Normalize overshoot: 1.0 = at tolerance, 3.0+ = very far
    xy_ratio = min(error_xy / tolerance_xy, 3.0) if tolerance_xy > 0 else 3.0
    yaw_ratio = min((error_yaw or 0) / tolerance_yaw, 3.0) if tolerance_yaw > 0 else 0.0

    # Weighted: distance matters more than yaw
    return 100.0 + 100.0 * xy_ratio + 33.0 * yaw_ratio


def compute_score(test_results: list,
                  score_weights: dict = None,
                  jitter_weights: dict = None) -> float:
    """Weighted composite score across all tests in a trial. Lower is better.

    Each test_result has 'success' flag. Successful tests are scored by metrics;
    failed tests get a graduated penalty based on final error.
    """
    total = 0.0
    for r in test_results:
        w = r['weight']

        if r.get('collision'):
            total += w * COLLISION_PENALTY
        elif r.get('success', True):
            total += w * compute_test_score(r['metrics'], r['duration'],
                                            score_weights, jitter_weights)
        else:
            total += w * compute_failure_penalty(
                r.get('error_xy'), r.get('error_yaw'),
                r.get('tolerance_xy', 0.25), r.get('tolerance_yaw', 0.25),
            )

    return round(total, 3)


# ---------------------------------------------------------------------------
# Optuna objective
# ---------------------------------------------------------------------------

def make_objective(all_params: dict, tests: list, baseline_only: bool = False,
                   stack_target: str = 'jetacker:nav2_odom',
                   full_reset_interval: int = 0, use_amcl: bool = False,
                   cycles: int = 1,
                   score_weights: dict = None,
                   jitter_weights: dict = None):
    """Create the Optuna objective function (closure over config)."""

    samplable = get_samplable_params(all_params)
    locked = get_locked_params(all_params)
    baseline = build_baseline(all_params)
    trial_count = 0
    locked_dirty = False  # True after stack restart (locked params need re-set)

    def objective(trial: optuna.Trial) -> float:
        nonlocal trial_count, locked_dirty
        trial_count += 1

        # Prophylactic full restart every N trials
        if (full_reset_interval > 0
                and trial_count > 1
                and (trial_count - 1) % full_reset_interval == 0):
            print(f"\n  PROPHYLACTIC RESTART (every {full_reset_interval} trials, "
                  f"count={trial_count})", file=sys.stderr)
            full_restart_stack(stack_target)
            locked_dirty = True

        # Re-set locked params after any stack restart
        if locked_dirty and locked:
            set_locked_params(all_params)
            locked_dirty = False

        # Build trial values
        trial_values = sample_trial_values(trial, all_params, baseline_only)

        # Log parameters
        log_trial_params(trial.number, trial_values, baseline, locked)

        # Set only sampled params (locked were set once at study start)
        sampled_values = {k: v for k, v in trial_values.items()
                          if k in samplable}
        if not set_all_params(sampled_values, all_params):
            handle_stack_crash(trial, trial_values, samplable,
                               stack_target)

        # Dump and verify actual ROS2 params
        dumps = dump_all_params()
        param_hashes = {}
        if dumps.get('controller'):
            param_hashes = record_trial_dumps(trial, dumps)
            mismatches = verify_trial_params(dumps, trial_values, all_params)
            abort_on_mismatch(mismatches)

        # Run all tests sequentially — continue after failures for graduated scoring
        test_results = []
        stderr_snippets = {}
        for test_id, waypoint_file, timeout, weight in tests:
            print(f"  Running {test_id}: {waypoint_file} (timeout={timeout}s, weight={weight})...",
                  file=sys.stderr)
            t0 = time.time()
            data = run_test(waypoint_file, timeout, use_amcl=use_amcl,
                           cycles=cycles)
            elapsed = time.time() - t0

            success, collision, test_duration, test_metrics, test_data = \
                extract_cycle_data(data, fallback_duration=elapsed)

            # Preserve raw per-cycle data for storage
            raw_cycles = data.get('results', [])

            # Extract accuracy from waypoint results (both success and failure)
            error_xy = None
            error_yaw = None
            wp_results = test_data.get('waypoint_results', [])
            if wp_results:
                last_wp = wp_results[-1]
                error_xy = last_wp.get('error_xy')
                error_yaw = last_wp.get('error_yaw')

            if collision:
                print(f"  COLLISION in {test_id}! penalty={COLLISION_PENALTY:.0f}",
                      file=sys.stderr)
                test_results.append({
                    'test_id': test_id,
                    'waypoint_file': waypoint_file,
                    'weight': weight,
                    'duration': test_duration,
                    'metrics': test_metrics,
                    'success': False,
                    'collision': True,
                    'error_xy': error_xy,
                    'error_yaw': error_yaw,
                    'raw_cycles': raw_cycles,
                })
                continue

            if not success:
                cycle_results = data.get('results', [])
                reason = (test_data.get('failure_reason', '') if cycle_results
                          else data.get('message', ''))
                print(f"  FAIL: {test_id} ({reason}).", file=sys.stderr)
                # Dump forensic lines from stderr and collect snippets
                stderr_text = data.get('_stderr', '')
                if stderr_text:
                    filtered = []
                    for line in stderr_text.splitlines():
                        if any(k in line.upper() for k in
                               ['TIMEOUT', 'DISTANCE', 'AMCL', 'ERROR', 'FAIL']):
                            print(f"    | {line.strip()}", file=sys.stderr)
                            filtered.append(line.strip())
                    if filtered:
                        stderr_snippets[test_id] = filtered[:20]

                # Stack crash vs bad params
                if not check_stack_health():
                    locked_dirty = True  # stack restart resets ROS2 params
                    handle_stack_crash(trial, trial_values, samplable,
                                      stack_target)

                penalty = compute_failure_penalty(error_xy, error_yaw)
                print(f"    PENALTY: {penalty:.0f} (error_xy={error_xy}, "
                      f"error_yaw={error_yaw})", file=sys.stderr)

                test_results.append({
                    'test_id': test_id,
                    'waypoint_file': waypoint_file,
                    'weight': weight,
                    'duration': test_duration,
                    'metrics': test_metrics,
                    'success': False,
                    'error_xy': error_xy,
                    'error_yaw': error_yaw,
                    'raw_cycles': raw_cycles,
                })
            else:
                print(f"    OK {test_id} in {test_duration:.1f}s", file=sys.stderr)
                test_results.append({
                    'test_id': test_id,
                    'waypoint_file': waypoint_file,
                    'weight': weight,
                    'duration': test_duration,
                    'metrics': test_metrics,
                    'success': True,
                    'error_xy': error_xy,
                    'error_yaw': error_yaw,
                    'raw_cycles': raw_cycles,
                })

        # Compute composite score
        score = compute_score(test_results, score_weights, jitter_weights)
        print(f"  Trial {trial.number} score: {score:.3f}", file=sys.stderr)

        # Store comprehensive trial data
        store_trial_data(
            trial, test_results,
            study_name=trial.study.study_name,
            score=score, trial_values=trial_values,
            stderr_snippets=stderr_snippets or None,
            param_hashes=param_hashes or None,
        )

        return score

    return objective


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Optuna MPPI tuning — shotgun composite full-matrix scorer',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python tune_mppi.py --trials 100 --tier 1
  python tune_mppi.py --trials 200 --tier 2 --resume
  python tune_mppi.py --trials 1 --baseline-only --tier 1
  python tune_mppi.py --trials 100 --tier 1 --full-reset-interval 50
  python tune_mppi.py --trials 50 --tier 1 --tests M1,M2
        """
    )
    add_common_args(parser)
    parser.add_argument('--tests', type=str, default=None,
                        help='Comma-separated test IDs to run (e.g. M1,M2). '
                             f'Available: {",".join(ALL_TESTS.keys())}. '
                             f'Default: {",".join(DEFAULT_TESTS)}.')
    parser.add_argument('--score-weights', type=str, default=None,
                        help='JSON dict of score component weights. '
                             'Keys: jitter (default 5.0), duration (0.1), '
                             'xtrack (0.05). e.g. \'{"jitter": 10.0}\'')
    parser.add_argument('--jitter-weights', type=str, default=None,
                        help='JSON dict of jitter sub-component weights. '
                             'Keys: steer_rms (2.0), steer_flip (0.5), '
                             'tv_angular (50.0), tv_linear (20.0), '
                             'tv_wheel (0.05), reversals (0.5). '
                             'e.g. \'{"steer_rms": 5.0, "reversals": 2.0}\'')

    args = parser.parse_args()

    # Parse weight overrides
    score_weights = None
    jitter_weights = None
    if args.score_weights:
        try:
            score_weights = json.loads(args.score_weights)
        except json.JSONDecodeError as e:
            print(f"ERROR: Invalid --score-weights JSON: {e}", file=sys.stderr)
            return 1
    if args.jitter_weights:
        try:
            jitter_weights = json.loads(args.jitter_weights)
            # Merge with defaults so partial overrides work
            merged = dict(DEFAULT_JITTER_WEIGHTS)
            merged.update(jitter_weights)
            jitter_weights = merged
        except json.JSONDecodeError as e:
            print(f"ERROR: Invalid --jitter-weights JSON: {e}", file=sys.stderr)
            return 1

    # Load .env for OPTUNA_STORAGE etc.
    load_dotenv()

    # Exclusive lock — prevent concurrent tuning harnesses on the same stack
    acquire_tuning_lock(args.study_name)

    # Resolve test list — 4-tuples: (test_id, waypoint_file, timeout, weight)
    if args.tests:
        test_ids = [t.strip().upper() for t in args.tests.split(',')]
        for tid in test_ids:
            if tid not in ALL_TESTS:
                print(f"ERROR: Unknown test '{tid}'. "
                      f"Available: {','.join(ALL_TESTS.keys())}",
                      file=sys.stderr)
                return 1
        tests = [(tid, *ALL_TESTS[tid]) for tid in test_ids]
    else:
        test_ids = DEFAULT_TESTS
        tests = [(tid, *ALL_TESTS[tid]) for tid in DEFAULT_TESTS]

    # Load params from YAML
    params_path = Path(args.params_file) if args.params_file else None
    all_params = load_tuning_params(max_tier=args.tier, path=params_path)

    if not all_params:
        print(f"ERROR: No params found at tier <= {args.tier}",
              file=sys.stderr)
        return 1

    # --params filter
    if args.params:
        try:
            apply_params_filter(all_params, args.params)
        except ValueError as e:
            print(f"ERROR: {e}", file=sys.stderr)
            return 1

    samplable = get_samplable_params(all_params)
    locked = get_locked_params(all_params)
    baseline = build_baseline(all_params)

    # Resolve storage
    storage = resolve_storage(args.storage, args.study_name)

    # Always restore baseline on exit
    atexit.register(restore_baseline, all_params)

    print(f"Optuna MPPI Tuning — Shotgun (composite)", file=sys.stderr)
    print(f"  Study: {args.study_name}", file=sys.stderr)
    print(f"  Storage: {storage}", file=sys.stderr)
    print(f"  Trials: {args.trials}", file=sys.stderr)
    print(f"  Tier: {args.tier} ({len(all_params)} params: "
          f"{len(samplable)} sampled, {len(locked)} locked)", file=sys.stderr)
    print(f"  Tests per trial: {len(tests)} ({','.join(test_ids)})",
          file=sys.stderr)
    print(f"  AMCL: {args.amcl}", file=sys.stderr)
    print(f"  Baseline-only: {args.baseline_only}", file=sys.stderr)
    print(f"  Stack: {args.stack}", file=sys.stderr)
    print(f"  Cycles per test: {args.cycles}", file=sys.stderr)
    print(f"  Full reset interval: {args.full_reset_interval or 'disabled'}",
          file=sys.stderr)
    drives_per_trial = len(tests) * args.cycles
    est_minutes = args.trials * drives_per_trial * 0.4  # ~24s/drive
    print(f"  Drives per trial: {drives_per_trial} ({len(tests)} tests x {args.cycles} cycles)",
          file=sys.stderr)
    print(f"  Estimated time: ~{est_minutes:.0f} min ({est_minutes / 60:.1f} hr)",
          file=sys.stderr)
    if score_weights:
        print(f"  Score weights: {score_weights}", file=sys.stderr)
    if jitter_weights:
        print(f"  Jitter weights: {jitter_weights}", file=sys.stderr)
    print(f"  Params:", file=sys.stderr)
    print_param_summary(all_params)

    # Create or load study
    study = create_or_load_study(
        study_name=args.study_name,
        storage=storage,
        resume=args.resume,
        direction='minimize',
        baseline=baseline,
        samplable=samplable,
    )

    # Set locked params once before optimization loop
    if locked:
        if not set_locked_params(all_params):
            print("ERROR: Failed to set locked params, aborting",
                  file=sys.stderr)
            return 1

    # Save study snapshot (param dumps + provenance)
    save_study_snapshot(
        args.study_name, all_params,
        is_resume=args.resume,
        params_file_path=args.params_file,
    )

    # Override trials count for baseline-only mode
    n_trials = 1 if args.baseline_only else args.trials

    # Run optimization
    objective = make_objective(
        all_params=all_params,
        tests=tests,
        baseline_only=args.baseline_only,
        stack_target=args.stack,
        full_reset_interval=args.full_reset_interval,
        use_amcl=args.amcl,
        cycles=args.cycles,
        score_weights=score_weights,
        jitter_weights=jitter_weights,
    )

    try:
        study.optimize(objective, n_trials=n_trials)
    except KeyboardInterrupt:
        print("\nInterrupted by user.", file=sys.stderr)

    # Print summary
    print(f"\n{'='*60}", file=sys.stderr)
    print(f"Optuna Summary", file=sys.stderr)
    print(f"{'='*60}", file=sys.stderr)
    print(f"  Completed trials: {len(study.trials)}", file=sys.stderr)

    complete = [t for t in study.trials
                if t.state == optuna.trial.TrialState.COMPLETE]
    if complete:
        best = study.best_trial
        print(f"  Best trial: #{best.number}", file=sys.stderr)
        print(f"  Best score: {best.value:.3f}", file=sys.stderr)
        print(f"  Best params:", file=sys.stderr)
        for k, v in sorted(best.params.items()):
            baseline_val = baseline.get(k, '?')
            delta = ''
            if isinstance(baseline_val, (int, float)) and isinstance(v, (int, float)):
                diff = v - baseline_val
                if abs(diff) > 0.001:
                    delta = f' (baseline: {baseline_val}, delta: {diff:+.3f})'
            print(f"    {k}: {v}{delta}", file=sys.stderr)

        # JSON to stdout for piping
        print(json.dumps({
            'best_trial': best.number,
            'best_score': best.value,
            'best_params': best.params,
            'total_trials': len(study.trials),
            'completed_trials': len(complete),
        }, indent=2))
    else:
        print("  No completed trials.", file=sys.stderr)

    print(f"{'='*60}", file=sys.stderr)

    return 0


if __name__ == '__main__':
    sys.exit(main())
