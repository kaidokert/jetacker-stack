#!/usr/bin/env python3
"""
Optuna MPPI Tuning Harness — "Laser" single-test multi-objective scorer.

Focuses on a single Nav2 matrix test with multi-objective Pareto optimization.
Four objectives (all minimized): time_to_goal, accuracy_error, jitter, reversals.
Strict pass/fail: collisions and failures are pruned (no graduated penalties).

Architecture:
    tune_mppi_focused.py (Optuna multi-objective)
      ├─ tune_common.py (shared infrastructure)
      ├─ NSGAIISampler or MOTPESampler
      ├─ Single test per trial (optionally repeated for noise reduction)
      ├─ Strict: collision/failure -> TrialPruned
      └─ Pareto front summary

Usage:
    python tune_mppi_focused.py --test M3 --trials 200 --tier 1
    python tune_mppi_focused.py --test M7 --trials 100 --tier 1 --reps 3
    python tune_mppi_focused.py --test M5 --trials 200 --tier 1 --sampler motpe
    python tune_mppi_focused.py --test M1 --trials 50 --tier 1 --baseline-only

See also: tune_mppi.py for composite full-matrix "shotgun" tuning.
"""

import argparse
import atexit
import json
import math
import sys
import time
from pathlib import Path

import optuna

from tune_common import (
    ALL_TESTS,
    abort_on_mismatch,
    acquire_tuning_lock,
    add_common_args,
    apply_params_filter,
    build_baseline,
    check_stack_health,
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
# Helpers
# ---------------------------------------------------------------------------

def _median(vals: list) -> float:
    """Median of a list of numbers."""
    s = sorted(vals)
    n = len(s)
    if n % 2 == 1:
        return s[n // 2]
    return (s[n // 2 - 1] + s[n // 2]) / 2


# ---------------------------------------------------------------------------
# Optuna objective
# ---------------------------------------------------------------------------

def make_focused_objective(all_params: dict, test_spec: tuple,
                           test_id: str = 'unknown',
                           baseline_only: bool = False,
                           stack_target: str = 'jetacker:nav2_odom',
                           full_reset_interval: int = 0,
                           use_amcl: bool = False, reps: int = 1,
                           record: bool = False):
    """Create the multi-objective function for a single test.

    Returns (time_to_goal, error_xy, error_yaw, jitter, reversals) — all minimized.
    """

    samplable = get_samplable_params(all_params)
    locked = get_locked_params(all_params)
    baseline = build_baseline(all_params)
    trial_count = 0
    locked_dirty = False  # True after stack restart (locked params need re-set)

    waypoint_file, timeout, _weight = test_spec

    def objective(trial: optuna.Trial) -> tuple:
        nonlocal trial_count, locked_dirty
        trial_count += 1

        # Prophylactic full restart every N trials
        if (full_reset_interval > 0
                and trial_count > 1
                and (trial_count - 1) % full_reset_interval == 0):
            print(f"\n  PROPHYLACTIC RESTART (every {full_reset_interval} "
                  f"trials, count={trial_count})", file=sys.stderr)
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

        # Run test (possibly multiple reps)
        times = []
        accuracies = []
        yaw_errors = []
        jitters = []
        reversals = []

        for rep in range(reps):
            if reps > 1:
                print(f"  Rep {rep+1}/{reps} of {waypoint_file}...",
                      file=sys.stderr)
            else:
                print(f"  Running {waypoint_file} (timeout={timeout}s)...",
                      file=sys.stderr)

            t0 = time.time()
            data = run_test(waypoint_file, timeout, use_amcl=use_amcl,
                            record=record)
            elapsed = time.time() - t0

            success, collision, duration, metrics, test_data = \
                extract_cycle_data(data, fallback_duration=elapsed)

            # Extract error distances (available for both success and failure)
            wp_results_raw = test_data.get('waypoint_results', [])
            fail_error_xy = None
            fail_error_yaw = None
            fail_reason_wp = None
            if wp_results_raw:
                last_wp = wp_results_raw[-1]
                fail_error_xy = last_wp.get('error_xy')
                fail_error_yaw = last_wp.get('error_yaw')
                fail_reason_wp = last_wp.get('reason')

            # Helper: store failed trial data before pruning
            def _store_failed(reason_str):
                store_trial_data(
                    trial,
                    test_results=[{
                        'test_id': test_id,
                        'waypoint_file': waypoint_file,
                        'weight': 1.0,
                        'success': False,
                        'collision': collision,
                        'duration': duration,
                        'metrics': metrics,
                        'error_xy': fail_error_xy,
                        'error_yaw': fail_error_yaw,
                        'failure_reason': reason_str,
                    }],
                    study_name=trial.study.study_name,
                    trial_values=trial_values,
                    param_hashes=param_hashes or None,
                )

            # Early stack crash detection — before attributing failure to params
            if not success and not collision:
                if not check_stack_health():
                    locked_dirty = True  # stack restart resets ROS2 params
                    handle_stack_crash(trial, trial_values, samplable,
                                      stack_target)
                    # handle_stack_crash raises RuntimeError — won't reach here

            if collision:
                dist_str = f', dist={fail_error_xy:.2f}m' if fail_error_xy else ''
                print(f"  COLLISION in {waypoint_file} at t={duration:.1f}s{dist_str}!",
                      file=sys.stderr)
                trial.set_user_attr('prune_reason', f'collision_rep{rep+1}')
                _store_failed(f'collision_rep{rep+1}')
                raise optuna.TrialPruned(
                    f"Collision on rep {rep+1}")

            if not success:
                cycle_results = data.get('results', [])
                reason = (test_data.get('failure_reason', '') if cycle_results
                          else data.get('message', ''))
                dist_str = f', dist={fail_error_xy:.2f}m' if fail_error_xy else ''
                yaw_str = f', yaw_err={math.degrees(fail_error_yaw):.1f}°' if fail_error_yaw else ''
                print(f"  FAIL: {waypoint_file} ({reason}{dist_str}{yaw_str})",
                      file=sys.stderr)

                trial.set_user_attr('prune_reason', reason or 'unknown')
                _store_failed(reason or 'unknown')
                raise optuna.TrialPruned(
                    f"Failed on rep {rep+1}: {reason}")

            # Sanity check — implausibly fast = warm reset failure
            if duration < 5.0:
                reason = f'bogus_duration={duration:.1f}s'
                print(f"  BOGUS rep {rep+1}: {reason} (warm reset artifact?)",
                      file=sys.stderr)
                trial.set_user_attr('prune_reason', reason)
                _store_failed(reason)
                raise optuna.TrialPruned(
                    f"Implausibly fast on rep {rep+1}: {reason}")

            # Extract accuracy from waypoint results
            error_xy = 0.0
            error_yaw = 0.0
            wp_results = test_data.get('waypoint_results', [])
            if wp_results:
                last_wp = wp_results[-1]
                error_xy = last_wp.get('error_xy', 0.0) or 0.0
                error_yaw = last_wp.get('error_yaw', 0.0) or 0.0

            cmdvel = metrics.get('cmd_vel', {})
            rev_count = cmdvel.get('reversal_count', 0)

            jitter_score = compute_jitter_score(metrics)

            # Quality gate — reject unstable drives (p≈0.8 cutoff)
            if jitter_score > 40 or rev_count > 75:
                reason = (f'unstable_jitter={jitter_score:.0f}'
                          if jitter_score > 40
                          else f'unstable_rev={rev_count}')
                print(f"  UNSTABLE rep {rep+1}: {reason}",
                      file=sys.stderr)
                trial.set_user_attr('prune_reason', reason)
                raise optuna.TrialPruned(
                    f"Unstable drive on rep {rep+1}: {reason}")

            times.append(duration)
            accuracies.append(error_xy)
            yaw_errors.append(error_yaw)
            jitters.append(jitter_score)
            reversals.append(rev_count)
            last_metrics = metrics

            if reps > 1:
                print(f"    Rep {rep+1}: time={duration:.1f}s, "
                      f"accuracy={error_xy:.3f}m, "
                      f"jitter={jitters[-1]:.2f}, "
                      f"reversals={rev_count}", file=sys.stderr)

        # Take median for noise reduction
        time_val = _median(times)
        acc_val = _median(accuracies)
        yaw_val = _median(yaw_errors)
        jitter_val = _median(jitters)
        rev_val = _median(reversals)

        print(f"  Trial {trial.number}: time={time_val:.1f}s, "
              f"error_xy={acc_val:.3f}m, error_yaw={math.degrees(yaw_val):.1f}°, "
              f"jitter={jitter_val:.2f}, reversals={rev_val:.0f}",
              file=sys.stderr)

        # Store comprehensive trial data
        store_trial_data(
            trial,
            test_results=[{
                'test_id': test_id,
                'waypoint_file': waypoint_file,
                'weight': 1.0,
                'success': True,
                'collision': False,
                'duration': time_val,
                'metrics': last_metrics,
                'error_xy': acc_val,
                'error_yaw': yaw_val,
            }],
            study_name=trial.study.study_name,
            objectives=(time_val, acc_val, yaw_val, jitter_val, rev_val),
            trial_values=trial_values,
            param_hashes=param_hashes or None,
        )

        return time_val, acc_val, yaw_val, jitter_val, rev_val

    return objective


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Optuna MPPI tuning — laser single-test multi-objective',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
Examples:
  python tune_mppi_focused.py --test M3 --trials 200 --tier 1
  python tune_mppi_focused.py --test M7 --trials 100 --tier 1 --reps 3
  python tune_mppi_focused.py --test M5 --trials 200 --tier 1 --sampler motpe

Available tests: {', '.join(ALL_TESTS.keys())}
        """
    )
    add_common_args(parser, default_study='mppi_focused')
    parser.add_argument('--test', type=str, required=True,
                        help='Single test ID to optimize (e.g. M3, M5, M7). '
                             f'Available: {",".join(ALL_TESTS.keys())}')
    parser.add_argument('--reps', type=int, default=1,
                        help='Repetitions per trial for noise reduction '
                             '(median of each objective). Default: 1')
    parser.add_argument('--sampler', type=str, default='nsgaii',
                        choices=['nsgaii', 'motpe'],
                        help='Multi-objective sampler (default: nsgaii)')
    parser.add_argument('--record', action='store_true',
                        help='Record rosbag2 during each test')

    args = parser.parse_args()

    # Load .env for OPTUNA_STORAGE etc.
    load_dotenv()

    # Exclusive lock — prevent concurrent tuning harnesses on the same stack
    acquire_tuning_lock(args.study_name)

    # Resolve test
    test_id = args.test.strip().upper()
    if test_id not in ALL_TESTS:
        print(f"ERROR: Unknown test '{test_id}'. "
              f"Available: {','.join(ALL_TESTS.keys())}",
              file=sys.stderr)
        return 1
    test_spec = ALL_TESTS[test_id]

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

    # Resolve storage — include test ID in default study name
    study_name = args.study_name
    if study_name == 'mppi_focused':
        study_name = f'mppi_focused_{test_id}'
    storage = resolve_storage(args.storage, study_name)

    # Sampler
    if args.sampler == 'motpe':
        sampler = optuna.samplers.MOTPESampler()
    else:
        sampler = optuna.samplers.NSGAIISampler()

    # Always restore baseline on exit
    atexit.register(restore_baseline, all_params)

    print(f"Optuna MPPI Tuning — Laser (multi-objective)", file=sys.stderr)
    print(f"  Study: {study_name}", file=sys.stderr)
    print(f"  Storage: {storage}", file=sys.stderr)
    print(f"  Test: {test_id} ({test_spec[0]})", file=sys.stderr)
    print(f"  Trials: {args.trials}", file=sys.stderr)
    print(f"  Reps/trial: {args.reps}", file=sys.stderr)
    print(f"  Sampler: {args.sampler.upper()}", file=sys.stderr)
    print(f"  Tier: {args.tier} ({len(all_params)} params: "
          f"{len(samplable)} sampled, {len(locked)} locked)", file=sys.stderr)
    print(f"  Objectives: time_to_goal (min), error_xy (min), error_yaw (min), "
          f"jitter (min), reversals (min)", file=sys.stderr)
    print(f"  AMCL: {args.amcl}", file=sys.stderr)
    print(f"  Baseline-only: {args.baseline_only}", file=sys.stderr)
    print(f"  Stack: {args.stack}", file=sys.stderr)
    print(f"  Full reset interval: {args.full_reset_interval or 'disabled'}",
          file=sys.stderr)
    est_per_trial = args.reps * 0.5  # rough minutes per rep
    est_minutes = int(args.trials * est_per_trial)
    print(f"  Estimated time: ~{est_minutes} min ({est_minutes / 60:.1f} hr)",
          file=sys.stderr)
    print(f"  Params:", file=sys.stderr)
    print_param_summary(all_params)

    # Create or load study (multi-objective)
    METRIC_NAMES = ['time_to_goal', 'error_xy', 'error_yaw', 'jitter', 'reversals']
    directions = ['minimize'] * len(METRIC_NAMES)

    if args.resume:
        study = create_or_load_study(
            study_name=study_name,
            storage=storage,
            resume=True,
        )
    else:
        # create_or_load_study with directions + sampler
        study = optuna.create_study(
            study_name=study_name,
            storage=storage,
            directions=directions,
            sampler=sampler,
            load_if_exists=True,
        )
        if len(study.trials) == 0 and samplable:
            baseline_enqueue = {name: baseline[name] for name in samplable}
            study.enqueue_trial(baseline_enqueue)
            print(f"  Created new study, baseline enqueued as trial 0",
                  file=sys.stderr)
        else:
            print(f"  Loaded existing study with {len(study.trials)} trials",
                  file=sys.stderr)

    # Set metric names for Optuna dashboard
    study.set_metric_names(METRIC_NAMES)

    # Set locked params once before optimization loop
    if locked:
        if not set_locked_params(all_params):
            print("ERROR: Failed to set locked params, aborting",
                  file=sys.stderr)
            return 1

    # Save study snapshot (param dumps + provenance)
    save_study_snapshot(
        study_name, all_params,
        is_resume=args.resume,
        params_file_path=args.params_file,
    )

    # Override trials count for baseline-only mode
    n_trials = 1 if args.baseline_only else args.trials

    # Run optimization
    objective = make_focused_objective(
        all_params=all_params,
        test_spec=test_spec,
        test_id=test_id,
        baseline_only=args.baseline_only,
        stack_target=args.stack,
        full_reset_interval=args.full_reset_interval,
        use_amcl=args.amcl,
        reps=args.reps,
        record=args.record,
    )

    try:
        study.optimize(objective, n_trials=n_trials,
                       catch=(RuntimeError,))
    except KeyboardInterrupt:
        print("\nInterrupted by user.", file=sys.stderr)

    # Print Pareto front summary
    print(f"\n{'='*60}", file=sys.stderr)
    print(f"Optuna Multi-Objective Summary — {test_id}", file=sys.stderr)
    print(f"{'='*60}", file=sys.stderr)
    print(f"  Total trials: {len(study.trials)}", file=sys.stderr)

    complete = [t for t in study.trials
                if t.state == optuna.trial.TrialState.COMPLETE]
    pruned = [t for t in study.trials
              if t.state == optuna.trial.TrialState.PRUNED]
    print(f"  Completed: {len(complete)}, Pruned: {len(pruned)}",
          file=sys.stderr)

    if complete:
        pareto = study.best_trials
        print(f"\nPareto front: {len(pareto)} trials", file=sys.stderr)

        pareto_json = []
        for t in sorted(pareto, key=lambda x: x.values[0]):
            vals = t.values
            print(f"  Trial {t.number}: time={vals[0]:.1f}s, "
                  f"error_xy={vals[1]:.3f}m, error_yaw={math.degrees(vals[2]):.1f}°, "
                  f"jitter={vals[3]:.2f}, reversals={vals[4]:.0f}",
                  file=sys.stderr)
            for k, v in sorted(t.params.items()):
                baseline_val = baseline.get(k, '?')
                delta = ''
                if (isinstance(baseline_val, (int, float))
                        and isinstance(v, (int, float))):
                    diff = v - baseline_val
                    if abs(diff) > 0.001:
                        delta = f' (delta: {diff:+.3f})'
                print(f"    {k}: {v}{delta}", file=sys.stderr)

            pareto_json.append({
                'trial': t.number,
                'time_to_goal': vals[0],
                'error_xy': vals[1],
                'error_yaw': vals[2],
                'jitter': vals[3],
                'reversals': vals[4],
                'params': t.params,
            })

        # JSON to stdout for piping
        print(json.dumps({
            'test': test_id,
            'total_trials': len(study.trials),
            'completed': len(complete),
            'pruned': len(pruned),
            'pareto_front': pareto_json,
        }, indent=2))
    else:
        print("  No completed trials (all pruned).", file=sys.stderr)

    print(f"{'='*60}", file=sys.stderr)

    return 0


if __name__ == '__main__':
    sys.exit(main())
