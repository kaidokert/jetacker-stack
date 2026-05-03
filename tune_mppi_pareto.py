#!/usr/bin/env python3
"""
Optuna MPPI Tuning Harness — "Pareto" lenient single-test multi-objective scorer.

Focuses on a single Nav2 matrix test with multi-objective Pareto optimization.
Configurable objectives (all minimized). Default: time, error_xy, error_yaw,
reversals, consistency.

Lenient scoring: only stack crashes and bogus durations (warm reset artifacts)
are pruned. Collisions, failures, and unstable drives are scored through — the
multi-objective sampler learns from their (bad) objective values. This maximizes
information per trial when exploring large parameter spaces on a single maneuver.

Architecture:
    tune_mppi_pareto.py (Optuna multi-objective, lenient)
      ├─ tune_common.py (shared infrastructure)
      ├─ NSGAIISampler or MOTPESampler
      ├─ Single test per trial (optionally repeated for noise reduction)
      ├─ Lenient: collision/failure -> scored, not pruned
      └─ Pareto front summary

Usage:
    python tune_mppi_pareto.py --test M4 --trials 200 --tier 1 --reps 5
    python tune_mppi_pareto.py --test M4 --trials 200 --params-file config/tuning_params_pareto_v2.yaml
    python tune_mppi_pareto.py --test M3 --trials 200 --tier 1 --reps 5 --objectives time,error_xy,error_yaw,jitter,reversals,collisions
    python tune_mppi_pareto.py --test M1 --trials 50 --tier 1 --baseline-only

See also: tune_mppi_focused.py for strict single-test (prunes on failure).
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
    extract_rep_metrics,
    full_restart_stack,
    get_locked_params,
    get_samplable_params,
    handle_stack_crash,
    load_dotenv,
    load_tuning_params,
    log_trial_params,
    print_param_summary,
    record_trial_dumps,
    select_medoid,
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
# Objective registry
# ---------------------------------------------------------------------------

# Full set of objectives that can be computed. Order = storage index in trial JSON.
FULL_OBJECTIVE_NAMES = [
    'time', 'error_xy', 'error_yaw', 'jitter', 'reversals', 'collisions',
    'consistency', 'pfc_mean',
]

# Default objectives for Optuna optimization.
DEFAULT_OBJECTIVES = ['time', 'error_xy', 'error_yaw', 'reversals', 'consistency']


# ---------------------------------------------------------------------------
# Optuna objective
# ---------------------------------------------------------------------------

def make_pareto_objective(all_params: dict, test_spec: tuple,
                          test_id: str = 'unknown',
                          baseline_only: bool = False,
                          stack_target: str = 'jetacker:nav2_odom',
                          full_reset_interval: int = 0,
                          use_amcl: bool = False, reps: int = 1,
                          record: bool = False,
                          active_objectives: list[str] = None):
    """Create the lenient multi-objective function for a single test.

    Lenient scoring: only stack crashes and bogus durations are pruned.
    Collisions, failures, and unstable drives are scored through — the
    multi-objective sampler learns from their (bad) objective values.

    Always computes all 7 objectives and stores them in the trial JSON.
    Returns only the active_objectives subset to Optuna for optimization.
    """
    if active_objectives is None:
        active_objectives = list(DEFAULT_OBJECTIVES)
    active_indices = [FULL_OBJECTIVE_NAMES.index(o) for o in active_objectives]

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
            if not set_locked_params(all_params):
                # Locked params failed even after retries — restart stack
                print("  Locked params failed, restarting stack...",
                      file=sys.stderr)
                full_restart_stack(stack_target)
                if not set_locked_params(all_params):
                    raise RuntimeError(
                        "Locked params failed after stack restart — aborting")
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
        # Track all reps, then filter to successful ones for scoring.
        # Require >=3 successful reps; prune if >2 failures.
        MIN_SUCCESSFUL_REPS = 3

        all_rep_data = []  # list of dicts per rep

        for rep in range(reps):
            if reps > 1:
                print(f"  Rep {rep+1}/{reps} of {waypoint_file}...",
                      file=sys.stderr)
            else:
                print(f"  Running {waypoint_file} (timeout={timeout}s)...",
                      file=sys.stderr)

            t0 = time.time()
            data = run_test(waypoint_file, timeout, use_amcl=use_amcl,
                            record=record, trust_nav2=True)
            elapsed = time.time() - t0

            rep_m = extract_rep_metrics(data, elapsed)
            success = rep_m['success']
            collision = rep_m['collision']
            duration = rep_m['duration']
            error_xy = rep_m['error_xy']
            error_yaw = rep_m['error_yaw']
            metrics = rep_m['_metrics']
            test_data = rep_m['_test_data']

            # Stack crash detection — infrastructure failure, not param-related
            if not success and not collision:
                if not check_stack_health():
                    locked_dirty = True
                    handle_stack_crash(trial, trial_values, samplable,
                                      stack_target)
                    # handle_stack_crash raises RuntimeError — won't reach here

            # Bogus duration — warm reset artifact, bad data
            if duration < 5.0:
                reason = f'bogus_duration={duration:.1f}s'
                print(f"  BOGUS rep {rep+1}: {reason} (warm reset artifact?)",
                      file=sys.stderr)
                trial.set_user_attr('prune_reason', reason)
                store_trial_data(
                    trial,
                    test_results=[{
                        'test_id': test_id,
                        'waypoint_file': waypoint_file,
                        'weight': 1.0,
                        'success': False,
                        'duration': duration,
                        'metrics': metrics,
                        'failure_reason': reason,
                    }],
                    study_name=trial.study.study_name,
                    trial_values=trial_values,
                    param_hashes=param_hashes or None,
                )
                raise optuna.TrialPruned(
                    f"Implausibly fast on rep {rep+1}: {reason}")

            # Determine rep outcome
            rep_success = success and not collision

            rep_error_xy = error_xy if error_xy is not None else 10.0
            rep_error_yaw = error_yaw if error_yaw is not None else math.pi

            # Log outcome
            if collision:
                dist_str = f', dist={rep_error_xy:.2f}m' if error_xy else ''
                print(f"  COLLISION rep {rep+1} at t={duration:.1f}s{dist_str}",
                      file=sys.stderr)
                trial.set_user_attr('had_collision', True)
            elif not success:
                cycle_results = data.get('results', [])
                reason = (test_data.get('failure_reason', '') if cycle_results
                          else data.get('message', ''))
                print(f"  FAIL rep {rep+1}: {reason} at t={duration:.1f}s",
                      file=sys.stderr)
                trial.set_user_attr('had_failure', True)

            all_rep_data.append({
                'success': rep_success,
                'duration': duration,
                'error_xy': rep_error_xy,
                'error_yaw': rep_error_yaw,
                'jitter': rep_m['jitter'],
                'reversals': rep_m['reversal_count'],
                'collisions': rep_m['collisions'],
                'metrics': metrics,
                'pfc': {k: rep_m[k] for k in ('pfc_mean', 'pfc_max',
                        'pfc_integral', 'pfc_count') if rep_m.get(k) is not None},
            })

            if reps > 1:
                status = 'OK' if rep_success else 'FAIL'
                coll_str = ' COLL' if rep_m['collisions'] else ''
                print(f"    Rep {rep+1} [{status}]: time={duration:.1f}s, "
                      f"reversals={rep_m['reversal_count']}{coll_str}", file=sys.stderr)

        # --- Filter to successful reps; prune if too few ---
        successful_indices = [i for i, r in enumerate(all_rep_data)
                              if r['success']]
        n_success = len(successful_indices)
        n_fail = len(all_rep_data) - n_success

        print(f"  Reps: {n_success}/{len(all_rep_data)} successful",
              file=sys.stderr)

        # Single-rep: prune on any failure (timeout, abort, collision)
        if reps == 1 and n_success == 0:
            reason = 'single-rep failure'
            print(f"  PRUNE: {reason}", file=sys.stderr)
            trial.set_user_attr('prune_reason', reason)
            trial.set_user_attr('reps_succeeded', 0)
            trial.set_user_attr('reps_failed', 1)
            store_trial_data(
                trial,
                test_results=[{
                    'test_id': test_id,
                    'waypoint_file': waypoint_file,
                    'weight': 1.0,
                    'success': False,
                    'duration': all_rep_data[0]['duration'],
                    'error_xy': all_rep_data[0]['error_xy'],
                    'error_yaw': all_rep_data[0]['error_yaw'],
                    'metrics': all_rep_data[0]['metrics'],
                    'failure_reason': reason,
                }],
                study_name=trial.study.study_name,
                trial_values=trial_values,
                param_hashes=param_hashes or None,
            )
            raise optuna.TrialPruned(reason)

        if reps >= MIN_SUCCESSFUL_REPS and n_success < MIN_SUCCESSFUL_REPS:
            reason = f'{n_fail}/{reps} reps failed (need >={MIN_SUCCESSFUL_REPS} successes)'
            print(f"  PRUNE: {reason}", file=sys.stderr)
            trial.set_user_attr('prune_reason', reason)
            trial.set_user_attr('reps_succeeded', n_success)
            trial.set_user_attr('reps_failed', n_fail)
            store_trial_data(
                trial,
                test_results=[{
                    'test_id': test_id,
                    'waypoint_file': waypoint_file,
                    'weight': 1.0,
                    'success': False,
                    'duration': all_rep_data[0]['duration'],
                    'error_xy': all_rep_data[0]['error_xy'],
                    'error_yaw': all_rep_data[0]['error_yaw'],
                    'metrics': all_rep_data[0]['metrics'],
                    'failure_reason': reason,
                }],
                study_name=trial.study.study_name,
                trial_values=trial_values,
                param_hashes=param_hashes or None,
            )
            raise optuna.TrialPruned(reason)

        # Score from successful reps only (or all if reps < MIN_SUCCESSFUL_REPS)
        if n_success > 0 and reps >= MIN_SUCCESSFUL_REPS:
            score_indices = successful_indices
        else:
            # Single rep or few reps — use all
            score_indices = list(range(len(all_rep_data)))

        times = [all_rep_data[i]['duration'] for i in score_indices]
        accuracies = [all_rep_data[i]['error_xy'] for i in score_indices]
        yaw_errors = [all_rep_data[i]['error_yaw'] for i in score_indices]
        jitters = [all_rep_data[i]['jitter'] for i in score_indices]
        reversals = [all_rep_data[i]['reversals'] for i in score_indices]
        collision_counts = [all_rep_data[i]['collisions'] for i in score_indices]
        scored_metrics = [all_rep_data[i]['metrics'] for i in score_indices]
        pfc_means = [all_rep_data[i].get('pfc', {}).get('pfc_mean', 999.0)
                     for i in score_indices]

        coll_val = sum(r['collisions'] for r in all_rep_data)  # total across ALL reps

        if len(times) == 1:
            idx = 0
            consistency = 0.0
        else:
            # Medoid from successful reps only
            medoid_obj_names = [o for o in active_objectives
                                if o not in ('consistency', 'collisions')]
            per_rep_data = {
                'time': times, 'error_xy': accuracies,
                'error_yaw': yaw_errors, 'jitter': jitters,
                'reversals': reversals, 'pfc_mean': pfc_means,
            }
            rep_objectives = [
                [per_rep_data[o][i] for o in medoid_obj_names]
                for i in range(len(times))
            ]
            idx, raw_consistency = select_medoid(rep_objectives)
            # Weight consistency by failure rate: a 3/5 trial with perfect
            # consistency among successes should still score worse than a 5/5
            # trial with some spread. Without this, NSGA-II prefers flaky
            # configs that happen to cluster tightly on their passing reps.
            # The additive term ensures any failure dominates pure variance.
            failure_penalty = n_fail / reps  # 0.0 for 5/5, 0.4 for 3/5
            consistency = raw_consistency + failure_penalty
            print(f"  Medoid: rep {score_indices[idx]+1}/{reps} "
                  f"(from {len(score_indices)} successful), "
                  f"consistency={consistency:.3f} "
                  f"(raw={raw_consistency:.3f} + fail_penalty={failure_penalty:.2f})",
                  file=sys.stderr)

        time_val = times[idx]
        acc_val = accuracies[idx]
        yaw_val = yaw_errors[idx]
        jitter_val = jitters[idx]
        rev_val = reversals[idx]
        rep_metrics = scored_metrics[idx]

        trial.set_user_attr('consistency', consistency)
        trial.set_user_attr('reversals', rev_val)
        trial.set_user_attr('reps_succeeded', n_success)
        trial.set_user_attr('reps_failed', n_fail)

        # Store PFC metrics from medoid rep (if available)
        rep_pfc = all_rep_data[score_indices[idx]].get('pfc', {})
        if rep_pfc:
            for k, v in rep_pfc.items():
                trial.set_user_attr(k, v)

        pfc_str = f", pfc_mean={rep_pfc['pfc_mean']:.2f}" if rep_pfc else ""
        print(f"  Trial {trial.number}: time={time_val:.1f}s, "
              f"jitter={jitter_val:.2f}, reversals={rev_val:.0f}, "
              f"collisions={coll_val}, consistency={consistency:.3f} "
              f"({n_success}/{reps} passed){pfc_str}",
              file=sys.stderr)

        # PFC mean from medoid rep (fallback to large value if unavailable)
        pfc_mean_val = rep_pfc.get('pfc_mean', 999.0) if rep_pfc else 999.0

        # Build full objectives tuple (all 8, order matches FULL_OBJECTIVE_NAMES)
        full_objectives = (
            time_val, acc_val, yaw_val, jitter_val, rev_val, coll_val,
            consistency, pfc_mean_val,
        )

        # Store comprehensive trial data
        store_trial_data(
            trial,
            test_results=[{
                'test_id': test_id,
                'waypoint_file': waypoint_file,
                'weight': 1.0,
                'success': n_fail == 0,
                'collision': coll_val > 0,
                'duration': time_val,
                'metrics': rep_metrics,
                'error_xy': acc_val,
                'error_yaw': yaw_val,
            }],
            study_name=trial.study.study_name,
            objectives=full_objectives,
            trial_values=trial_values,
            param_hashes=param_hashes or None,
        )

        # Return only active objectives to Optuna
        return tuple(full_objectives[i] for i in active_indices)

    return objective


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Optuna MPPI tuning — lenient Pareto single-test multi-objective',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
Examples:
  python tune_mppi_pareto.py --test M4 --trials 200 --tier 1
  python tune_mppi_pareto.py --test M4 --trials 200 --params-file config/tuning_params_pareto_v1.yaml
  python tune_mppi_pareto.py --test M3 --trials 200 --tier 1 --reps 3
  python tune_mppi_pareto.py --test M5 --trials 200 --tier 1 --sampler motpe

Available tests: {', '.join(ALL_TESTS.keys())}
        """
    )
    add_common_args(parser, default_study='mppi_pareto')
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
    parser.add_argument('--objectives', type=str, default=None,
                        help='Comma-separated list of active objectives '
                             '(default: ' + ','.join(DEFAULT_OBJECTIVES) + '). '
                             'Available: ' + ','.join(FULL_OBJECTIVE_NAMES))

    args = parser.parse_args()

    # Parse active objectives
    if args.objectives:
        active_objectives = [o.strip() for o in args.objectives.split(',')]
        for o in active_objectives:
            if o not in FULL_OBJECTIVE_NAMES:
                print(f"ERROR: Unknown objective '{o}'. "
                      f"Available: {','.join(FULL_OBJECTIVE_NAMES)}",
                      file=sys.stderr)
                return 1
    else:
        active_objectives = list(DEFAULT_OBJECTIVES)

    # Load .env for OPTUNA_STORAGE etc.
    load_dotenv()

    # Auto-log to timestamped file (append-safe, never overwrites)
    from datetime import datetime
    log_dir = Path('logs')
    log_dir.mkdir(exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_path = log_dir / f'{args.study_name}_{ts}.log'

    class TeeStderr:
        """Tee stderr to both console and a log file (append mode)."""
        def __init__(self, log_file, original):
            self.log_file = log_file
            self.original = original
        def write(self, data):
            self.original.write(data)
            self.log_file.write(data)
            self.log_file.flush()
        def flush(self):
            self.original.flush()
            self.log_file.flush()

    _log_fh = open(log_path, 'a', encoding='utf-8')
    sys.stderr = TeeStderr(_log_fh, sys.stderr)
    print(f"  Logging to: {log_path}", file=sys.stderr)

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
    if study_name == 'mppi_pareto':
        study_name = f'mppi_pareto_{test_id}'
    storage = resolve_storage(args.storage, study_name)

    # Sampler
    if args.sampler == 'motpe':
        sampler = optuna.samplers.MOTPESampler()
    else:
        sampler = optuna.samplers.NSGAIISampler()

    # Always restore baseline on exit
    atexit.register(restore_baseline, all_params)

    print(f"Optuna MPPI Tuning — Pareto (lenient multi-objective)", file=sys.stderr)
    print(f"  Study: {study_name}", file=sys.stderr)
    print(f"  Storage: {storage}", file=sys.stderr)
    print(f"  Test: {test_id} ({test_spec[0]})", file=sys.stderr)
    print(f"  Trials: {args.trials}", file=sys.stderr)
    print(f"  Reps/trial: {args.reps}", file=sys.stderr)
    print(f"  Sampler: {args.sampler.upper()}", file=sys.stderr)
    print(f"  Tier: {args.tier} ({len(all_params)} params: "
          f"{len(samplable)} sampled, {len(locked)} locked)", file=sys.stderr)
    print(f"  Objectives: {', '.join(active_objectives)} (all minimized)",
          file=sys.stderr)
    print(f"  Scoring: LENIENT (failures/collisions scored, not pruned)",
          file=sys.stderr)
    print(f"  AMCL: {args.amcl}", file=sys.stderr)
    print(f"  Baseline-only: {args.baseline_only}", file=sys.stderr)
    print(f"  Stack: {args.stack}", file=sys.stderr)
    print(f"  Full reset interval: {args.full_reset_interval or 'disabled'}",
          file=sys.stderr)
    if params_path:
        print(f"  Params file: {params_path}", file=sys.stderr)
    est_per_trial = args.reps * 0.5  # rough minutes per rep
    est_minutes = int(args.trials * est_per_trial)
    print(f"  Estimated time: ~{est_minutes} min ({est_minutes / 60:.1f} hr)",
          file=sys.stderr)
    print(f"  Params:", file=sys.stderr)
    print_param_summary(all_params)

    # Create or load study (multi-objective)
    METRIC_NAMES = list(active_objectives)
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
    objective = make_pareto_objective(
        all_params=all_params,
        test_spec=test_spec,
        test_id=test_id,
        baseline_only=args.baseline_only,
        stack_target=args.stack,
        full_reset_interval=args.full_reset_interval,
        use_amcl=args.amcl,
        reps=args.reps,
        record=args.record,
        active_objectives=active_objectives,
    )

    try:
        study.optimize(objective, n_trials=n_trials,
                       catch=(RuntimeError,))
    except KeyboardInterrupt:
        print("\nInterrupted by user.", file=sys.stderr)

    # Print Pareto front summary
    print(f"\n{'='*60}", file=sys.stderr)
    print(f"Optuna Pareto Summary — {test_id} (lenient)", file=sys.stderr)
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

        # Format helpers for each objective type
        def _fmt_obj(name, val):
            if name == 'error_yaw':
                return f"{name}={math.degrees(val):.1f}deg"
            elif name in ('time', 'jitter', 'consistency'):
                return f"{name}={val:.1f}" if val >= 10 else f"{name}={val:.2f}"
            elif name in ('error_xy',):
                return f"{name}={val:.3f}m"
            elif name in ('reversals', 'collisions'):
                return f"{name}={val:.0f}"
            else:
                return f"{name}={val:.3f}"

        pareto_json = []
        for t in sorted(pareto, key=lambda x: x.values[0]):
            vals = t.values
            obj_strs = [_fmt_obj(active_objectives[i], vals[i])
                        for i in range(len(active_objectives))]
            print(f"  Trial {t.number}: {', '.join(obj_strs)}",
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

            trial_obj = {active_objectives[i]: vals[i]
                         for i in range(len(active_objectives))}
            trial_obj['trial'] = t.number
            trial_obj['params'] = t.params
            pareto_json.append(trial_obj)

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
