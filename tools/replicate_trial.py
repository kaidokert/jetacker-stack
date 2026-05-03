#!/usr/bin/env python3
"""Replicate a specific trial from a study.

Loads params from the trial JSON, applies them to the running stack,
runs the same test(s), and compares results against the original.

Usage:
    # Replicate trial 24 from pareto study (all tests in that trial)
    python tools/replicate_trial.py pareto_3critic_M4_v4 24

    # Specific test only, 3 cycles for confidence
    python tools/replicate_trial.py pareto_3critic_M4_v4 24 --test M4 --cycles 3

    # With warm reset between cycles
    python tools/replicate_trial.py pareto_3critic_M4_v4 24 --cycles 5 --warm-reset

    # Also set locked params from a params file (for fresh stack)
    python tools/replicate_trial.py pareto_3critic_M4_v4 24 \
        --params-file config/tuning_params_pareto_v2.yaml
"""

import argparse
import hashlib
import json
import sys
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tune_common import (
    ALL_TESTS,
    DUMP_NODE_KEYS,
    compute_jitter_score,
    load_tuning_params,
    set_all_params,
    dump_all_params,
    verify_trial_params,
)

REPLICATION_DUMP_DIR = Path("logs/replication_dumps")


def save_replication_dumps(dumps: dict, study: str, trial_num: int) -> dict[str, str]:
    """Save param dumps under replication dir, return hashes per node."""
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = REPLICATION_DUMP_DIR / f"{study}_t{trial_num:04d}_{ts}"
    out_dir.mkdir(parents=True, exist_ok=True)

    hashes = {}
    for key in DUMP_NODE_KEYS:
        content = dumps.get(key)
        if not content:
            continue
        h = hashlib.md5(content.encode()).hexdigest()
        hashes[key] = h
        (out_dir / f"{key}_{h}.yaml").write_text(content)

    # Also save a summary JSON
    summary = {"study": study, "trial": trial_num, "timestamp": ts, "hashes": hashes}
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2))

    return hashes


def load_trial(study: str, trial_num: int) -> dict:
    path = Path("logs/optuna_trials") / study / f"trial_{trial_num:04d}.json"
    if not path.exists():
        print(f"ERROR: {path} not found")
        sys.exit(1)
    with open(path) as f:
        return json.load(f)


def format_num(v):
    if v is None:
        return "-"
    if isinstance(v, int) or (isinstance(v, float) and v == int(v) and abs(v) < 1000):
        return str(int(v))
    if abs(v) >= 100:
        return f"{v:.0f}"
    elif abs(v) >= 10:
        return f"{v:.1f}"
    elif abs(v) >= 1:
        return f"{v:.2f}"
    else:
        return f"{v:.3f}"


def extract_result(raw: dict) -> dict:
    """Extract comparable metrics from raw nav2 test runner JSON.

    Raw JSON has: duration, waypoint_results[0].{error_xy, error_yaw},
    metrics.cmd_vel.{reversal_count, tv_linear, tv_angular},
    metrics.steering.{rms_rate, flip_hz}.

    Returns flat dict matching the trial JSON per_test format.
    """
    wp = {}
    if raw.get("waypoint_results"):
        wp = raw["waypoint_results"][-1]  # last (usually only) waypoint
    m = raw.get("metrics", {})
    cmd = m.get("cmd_vel", {})

    jitter = round(compute_jitter_score(m), 3) if m else 0

    return {
        "success": raw.get("success", False),
        "collision": raw.get("collision", False),
        "duration": raw.get("duration", 0),
        "accuracy_xy": wp.get("error_xy"),
        "accuracy_yaw": wp.get("error_yaw"),
        "jitter": jitter,
        "reversal_count": cmd.get("reversal_count", 0),
        "tv_linear": cmd.get("tv_linear"),
        "tv_angular": cmd.get("tv_angular"),
    }


def print_comparison(original: dict, results: list[dict], test_id: str):
    """Print side-by-side comparison of original vs replicated results."""
    orig_test = original.get("per_test", {}).get(test_id, {})

    fields = [
        "success", "duration", "accuracy_xy", "accuracy_yaw",
        "jitter", "reversal_count", "collision",
    ]

    # Show all cycles
    print(f"\n  {'Metric':<20s} {'Original':>10s}", end="")
    for i in range(len(results)):
        print(f" {'C'+str(i+1):>10s}", end="")
    print()
    print(f"  {'------':<20s} {'--------':>10s}", end="")
    for _ in results:
        print(f" {'-----':>10s}", end="")
    print()

    for key in fields:
        ov = orig_test.get(key)
        row = f"  {key:<20s} {format_num(ov):>10s}"
        for r in results:
            rv = r.get(key)
            row += f" {format_num(rv):>10s}"
        print(row)


def main():
    parser = argparse.ArgumentParser(
        description="Replicate a trial from a study",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("study", help="Study name")
    parser.add_argument("trial", type=int, help="Trial number")
    parser.add_argument("--test", help="Specific test ID (e.g. M4). Default: all tests in the trial")
    parser.add_argument("--cycles", type=int, default=1, help="Cycles per test (default: 1)")
    parser.add_argument("--warm-reset", action="store_true", help="Use warm reset between cycles")
    parser.add_argument("--no-amcl", action="store_true", help="Odometry-only (no AMCL)")
    parser.add_argument("--params-file", help="Tuning params YAML (to also set locked params on fresh stack)")
    parser.add_argument("--dry-run", action="store_true", help="Print params without running")
    args = parser.parse_args()

    # Load trial
    trial = load_trial(args.study, args.trial)
    sampled_params = trial["params"]
    # trial_values has ALL params (sampled + locked) — use for applying
    all_trial_params = trial.get("trial_values", sampled_params)
    per_test = trial.get("per_test", {})

    print(f"Study: {args.study}, Trial: {args.trial}")
    print(f"Original: passed={trial.get('tests_passed')}/{trial.get('tests_passed',0)+trial.get('tests_failed',0)}, "
          f"collision={'YES' if trial.get('any_collision') else 'no'}")
    if trial.get("objectives"):
        obj_names = ["time", "xy", "yaw", "jitter", "rev", "coll"]
        obj_str = ", ".join(f"{n}={format_num(v)}" for n, v in zip(obj_names, trial["objectives"]))
        print(f"Objectives: {obj_str}")

    # Determine tests to run
    if args.test:
        test_ids = [args.test.upper()]
    else:
        test_ids = list(per_test.keys())

    for tid in test_ids:
        if tid not in ALL_TESTS:
            print(f"ERROR: Unknown test '{tid}'. Available: {','.join(ALL_TESTS.keys())}")
            sys.exit(1)

    # Print params
    print(f"\nSampled params ({len(sampled_params)}):")
    for k, v in sorted(sampled_params.items()):
        print(f"  {k}: {format_num(v)}")
    locked_in_trial = {k: v for k, v in all_trial_params.items() if k not in sampled_params}
    if locked_in_trial:
        print(f"Locked params ({len(locked_in_trial)}):")
        for k, v in sorted(locked_in_trial.items()):
            print(f"  {k}: {format_num(v)}")

    if args.dry_run:
        print("\n--dry-run: not applying params or running tests")
        return

    # Build full param set to apply — use all_trial_params (sampled + locked)
    apply_params = dict(all_trial_params)

    # If params-file provided, also include any locked params not in trial_values
    all_param_specs = None
    if args.params_file:
        all_param_specs = load_tuning_params(max_tier=99, path=Path(args.params_file))
        for pname, spec in all_param_specs.items():
            if "locked" in spec and pname not in apply_params:
                apply_params[pname] = spec["locked"]
    else:
        # Build specs from all trial params — all MPPI params are on controller_server
        all_param_specs = {}
        for pname, val in all_trial_params.items():
            all_param_specs[pname] = {
                "ros_path": f"controller_server:FollowPath.{pname}",
                "type": "int" if isinstance(val, int) else "float",
            }

    # Apply params
    print(f"\nApplying {len(apply_params)} params...")
    if not set_all_params(apply_params, all_param_specs):
        print("ERROR: Failed to set params on running stack")
        print("Is the stack running? Try: python stack.py start jetacker:nav2")
        sys.exit(1)

    # Dump, verify, save, and compare hashes
    dumps = dump_all_params()
    if dumps.get("controller"):
        mismatches = verify_trial_params(dumps, apply_params, all_param_specs)
        if mismatches:
            print(f"WARNING: {len(mismatches)} param mismatches after setting:")
            for m in mismatches:
                print(f"  {m}")
        else:
            print("Params verified OK")

        # Save dumps and compare hashes
        repl_hashes = save_replication_dumps(dumps, args.study, args.trial)
        orig_hashes = trial.get("param_hashes", {})

        print(f"\nParam dump hashes:")
        print(f"  {'node':<12s} {'original':>34s} {'replication':>34s} {'match':>6s}")
        print(f"  {'----':<12s} {'--------':>34s} {'-----------':>34s} {'-----':>6s}")
        for key in DUMP_NODE_KEYS:
            oh = orig_hashes.get(key, "-")
            rh = repl_hashes.get(key, "-")
            match = "YES" if oh == rh else "NO"
            print(f"  {key:<12s} {oh:>34s} {rh:>34s} {match:>6s}")
    else:
        print("WARNING: Could not dump params for verification")

    # Run tests
    # Import here to avoid circular imports and keep startup fast
    from drive_nav2 import run_nav2_test, run_warm_reset, run_nav2_sim_reset

    print(f"\nRunning {len(test_ids)} test(s), {args.cycles} cycle(s) each"
          f"{', warm-reset' if args.warm_reset else ''}...")

    for tid in test_ids:
        waypoint_file, timeout, _weight = ALL_TESTS[tid]
        print(f"\n{'='*60}")
        print(f"Test {tid}: {waypoint_file} (timeout={timeout}s)")
        print(f"{'='*60}")

        cycle_results = []
        for cycle in range(1, args.cycles + 1):
            if args.cycles > 1:
                print(f"\n--- Cycle {cycle}/{args.cycles} ---")

            # Reset
            if cycle > 1 or args.warm_reset:
                if args.warm_reset:
                    ok, _ = run_warm_reset(quiet=True, cycle_num=cycle)
                else:
                    ok, _ = run_nav2_sim_reset(quiet=True, use_amcl=not args.no_amcl)
                if not ok:
                    print(f"  Reset failed, skipping cycle {cycle}")
                    cycle_results.append({"success": False, "reset_failed": True})
                    continue

            # Run
            success, data = run_nav2_test(
                waypoints_file=waypoint_file,
                timeout=timeout,
                quiet=False,
                use_amcl=not args.no_amcl,
            )
            extracted = extract_result(data)
            cycle_results.append(extracted)
            status = "PASS" if success else "FAIL"
            print(f"  Result: {status} (time={extracted['duration']:.1f}s, "
                  f"xy={format_num(extracted['accuracy_xy'])}, "
                  f"yaw={format_num(extracted['accuracy_yaw'])}, "
                  f"jit={format_num(extracted['jitter'])}, "
                  f"rev={extracted['reversal_count']})")

        # Summary for this test
        passed = sum(1 for r in cycle_results if r.get("success"))
        print(f"\n{tid} summary: {passed}/{len(cycle_results)} cycles passed")

        # Compare against original
        if tid in per_test and cycle_results:
            print_comparison(trial, cycle_results, tid)

    print(f"\nDone.")


if __name__ == "__main__":
    main()
