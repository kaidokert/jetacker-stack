#!/usr/bin/env python3
"""Mid-study parameter analysis for Optuna trials.

Reads trial JSONs directly (no Optuna dependency), filters into quality tiers,
computes Spearman importance, and generates ranked parameter tables.
Saves snapshots for delta comparison across runs.

Usage:
    python tools/study_analysis.py pareto_3critic_M4_v4 \
        --params-file config/tuning_params_pareto_v2.yaml

    # Custom filters
    python tools/study_analysis.py pareto_3critic_M4_v4 \
        --params-file config/tuning_params_pareto_v2.yaml \
        --good-time 30 --good-yaw 0.75 --good-rev 100 \
        --exc-time 10 --exc-yaw 0.2 --exc-rev 10
"""

import argparse
import io
import json
import math
import sys
from datetime import datetime
from pathlib import Path

# Allow importing tune_common from project root
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tune_common import load_tuning_params


# Objective indices for pareto studies
OBJ_TIME = 0
OBJ_XY = 1
OBJ_YAW = 2
OBJ_JITTER = 3
OBJ_REVERSALS = 4
OBJ_COLLISIONS = 5

OBJ_NAMES = ["time", "error_xy", "error_yaw", "jitter", "reversals", "collisions",
             "consistency"]


def load_trials(study_name: str) -> list[dict]:
    """Load all trial JSONs for a study, filtering to those with objectives."""
    trial_dir = Path("logs/optuna_trials") / study_name
    if not trial_dir.exists():
        print(f"ERROR: Trial directory not found: {trial_dir}")
        sys.exit(1)

    trials = []
    for path in sorted(trial_dir.glob("trial_*.json")):
        with open(path) as f:
            trial = json.load(f)
        if "objectives" in trial and trial["objectives"] is not None:
            trials.append(trial)
    return trials


def filter_trials(trials: list[dict], good_time: float, good_yaw: float,
                  good_rev: float, exc_time: float, exc_yaw: float,
                  exc_rev: float) -> tuple[list[dict], list[dict]]:
    """Filter trials into good and excellent tiers."""
    good, excellent = [], []
    for t in trials:
        obj = t["objectives"]
        # Skip trials with collisions
        if obj[OBJ_COLLISIONS] > 0:
            continue
        if obj[OBJ_TIME] < good_time and obj[OBJ_YAW] < good_yaw and obj[OBJ_REVERSALS] < good_rev:
            good.append(t)
        if obj[OBJ_TIME] < exc_time and obj[OBJ_YAW] < exc_yaw and obj[OBJ_REVERSALS] < exc_rev:
            excellent.append(t)
    return good, excellent


def percentile(values: list[float], p: float) -> float:
    """Compute percentile using linear interpolation."""
    if not values:
        return float("nan")
    s = sorted(values)
    n = len(s)
    if n == 1:
        return s[0]
    k = (n - 1) * p / 100.0
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return s[int(k)]
    return s[f] * (c - k) + s[c] * (k - f)


def compute_stats(values: list[float]) -> dict:
    """Compute min/p25/med/p75/max."""
    if not values:
        return {"min": None, "p25": None, "med": None, "p75": None, "max": None}
    return {
        "min": min(values),
        "p25": percentile(values, 25),
        "med": percentile(values, 50),
        "p75": percentile(values, 75),
        "max": max(values),
    }


def rank_array(arr: list[float]) -> list[float]:
    """Compute ranks (1-based, averaged for ties)."""
    n = len(arr)
    indexed = sorted(range(n), key=lambda i: arr[i])
    ranks = [0.0] * n
    i = 0
    while i < n:
        j = i
        while j < n - 1 and arr[indexed[j + 1]] == arr[indexed[j]]:
            j += 1
        avg_rank = (i + j) / 2.0 + 1
        for k in range(i, j + 1):
            ranks[indexed[k]] = avg_rank
        i = j + 1
    return ranks


def spearman_r(x: list[float], y: list[float]) -> float:
    """Spearman rank correlation between x and y."""
    if len(x) < 3:
        return 0.0
    rx = rank_array(x)
    ry = rank_array(y)
    n = len(rx)
    mean_rx = sum(rx) / n
    mean_ry = sum(ry) / n
    cov = sum((rx[i] - mean_rx) * (ry[i] - mean_ry) for i in range(n))
    std_x = math.sqrt(sum((rx[i] - mean_rx) ** 2 for i in range(n)))
    std_y = math.sqrt(sum((ry[i] - mean_ry) ** 2 for i in range(n)))
    if std_x == 0 or std_y == 0:
        return 0.0
    return cov / (std_x * std_y)


def compute_importance(trials: list[dict], param_names: list[str]) -> tuple[dict[str, float], dict[str, list[float]], list[str]]:
    """Compute importance = max |Spearman r| across all objectives per param.

    Returns (importance_dict, matrix_dict, active_obj_names) where matrix_dict
    maps param -> list of |r| per objective (only for objectives present in data).
    """
    if len(trials) < 3:
        zeros = [0.0] * len(OBJ_NAMES)
        return {p: 0.0 for p in param_names}, {p: list(zeros) for p in param_names}, list(OBJ_NAMES)

    # Handle variable-length objectives (old trials may have fewer)
    min_obj_len = min(len(t["objectives"]) for t in trials)
    active_obj_names = OBJ_NAMES[:min_obj_len]
    objectives = [[t["objectives"][i] for t in trials] for i in range(min_obj_len)]
    importance = {}
    matrix = {}
    for pname in param_names:
        param_values = [t["params"][pname] for t in trials]
        row = []
        for obj_values in objectives:
            row.append(round(abs(spearman_r(param_values, obj_values)), 3))
        matrix[pname] = row
        importance[pname] = max(row)
    return importance, matrix, active_obj_names


def format_stats(stats: dict, width: int = 4) -> str:
    """Format stats dict as compact string."""
    if stats["min"] is None:
        return "-"
    vals = [stats["min"], stats["p25"], stats["med"], stats["p75"], stats["max"]]
    parts = []
    for v in vals:
        if abs(v) >= 100:
            parts.append(f"{v:.0f}")
        elif abs(v) >= 10:
            parts.append(f"{v:.1f}")
        elif abs(v) >= 1:
            parts.append(f"{v:.2f}")
        else:
            parts.append(f"{v:.3f}")
    return "/".join(parts)


def format_num(v: float) -> str:
    """Format a single number compactly."""
    if abs(v) >= 100:
        return f"{v:.0f}"
    elif abs(v) >= 10:
        return f"{v:.1f}"
    elif abs(v) >= 1:
        return f"{v:.2f}"
    else:
        return f"{v:.3f}"


def save_snapshot(study_name: str, data: dict) -> Path:
    """Save analysis snapshot to logs/study_snapshots/<study>/<timestamp>.json."""
    snap_dir = Path("logs/study_snapshots") / study_name
    snap_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = snap_dir / f"{ts}.json"
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    return path


def load_latest_snapshot(study_name: str) -> dict | None:
    """Load the most recent snapshot for comparison."""
    snap_dir = Path("logs/study_snapshots") / study_name
    if not snap_dir.exists():
        return None
    files = sorted(snap_dir.glob("*.json"))
    if not files:
        return None
    with open(files[-1]) as f:
        return json.load(f)


def print_delta(prev: dict, curr: dict):
    """Print delta between two snapshots."""
    prev_params = prev["params"]
    curr_params = curr["params"]

    print(f"\n## Delta vs previous snapshot ({prev['timestamp']})")
    print(f"Trials: {prev['total_trials']} -> {curr['total_trials']} "
          f"(+{curr['total_trials'] - prev['total_trials']})")
    print(f"Good: {prev['good_n']} -> {curr['good_n']}")
    print(f"Excellent: {prev['excellent_n']} -> {curr['excellent_n']}")

    # Check for meaningful changes in excellent tier
    changes = []
    for pname in curr_params:
        if pname not in prev_params:
            continue
        ce = curr_params[pname].get("excellent", {})
        pe = prev_params[pname].get("excellent", {})
        if ce.get("med") is None or pe.get("med") is None:
            continue
        c_med = ce["med"]
        p_med = pe["med"]
        if p_med == 0:
            continue
        pct = (c_med - p_med) / abs(p_med) * 100
        if abs(pct) > 1:
            arrow = "UP" if pct > 0 else "DN"
            changes.append((pname, p_med, c_med, pct, arrow))

    if changes:
        print(f"\nExcellent tier median shifts:")
        for pname, p_med, c_med, pct, arrow in sorted(changes, key=lambda x: -abs(x[3])):
            print(f"  {pname}: {format_num(p_med)} -> {format_num(c_med)} ({arrow} {abs(pct):.1f}%)")
    else:
        print("\nNo significant shifts in excellent medians.")



def main():
    parser = argparse.ArgumentParser(description="Mid-study parameter analysis")
    parser.add_argument("study", help="Study name (directory under logs/optuna_trials/)")
    parser.add_argument("--params-file", required=True, help="Tuning params YAML")
    parser.add_argument("--output", help="Output markdown file (default: studies/<study>_analysis.md)")
    parser.add_argument("--good-time", type=float, default=30.0)
    parser.add_argument("--good-yaw", type=float, default=0.75)
    parser.add_argument("--good-rev", type=float, default=100.0)
    parser.add_argument("--exc-time", type=float, default=10.0)
    parser.add_argument("--exc-yaw", type=float, default=0.2)
    parser.add_argument("--exc-rev", type=float, default=10.0)
    parser.add_argument("--no-snapshot", action="store_true", help="Skip saving snapshot")
    args = parser.parse_args()

    # Tee output to both stdout and markdown file
    out_path = Path(args.output) if args.output else Path("studies") / f"{args.study}_analysis.md"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    buf = io.StringIO()

    def out(line=""):
        print(line)
        buf.write(line + "\n")

    # Load tuning params
    tuning_params = load_tuning_params(max_tier=99, path=Path(args.params_file))
    param_names = sorted(tuning_params.keys())

    # Load trials
    trials = load_trials(args.study)
    out(f"# Study: {args.study}")
    out(f"Total trials with objectives: {len(trials)}")

    # Filter
    good, excellent = filter_trials(
        trials, args.good_time, args.good_yaw, args.good_rev,
        args.exc_time, args.exc_yaw, args.exc_rev,
    )
    out(f"Good (time<{args.good_time}, yaw<{args.good_yaw}, rev<{args.good_rev}, no collision): {len(good)}")
    out(f"Excellent (time<{args.exc_time}, yaw<{args.exc_yaw}, rev<{args.exc_rev}, no collision): {len(excellent)}")

    # Filter param_names to only those actually present in trial data
    if trials:
        trial_params = set(trials[0]["params"].keys())
        param_names = [p for p in param_names if p in trial_params]

    # Compute importance on good trials
    imp_trials = good if len(good) >= 10 else trials
    importance, imp_matrix, matrix_obj_names = compute_importance(imp_trials, param_names)

    # Build per-param analysis
    param_analysis = {}
    for pname in param_names:
        all_vals = [t["params"][pname] for t in trials]
        good_vals = [t["params"][pname] for t in good]
        exc_vals = [t["params"][pname] for t in excellent]
        spec = tuning_params.get(pname, {})

        param_analysis[pname] = {
            "importance": importance[pname],
            "sampled": {"min": min(all_vals), "max": max(all_vals)} if all_vals else {},
            "good": compute_stats(good_vals),
            "excellent": compute_stats(exc_vals),
            "study_range": spec.get("range", [None, None]),
        }

    # Sort by importance descending
    sorted_params = sorted(param_names, key=lambda p: -importance[p])

    # Print parameter table
    out(f"\n## Parameter Analysis")
    out(f"| # | Parameter | Imp | Sampled | Good (n={len(good)}) | Excellent (n={len(excellent)}) | Study Range |")
    out(f"|---|-----------|-----|---------|{'-' * (len(str(len(good))) + 10)}|{'-' * (len(str(len(excellent))) + 14)}|-------------|")
    for i, pname in enumerate(sorted_params, 1):
        pa = param_analysis[pname]
        samp = f"{format_num(pa['sampled']['min'])}..{format_num(pa['sampled']['max'])}" if pa["sampled"] else "-"
        good_s = format_stats(pa["good"])
        exc_s = format_stats(pa["excellent"])
        rng = pa["study_range"]
        rng_s = f"[{format_num(rng[0])}, {format_num(rng[1])}]" if rng[0] is not None else "-"
        out(f"| {i} | {pname} | {pa['importance']:.3f} | {samp} | {good_s} | {exc_s} | {rng_s} |")

    # Print importance matrix (param x objective)
    out(f"\n## Spearman Importance Matrix (|r|, n={len(imp_trials)} {'good' if len(good) >= 10 else 'all'} trials)")
    hdr = "| # | Parameter | " + " | ".join(matrix_obj_names) + " |"
    sep = "|---|-----------|" + "|".join("------:" for _ in matrix_obj_names) + "|"
    out(hdr)
    out(sep)
    for i, pname in enumerate(sorted_params, 1):
        row = imp_matrix[pname]
        cells = " | ".join(f"**{v:.3f}**" if v == max(row) else f"{v:.3f}" for v in row)
        out(f"| {i} | {pname} | {cells} |")

    # Boundary analysis
    out("\n## Boundary Analysis (excellent tier within 10% of study range)")
    flags = []
    for pname, pa in param_analysis.items():
        exc = pa.get("excellent", {})
        if exc.get("min") is None:
            continue
        spec = tuning_params.get(pname, {})
        lo, hi = spec.get("range", [None, None])
        if lo is None:
            continue
        scale = spec.get("scale", "linear")
        if scale == "log":
            log_lo, log_hi = math.log(lo), math.log(hi)
            span = log_hi - log_lo
            threshold = span * 0.10
            at_floor = (math.log(exc["min"]) - log_lo) < threshold
            at_ceil = (log_hi - math.log(exc["max"])) < threshold
        else:
            span = hi - lo
            threshold = span * 0.10
            at_floor = (exc["min"] - lo) < threshold
            at_ceil = (hi - exc["max"]) < threshold
        if at_floor or at_ceil:
            flags.append((pname, lo, hi, exc["min"], exc["max"],
                          "FLOOR" if at_floor else "", "CEILING" if at_ceil else ""))

    if flags:
        for pname, lo, hi, e_min, e_max, floor_flag, ceil_flag in flags:
            markers = " ".join(filter(None, [floor_flag, ceil_flag]))
            out(f"  {pname}: excellent [{format_num(e_min)}, {format_num(e_max)}] "
                f"vs range [{format_num(lo)}, {format_num(hi)}] -- {markers}")
    else:
        out("  No parameters hitting study boundaries.")

    # Snapshot handling
    prev_snapshot = load_latest_snapshot(args.study)

    snapshot_data = {
        "timestamp": datetime.now().isoformat(),
        "total_trials": len(trials),
        "good_n": len(good),
        "excellent_n": len(excellent),
        "filters": {
            "good_time": args.good_time, "good_yaw": args.good_yaw, "good_rev": args.good_rev,
            "exc_time": args.exc_time, "exc_yaw": args.exc_yaw, "exc_rev": args.exc_rev,
        },
        "params": param_analysis,
    }

    if prev_snapshot:
        print_delta(prev_snapshot, snapshot_data)

    if not args.no_snapshot:
        path = save_snapshot(args.study, snapshot_data)
        out(f"\nSnapshot saved: {path}")

    # Write markdown file
    out_path.write_text(buf.getvalue(), encoding="utf-8")
    print(f"Output written to: {out_path}")


if __name__ == "__main__":
    main()
