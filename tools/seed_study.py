#!/usr/bin/env python3
"""Seed an Optuna study with trial configs, validated against the params YAML.

Reads the tuning params YAML to get ranges and types, then validates and
clamps every seed value before enqueuing. No more out-of-range crashes.

Usage:
    # Seed from a CSV of previous study results:
    python tools/seed_study.py \
        --study mppi_pareto_pfc_pac_m7_v15 \
        --params-file config/tuning_params_pareto_v15.yaml \
        --csv mppi_pareto_pfc_m7_v4_fin.csv \
        --csv-map 'PathFollowCritic.cost_weight=Param PathFollowCritic.cost_weight' \
        --fill 'PathAlignCritic.cost_weight=30.0' \
        --fill 'min_inversion_horizon=0.40'

    # Seed from a JSON file of explicit configs:
    python tools/seed_study.py \
        --study mppi_pareto_pfc_pac_m7_v15 \
        --params-file config/tuning_params_pareto_v15.yaml \
        --json seeds.json

    # Seed perturbations of a base config:
    python tools/seed_study.py \
        --study mppi_pareto_pfc_pac_m7_v15 \
        --params-file config/tuning_params_pareto_v15.yaml \
        --base '{"PathFollowCritic.cost_weight": 117.9, ...}' \
        --perturb 10 --perturb-pct 0.08
"""

import argparse
import csv
import json
import math
import os
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
from tune_common import load_dotenv, load_tuning_params, get_samplable_params

load_dotenv()
import optuna


def clamp_and_validate(values: dict, all_params: dict) -> dict:
    """Validate and clamp seed values against the params YAML spec.

    - Clamps numeric values to [range_min, range_max]
    - Coerces types (int↔float)
    - Skips locked params (they're not sampled)
    - Warns on missing params (fills with baseline)
    - Errors on unknown params

    Returns cleaned dict ready for study.enqueue_trial().
    """
    samplable = get_samplable_params(all_params)
    cleaned = {}

    for name, spec in samplable.items():
        if name in values:
            val = values[name]
        elif 'baseline' in spec:
            val = spec['baseline']
        else:
            print(f"  WARN: no value or baseline for {name}, skipping",
                  file=sys.stderr)
            continue

        param_type = spec.get('type', 'float')
        param_range = spec.get('range')

        # Type coercion
        if param_type == 'float':
            val = float(val)
        elif param_type == 'int':
            val = int(round(float(val)))
        elif param_type == 'bool':
            val = bool(val)

        # Range clamping
        if param_range and param_type in ('float', 'int'):
            lo, hi = param_range
            old_val = val
            val = max(lo, min(hi, val))
            if val != old_val:
                print(f"  CLAMP: {name}: {old_val} -> {val} "
                      f"(range [{lo}, {hi}])", file=sys.stderr)

        cleaned[name] = val

    # Warn on unknown keys
    for name in values:
        if name not in samplable:
            print(f"  SKIP: {name} not in samplable params", file=sys.stderr)

    return cleaned


def perturb_config(base: dict, all_params: dict, pct: float) -> dict:
    """Create a random perturbation of base config within ±pct."""
    samplable = get_samplable_params(all_params)
    perturbed = {}
    for name, val in base.items():
        spec = samplable.get(name, {})
        param_type = spec.get('type', 'float')
        if param_type == 'int':
            delta = max(1, round(val * pct))
            perturbed[name] = val + random.randint(-delta, delta)
        elif param_type == 'float':
            perturbed[name] = round(val * (1 + random.uniform(-pct, pct)), 6)
        else:
            perturbed[name] = val
    return clamp_and_validate(perturbed, all_params)


def seeds_from_csv(csv_path: str, col_map: dict, fill: dict,
                   all_params: dict) -> list[dict]:
    """Read seeds from a CSV file with column mapping and fill values."""
    samplable = get_samplable_params(all_params)
    seeds = []
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            values = dict(fill)  # start with fill defaults
            for param_name, csv_col in col_map.items():
                if csv_col in row:
                    values[param_name] = float(row[csv_col])
            seeds.append(clamp_and_validate(values, all_params))
    return seeds


def main():
    parser = argparse.ArgumentParser(
        description='Seed an Optuna study with validated trial configs')
    parser.add_argument('--study', required=True, help='Optuna study name')
    parser.add_argument('--params-file', required=True,
                        help='Tuning params YAML (for validation/ranges)')
    parser.add_argument('--storage', default=None,
                        help='Optuna storage URL (default: $OPTUNA_STORAGE)')
    parser.add_argument('--csv', action='append', default=[],
                        help='CSV file to seed from (repeatable)')
    parser.add_argument('--csv-map', action='append', default=[],
                        help='Column mapping: param_name=csv_column (repeatable)')
    parser.add_argument('--fill', action='append', default=[],
                        help='Fill value for params not in CSV: name=value')
    parser.add_argument('--json', default=None,
                        help='JSON file with list of seed dicts')
    parser.add_argument('--base', default=None,
                        help='JSON string of base config for perturbations')
    parser.add_argument('--perturb', type=int, default=0,
                        help='Number of perturbations to generate from --base')
    parser.add_argument('--perturb-pct', type=float, default=0.08,
                        help='Perturbation percentage (default: 0.08 = ±8%%)')
    parser.add_argument('--seed', type=int, default=42,
                        help='Random seed for perturbations')
    parser.add_argument('--dry-run', action='store_true',
                        help='Print seeds without enqueuing')
    args = parser.parse_args()

    # Load param spec
    all_params = load_tuning_params(
        max_tier=1, path=Path(args.params_file))
    if not all_params:
        print(f"ERROR: No params loaded from {args.params_file}",
              file=sys.stderr)
        return 1

    samplable = get_samplable_params(all_params)
    print(f"Params: {len(samplable)} samplable from {args.params_file}")

    # Parse fill values
    fill = {}
    for f in args.fill:
        k, v = f.split('=', 1)
        fill[k] = float(v) if '.' in v else int(v)

    # Parse column mapping
    col_map = {}
    for m in args.csv_map:
        k, v = m.split('=', 1)
        col_map[k] = v

    # Collect all seeds
    all_seeds = []

    for csv_path in args.csv:
        seeds = seeds_from_csv(csv_path, col_map, fill, all_params)
        print(f"CSV {csv_path}: {len(seeds)} seeds")
        all_seeds.extend(seeds)

    if args.json:
        with open(args.json) as f:
            json_seeds = json.load(f)
        for s in json_seeds:
            all_seeds.append(clamp_and_validate(s, all_params))
        print(f"JSON: {len(json_seeds)} seeds")

    if args.base:
        base = json.loads(args.base)
        base_clean = clamp_and_validate(base, all_params)
        all_seeds.append(base_clean)
        print(f"Base config: 1 seed")

        if args.perturb > 0:
            random.seed(args.seed)
            for i in range(args.perturb):
                all_seeds.append(perturb_config(base_clean, all_params,
                                                args.perturb_pct))
            print(f"Perturbations: {args.perturb} seeds (±{args.perturb_pct*100:.0f}%)")

    if not all_seeds:
        print("No seeds to enqueue. Use --csv, --json, or --base.")
        return 1

    print(f"\nTotal: {len(all_seeds)} seeds")

    if args.dry_run:
        for i, s in enumerate(all_seeds):
            print(f"  [{i}] {s}")
        return 0

    # Enqueue
    storage = args.storage or os.environ.get('OPTUNA_STORAGE')
    if not storage:
        print("ERROR: No storage URL. Set --storage or $OPTUNA_STORAGE",
              file=sys.stderr)
        return 1

    study = optuna.load_study(study_name=args.study, storage=storage)
    print(f"Study {args.study}: {len(study.trials)} trials before seeding")

    for s in all_seeds:
        study.enqueue_trial(s)

    print(f"Enqueued {len(all_seeds)} seeds. "
          f"Study now has {len(study.trials)} trials.")
    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
