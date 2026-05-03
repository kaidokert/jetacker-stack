#!/usr/bin/env python3
"""Delete specific trials from an Optuna study via direct SQL.

Usage:
    python tools/nuke_trials.py 82 83 84 85 86
    python tools/nuke_trials.py 82-86
    python tools/nuke_trials.py 82-86 --study mppi_full_matrix_tier1_v3 --dry-run
"""

import argparse
import os
import sys
from pathlib import Path

# Load .env for OPTUNA_STORAGE
sys.path.insert(0, str(Path(__file__).parent.parent))
from tune_common import load_dotenv

load_dotenv()

import optuna


def parse_trial_range(args: list[str]) -> list[int]:
    """Parse trial numbers from args like ['82', '83'] or ['82-86']."""
    trials = []
    for arg in args:
        if '-' in arg:
            start, end = arg.split('-', 1)
            trials.extend(range(int(start), int(end) + 1))
        else:
            trials.append(int(arg))
    return sorted(set(trials))


def delete_trials_sql(storage_url: str, study_name: str, trial_numbers: list[int]):
    """Delete trials via direct SQL (works with both SQLite and PostgreSQL)."""
    from sqlalchemy import create_engine, text

    engine = create_engine(storage_url)
    with engine.begin() as conn:
        # Get study_id
        row = conn.execute(
            text("SELECT study_id FROM studies WHERE study_name = :name"),
            {"name": study_name}
        ).fetchone()
        if not row:
            print(f"ERROR: study '{study_name}' not found", file=sys.stderr)
            return False
        study_id = row[0]

        # Get trial_ids for the given trial numbers
        rows = conn.execute(
            text("SELECT trial_id, number FROM trials "
                 "WHERE study_id = :sid AND number = ANY(:nums)"),
            {"sid": study_id, "nums": trial_numbers}
        ).fetchall()

        if not rows:
            print("No matching trials found in DB.")
            return False

        trial_ids = [r[0] for r in rows]
        print(f"  Found {len(trial_ids)} trials in DB (trial_ids: {trial_ids})")

        # Delete in dependency order
        for table in ['trial_user_attributes', 'trial_system_attributes',
                       'trial_params', 'trial_values', 'trial_intermediate_values',
                       'trial_heartbeats']:
            result = conn.execute(
                text(f"DELETE FROM {table} WHERE trial_id = ANY(:ids)"),
                {"ids": trial_ids}
            )
            if result.rowcount:
                print(f"  {table}: deleted {result.rowcount} rows")

        result = conn.execute(
            text("DELETE FROM trials WHERE trial_id = ANY(:ids)"),
            {"ids": trial_ids}
        )
        print(f"  trials: deleted {result.rowcount} rows")

    return True


def main():
    parser = argparse.ArgumentParser(description='Delete trials from Optuna study')
    parser.add_argument('trials', nargs='+', help='Trial numbers (e.g. 82 83 84 or 82-86)')
    parser.add_argument('--study', default='mppi_full_matrix_tier1_v3',
                        help='Study name (default: mppi_full_matrix_tier1_v3)')
    parser.add_argument('--storage', default=None,
                        help='Storage URL (default: $OPTUNA_STORAGE from .env)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Show what would be deleted without deleting')
    args = parser.parse_args()

    storage = args.storage or os.environ.get('OPTUNA_STORAGE')
    if not storage:
        print("ERROR: No storage URL. Set OPTUNA_STORAGE in .env or pass --storage",
              file=sys.stderr)
        return 1

    trial_numbers = parse_trial_range(args.trials)
    print(f"Study: {args.study}")
    print(f"Storage: {storage}")
    print(f"Trials to delete: {trial_numbers}")
    print()

    # Show what we're about to delete
    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = optuna.load_study(study_name=args.study, storage=storage)

    found = []
    for t in study.trials:
        if t.number in trial_numbers:
            per_test = t.user_attrs.get('per_test', {})
            tests_str = ','.join(per_test.keys()) if per_test else '(no user_attrs)'
            try:
                score_str = f"score={t.value}"
            except RuntimeError:
                score_str = f"values={t.values}"
            print(f"  Trial {t.number}: state={t.state.name}, "
                  f"{score_str}, tests=[{tests_str}]")
            found.append(t.number)

    missing = set(trial_numbers) - set(found)
    if missing:
        print(f"\n  WARNING: trials not found: {sorted(missing)}")

    if not found:
        print("\nNothing to delete.")
        return 0

    if args.dry_run:
        print(f"\nDRY RUN: would delete {len(found)} trials: {found}")
        return 0

    print(f"\nDeleting {len(found)} trials via SQL...")
    if delete_trials_sql(storage, args.study, found):
        print("\nDone. Verify with: python tools/nuke_trials.py <same numbers> --dry-run")
    else:
        print("\nFailed.", file=sys.stderr)
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
