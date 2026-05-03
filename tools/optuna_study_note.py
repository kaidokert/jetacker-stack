#!/usr/bin/env python3
"""Read or write Optuna dashboard study notes via the study system attributes.

The Optuna dashboard stores notes in study_system_attributes with keys:
  - dashboard:note_str:0  (the markdown body)
  - dashboard:note_ver    (integer version counter for optimistic locking)

Usage:
    # Read note for a study
    python tools/optuna_study_note.py pareto_3critic_M4_v2

    # Write note from stdin
    echo "# My Note" | python tools/optuna_study_note.py pareto_3critic_M4_v2 --set

    # Write note from file
    python tools/optuna_study_note.py pareto_3critic_M4_v2 --set --file notes/my_note.md

    # Auto-generate header (commit, date) and append custom body
    python tools/optuna_study_note.py pareto_3critic_M4_v2 --set --auto-header --file notes/my_note.md

    # List all studies with notes
    python tools/optuna_study_note.py --list
"""

import argparse
import os
import subprocess
import sys
import warnings
from pathlib import Path

# Suppress FutureWarning for set_system_attr (deprecated but still the only way)
warnings.filterwarnings('ignore', category=FutureWarning, module='optuna')

sys.path.insert(0, str(Path(__file__).parent.parent))


def get_storage():
    """Resolve Optuna storage URL."""
    # Try .env file
    env_path = Path(__file__).parent.parent / '.env'
    if env_path.is_file():
        with open(env_path) as f:
            for line in f:
                line = line.strip()
                if line.startswith('OPTUNA_STORAGE='):
                    return line.split('=', 1)[1]
    return os.environ.get('OPTUNA_STORAGE', '')


def get_git_info():
    """Get current git commit hash and branch."""
    try:
        commit = subprocess.run(
            ['git', 'log', '--oneline', '-1'],
            capture_output=True, text=True, timeout=5,
        ).stdout.strip()
        branch = subprocess.run(
            ['git', 'branch', '--show-current'],
            capture_output=True, text=True, timeout=5,
        ).stdout.strip()
        return commit, branch
    except Exception:
        return 'unknown', 'unknown'


def read_note(study):
    """Read current note from study."""
    attrs = study.system_attrs
    body = attrs.get('dashboard:note_str:0', '')
    version = attrs.get('dashboard:note_ver', 0)
    return body, version


def write_note(study, body):
    """Write note to study, incrementing version."""
    _, current_ver = read_note(study)
    new_ver = current_ver + 1 if current_ver else 1
    study.set_system_attr('dashboard:note_str:0', body)
    study.set_system_attr('dashboard:note_ver', new_ver)
    return new_ver


def list_studies_with_notes(storage):
    """List all studies that have notes."""
    import optuna
    optuna.logging.set_verbosity(optuna.logging.WARNING)

    summaries = optuna.study.get_all_study_summaries(storage=storage)
    for s in summaries:
        study = optuna.load_study(study_name=s.study_name, storage=storage)
        body, ver = read_note(study)
        n_complete = sum(1 for t in study.trials
                         if t.state == optuna.trial.TrialState.COMPLETE)
        has_note = 'Y' if body else '-'
        first_line = body.split('\n')[0][:60] if body else ''
        print(f'  [{has_note}] {s.study_name:<40s}  trials={n_complete:>4d}  {first_line}')


def main():
    parser = argparse.ArgumentParser(
        description='Read/write Optuna dashboard study notes',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('study_name', nargs='?', help='Study name')
    parser.add_argument('--set', action='store_true', help='Write note (reads from --file or stdin)')
    parser.add_argument('--file', type=str, help='Read note body from file')
    parser.add_argument('--auto-header', action='store_true',
                        help='Prepend auto-generated header (git commit, date, study name)')
    parser.add_argument('--list', action='store_true', help='List all studies with note status')
    parser.add_argument('--storage', type=str, default=None, help='Optuna storage URL')
    args = parser.parse_args()

    storage = args.storage or get_storage()
    if not storage:
        print('ERROR: No storage URL. Set OPTUNA_STORAGE or use --storage', file=sys.stderr)
        return 1

    import optuna
    optuna.logging.set_verbosity(optuna.logging.WARNING)

    if args.list:
        list_studies_with_notes(storage)
        return 0

    if not args.study_name:
        parser.error('study_name is required (unless using --list)')

    study = optuna.load_study(study_name=args.study_name, storage=storage)

    if args.set:
        # Read body
        if args.file:
            body = Path(args.file).read_text()
        else:
            body = sys.stdin.read()

        if args.auto_header:
            commit, branch = get_git_info()
            from datetime import datetime
            header = (
                f'# {args.study_name}\n\n'
                f'**Commit**: {commit} ({branch})\n'
                f'**Date**: {datetime.now().strftime("%Y-%m-%d %H:%M")}\n\n'
            )
            body = header + body

        ver = write_note(study, body)
        print(f'Note written (version {ver}, {len(body)} chars)')
    else:
        # Read
        body, ver = read_note(study)
        if body:
            sys.stdout.buffer.write(body.encode('utf-8'))
            sys.stdout.buffer.write(b'\n')
        else:
            print(f'(no note set for {args.study_name})', file=sys.stderr)

    return 0


if __name__ == '__main__':
    sys.exit(main())
