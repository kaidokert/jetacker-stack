#!/usr/bin/env python3
"""
Shared infrastructure for Optuna MPPI tuning harnesses.

Provides parameter loading, setting (batched docker exec), stack health checks,
test execution, scoring helpers, and Optuna study management. Used by both
tune_mppi.py (shotgun/composite) and tune_mppi_focused.py (laser/multi-objective).
"""

import atexit
import datetime
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / 'tools'))
from subprocess_utils import run as _run, docker_exec as _docker_exec

import yaml

try:
    import optuna
except ImportError:
    print("ERROR: optuna not installed. Run: pip install optuna", file=sys.stderr)
    sys.exit(1)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

TUNING_PARAMS_PATH = Path(__file__).parent / 'config' / 'tuning_params.yaml'

# Key containers to check for stack health (service name substrings)
HEALTH_CHECK_CONTAINERS = [
    'jetacker-ekf',
    'jetacker-controller-server',
    'jetacker-planner-server',
    'jetacker-bt-navigator',
    'test-drive',
]

PARAM_SET_PENALTY = 50000.0  # Failed to set params — something very wrong

# Test suite: short_name -> (waypoint_file, timeout_seconds, weight)
ALL_TESTS = {
    'M1': ('nav2_matrix_1_forward_straight', 30, 1.0),
    'M2': ('nav2_matrix_2_reverse_straight', 30, 1.0),
    'M3': ('nav2_matrix_3_forward_left_90', 50, 1.5),
    'M4': ('nav2_matrix_4_forward_right_90', 60, 1.5),
    'M5': ('nav2_matrix_5_reverse_left_90', 90, 1.5),
    'M6': ('nav2_matrix_6_reverse_right_90', 45, 1.5),
    'M7': ('nav2_matrix_7_forward_180', 75, 2.0),
    'M8': ('nav2_matrix_8_lateral_shift_right', 90, 1.0),
}
DEFAULT_TESTS = ['M1', 'M2', 'M4', 'M6']


# ---------------------------------------------------------------------------
# Tuning lock — prevent concurrent tuning harnesses on the same robot stack
# ---------------------------------------------------------------------------
# Windows: kernel mutex via win32event. Auto-released on process death (even
# SIGKILL / crash / orphaned nohup children). This is the ONLY reliable lock
# on Windows — PID-based lockfiles fail when atexit never fires.
# Non-Windows: no-op (lockfile was unreliable, not worth the complexity).
# ---------------------------------------------------------------------------

LOCK_PATH = Path(__file__).parent / 'logs' / '.tune_mppi.lock'

try:
    import win32event  # type: ignore[import-untyped]
    import win32api    # type: ignore[import-untyped]
    _HAS_WIN32 = True
except ImportError:
    _HAS_WIN32 = False

_win_mutex = None  # handle kept alive for process lifetime


def _release_win_mutex():
    """Release Windows mutex + close handle. Safe to call multiple times."""
    global _win_mutex
    if _win_mutex is not None:
        try:
            win32event.ReleaseMutex(_win_mutex)
        except Exception:
            pass
        try:
            win32api.CloseHandle(_win_mutex)
        except Exception:
            pass
        _win_mutex = None

    # Clean up info lockfile
    try:
        if LOCK_PATH.exists():
            data = json.loads(LOCK_PATH.read_text())
            if data.get('pid') == os.getpid():
                LOCK_PATH.unlink()
    except Exception:
        pass


def acquire_tuning_lock(study_name: str) -> None:
    """Acquire exclusive lock for tuning. Exits if another harness is running.

    On Windows: uses a kernel mutex (Global\\tune_mppi_lock). The kernel
    auto-releases the mutex when the owning process exits — no stale locks,
    no orphan zombie issues.

    On non-Windows: no-op (prints warning).
    """
    global _win_mutex

    if not _HAS_WIN32:
        print("  [lock] Skipping tuning lock (win32 not available)",
              file=sys.stderr)
        return

    mutex_name = "Global\\tune_mppi_lock"
    _win_mutex = win32event.CreateMutex(None, False, mutex_name)
    if not _win_mutex:
        print(f"ERROR: Failed to create Windows mutex '{mutex_name}'",
              file=sys.stderr)
        sys.exit(1)

    # Non-blocking acquire: WAIT_OBJECT_0 = acquired, WAIT_ABANDONED = previous
    # owner died (we still get ownership), WAIT_TIMEOUT = someone else has it.
    result = win32event.WaitForSingleObject(_win_mutex, 0)

    if result == win32event.WAIT_OBJECT_0:
        pass  # clean acquisition
    elif result == win32event.WAIT_ABANDONED:
        print(f"  [lock] Acquired abandoned mutex (previous owner crashed)",
              file=sys.stderr)
    else:
        # WAIT_TIMEOUT — another process holds the mutex
        win32api.CloseHandle(_win_mutex)
        _win_mutex = None

        # Try to read info lockfile for a helpful error
        owner_info = ""
        if LOCK_PATH.exists():
            try:
                data = json.loads(LOCK_PATH.read_text())
                owner_info = (f"\n  Study: {data.get('study')}"
                              f"\n  PID: {data.get('pid')}"
                              f"\n  Started: {data.get('started')}")
            except Exception:
                pass

        print(f"ERROR: Another tuning harness is already running!"
              f"{owner_info}\n"
              f"\n  The Windows mutex '{mutex_name}' is held.\n"
              f"  Kill the other process, then retry.",
              file=sys.stderr)
        sys.exit(1)

    # Write info lockfile for human visibility (which study, when, PID)
    LOCK_PATH.parent.mkdir(parents=True, exist_ok=True)
    LOCK_PATH.write_text(json.dumps({
        'pid': os.getpid(),
        'study': study_name,
        'started': datetime.datetime.now().isoformat(),
    }))

    # atexit cleans up the info lockfile. The kernel mutex auto-releases on
    # process death (even SIGKILL) — that's the whole point of using it.
    atexit.register(_release_win_mutex)


def release_tuning_lock() -> None:
    """Release the tuning lock. Safe to call multiple times."""
    _release_win_mutex()


# ---------------------------------------------------------------------------
# Dotenv
# ---------------------------------------------------------------------------

def load_dotenv(env_path: Path = None) -> None:
    """Load KEY=VALUE lines from .env into os.environ (skip comments, empty).

    Does not override already-set env vars. Call explicitly at runner startup.
    """
    if env_path is None:
        env_path = Path(__file__).parent / '.env'
    if not env_path.is_file():
        return
    with open(env_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or '=' not in line:
                continue
            key, _, val = line.partition('=')
            key = key.strip()
            val = val.strip()
            if key and key not in os.environ:
                os.environ[key] = val


# ---------------------------------------------------------------------------
# YAML parameter loading
# ---------------------------------------------------------------------------

def load_tuning_params(max_tier: int = 1, path: Path = None) -> dict:
    """Load params from YAML, filtered to tier 1..max_tier.

    Args:
        max_tier: Include params with tier <= max_tier.
        path: Path to tuning params YAML. Defaults to config/tuning_params.yaml.

    Returns dict of {name: spec} where spec has keys:
        ros_path, baseline, range, type, scale, tier, and optionally locked.
    """
    p = path or TUNING_PARAMS_PATH
    with open(p) as f:
        raw = yaml.safe_load(f)
    return {name: spec for name, spec in raw.items()
            if isinstance(spec, dict) and spec.get('tier', 99) <= max_tier}


def parse_ros_path(ros_path: str) -> tuple:
    """Parse 'node:param' into (node, param).

    Example: 'controller_server:FollowPath.temperature' -> ('controller_server', 'FollowPath.temperature')
    """
    node, _, param = ros_path.partition(':')
    return node, param


def build_baseline(params: dict) -> dict:
    """Extract baseline values from loaded params. Keys are param names.

    For locked params, the locked value IS the baseline.
    """
    return {name: spec.get('baseline', spec.get('locked'))
            for name, spec in params.items()}


def get_samplable_params(params: dict) -> dict:
    """Return only params that should be sampled by Optuna (not locked)."""
    return {name: spec for name, spec in params.items()
            if 'locked' not in spec}


def get_locked_params(params: dict) -> dict:
    """Return params that are locked to a fixed value."""
    return {name: spec for name, spec in params.items()
            if 'locked' in spec}


def apply_params_filter(all_params: dict, params_csv: str) -> None:
    """Lock all params except those named in params_csv at their baseline.

    Mutates all_params in place. Raises ValueError if any name is unknown.
    """
    if not params_csv:
        return
    param_names = [p.strip() for p in params_csv.split(',')]
    for pn in param_names:
        if pn not in all_params:
            raise ValueError(
                f"Unknown param '{pn}'. Available: "
                f"{', '.join(sorted(all_params.keys()))}")
    for name, spec in all_params.items():
        if name not in param_names and 'locked' not in spec:
            spec['locked'] = spec['baseline']


# ---------------------------------------------------------------------------
# Parameter management
# ---------------------------------------------------------------------------

def _format_param_value(value) -> str:
    """Format a parameter value for the ROS2 SetParameters service.

    Must include explicit `type` field — without it, type defaults to 0
    (PARAMETER_NOT_SET) and the service silently treats the call as an
    undeclare request, returning successful=False.

    ROS2 ParameterType: BOOL=1, INTEGER=2, DOUBLE=3, STRING=4.
    """
    if isinstance(value, bool):
        return f"type: 1, bool_value: {str(value).lower()}"
    elif isinstance(value, int):
        return f"type: 2, integer_value: {value}"
    elif isinstance(value, float):
        return f"type: 3, double_value: {value}"
    else:
        return f"type: 4, string_value: '{value}'"


def _set_params_batch_service(node: str,
                              params: list[tuple[str, object]]) -> bool:
    """Set multiple params on one node via SetParameters service call.

    Uses ros2 service call /{node}/set_parameters with a list of
    Parameter messages. One docker exec + one DDS call per node.

    Uses docker_exec() which wraps the inner command with `timeout` to
    prevent container-side orphans when the host subprocess times out.
    """
    if not params:
        return True

    # Build the Parameter list YAML
    param_entries = []
    for param_name, value in params:
        val_field = _format_param_value(value)
        param_entries.append(
            f"{{name: '{param_name}', "
            f"value: {{{val_field}}}}}"
        )
    params_yaml = "{parameters: [" + ", ".join(param_entries) + "]}"

    ros_cmd = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"ros2 service call /{node}/set_parameters "
        f"rcl_interfaces/srv/SetParameters \"{params_yaml}\""
    )

    try:
        result = _docker_exec(ros_cmd, timeout=30)
        if result.returncode != 0:
            print(f"  WARN: batch service call to /{node} failed: "
                  f"{result.stderr.strip()[:200]}", file=sys.stderr)
            return False

        # Parse response — check for successful=False in any result
        stdout = result.stdout
        if 'successful=False' in stdout:
            # Extract reason(s) from response
            import re
            reasons = re.findall(r"reason='([^']*)'", stdout)
            reasons_str = '; '.join(r for r in reasons if r)
            print(f"  WARN: /{node} SetParameters rejected: {reasons_str or stdout.strip()[:200]}",
                  file=sys.stderr)
            return False

        return True
    except subprocess.TimeoutExpired:
        print(f"  WARN: batch service call to /{node} timed out",
              file=sys.stderr)
        return False


def set_all_params(trial_values: dict, all_params: dict) -> bool:
    """Set parameters via batched service calls, grouped by node.

    Groups params by target node, then makes one SetParameters service
    call per node. Typically 1-2 calls total instead of N individual
    ros2 param set commands.
    """
    # Group by node, coercing values to match declared ROS2 type
    by_node: dict[str, list[tuple[str, object]]] = {}
    for name, value in trial_values.items():
        spec = all_params[name]
        node, param = parse_ros_path(spec['ros_path'])
        # Coerce: ROS2 rejects int where double is expected (and vice versa)
        declared_type = spec.get('type', '')
        if declared_type == 'float' and isinstance(value, int) and not isinstance(value, bool):
            value = float(value)
        elif declared_type == 'int' and isinstance(value, float):
            value = int(round(value))
        by_node.setdefault(node, []).append((param, value))

    ok = True
    for node, params in by_node.items():
        if not _set_params_batch_service(node, params):
            ok = False
    return ok


def set_locked_params(all_params: dict, max_attempts: int = 3) -> bool:
    """Set locked params and verify they took effect.

    Retries up to max_attempts times with increasing delays because
    SetParameters can silently fail after a fresh stack restart (node
    accepts the call before it's fully initialized).

    Returns True if all locked params verified successfully.
    """
    locked = get_locked_params(all_params)
    if not locked:
        return True
    locked_values = {name: spec['locked'] for name, spec in locked.items()}
    n = len(locked_values)

    for attempt in range(1, max_attempts + 1):
        if attempt > 1:
            delay = 5 * attempt
            print(f"  Retry {attempt}/{max_attempts}: waiting {delay}s...",
                  file=sys.stderr)
            time.sleep(delay)

        print(f"  Setting {n} locked params (attempt {attempt}/{max_attempts})...",
              file=sys.stderr)
        ok = set_all_params(locked_values, all_params)
        if not ok:
            print(f"  WARNING: set_all_params returned failure", file=sys.stderr)
            continue

        # Verify by dumping and comparing
        dumps = dump_all_params()
        if not dumps.get('controller'):
            print(f"  WARNING: could not dump params for verification",
                  file=sys.stderr)
            continue

        mismatches = verify_trial_params(dumps, locked_values, all_params)
        if not mismatches:
            print(f"  Locked params verified OK", file=sys.stderr)
            return True

        print(f"  WARNING: {len(mismatches)} locked param mismatches:",
              file=sys.stderr)
        for m in mismatches:
            print(f"    {m}", file=sys.stderr)

    print(f"  FATAL: locked params failed after {max_attempts} attempts",
          file=sys.stderr)
    return False


def restore_baseline(all_params: dict) -> None:
    """Restore baseline parameters (called via atexit)."""
    print("\nRestoring baseline MPPI parameters...", file=sys.stderr)
    baseline = build_baseline(all_params)
    if set_all_params(baseline, all_params):
        print("Baseline restored.", file=sys.stderr)
    else:
        print("WARNING: Failed to restore some baseline parameters!",
              file=sys.stderr)


# ---------------------------------------------------------------------------
# Stack health check & restart
# ---------------------------------------------------------------------------

def check_stack_health() -> bool:
    """Check if key containers are running and /odometry/filtered has a publisher.

    Returns True if stack looks healthy, False if something is down.
    Cost: ~2-3s.
    """
    # Check containers via docker ps
    try:
        result = _run(
            ['docker', 'compose', 'ps', '--format', '{{.Name}} {{.State}}'],
            timeout=10,
        )
        if result.returncode != 0:
            print("  HEALTH: docker compose ps failed", file=sys.stderr)
            return False

        running_containers = result.stdout
        for container in HEALTH_CHECK_CONTAINERS:
            if container not in running_containers:
                print(f"  HEALTH: container '{container}' not found in "
                      f"docker ps output", file=sys.stderr)
                return False
            for line in running_containers.splitlines():
                if container in line and 'running' not in line.lower():
                    print(f"  HEALTH: container '{container}' is not running: "
                          f"{line.strip()}", file=sys.stderr)
                    return False

    except (subprocess.TimeoutExpired, Exception) as e:
        print(f"  HEALTH: docker ps check failed: {e}", file=sys.stderr)
        return False

    # Check /odometry/filtered topic has a publisher
    try:
        odom_cmd = "source /opt/ros/jazzy/setup.bash && ros2 topic info /odometry/filtered"
        result = _docker_exec(odom_cmd, timeout=10)
        if result.returncode != 0:
            print("  HEALTH: ros2 topic info /odometry/filtered failed", file=sys.stderr)
            return False
        if 'Publisher count: 0' in result.stdout:
            print("  HEALTH: /odometry/filtered has no publishers", file=sys.stderr)
            return False
    except (subprocess.TimeoutExpired, Exception) as e:
        print(f"  HEALTH: /odometry/filtered check failed: {e}", file=sys.stderr)
        return False

    return True


def full_restart_stack(stack_target: str) -> bool:
    """Force-clean and restart the stack. Cost: ~60s.

    Uses subprocess (not import) because stack.py calls sys.exit on failure.
    Returns True on success.
    """
    print(f"\n  RESTART: force-clean + start {stack_target}...",
          file=sys.stderr)

    stack_py = str(Path(__file__).parent / 'stack.py')

    # force-clean
    try:
        result = _run(
            [sys.executable, stack_py, 'force-clean'],
            timeout=120,
        )
        if result.returncode != 0:
            print(f"  RESTART: force-clean failed: {result.stderr.strip()}",
                  file=sys.stderr)
            return False
    except subprocess.TimeoutExpired:
        print("  RESTART: force-clean timed out", file=sys.stderr)
        return False

    # start
    try:
        result = _run(
            [sys.executable, stack_py, 'start', stack_target],
            timeout=120,
        )
        if result.returncode != 0:
            print(f"  RESTART: start failed: {result.stderr.strip()}",
                  file=sys.stderr)
            return False
    except subprocess.TimeoutExpired:
        print("  RESTART: start timed out", file=sys.stderr)
        return False

    # Wait for Nav2 lifecycle activation
    print("  RESTART: waiting 30s for Nav2 lifecycle activation...",
          file=sys.stderr)
    time.sleep(30)

    if check_stack_health():
        print("  RESTART: stack healthy after restart", file=sys.stderr)
        return True
    else:
        print("  RESTART: stack unhealthy after restart!", file=sys.stderr)
        return False


# ---------------------------------------------------------------------------
# Goal checker tolerance programming
# ---------------------------------------------------------------------------

# Cache: waypoint_file -> (tolerance_xy, tolerance_yaw)
_waypoint_tolerance_cache: dict[str, tuple[float, float]] = {}


def _load_waypoint_tolerances(waypoint_file: str) -> tuple[float, float] | None:
    """Load tolerance_xy and tolerance_yaw from a waypoint YAML file.

    Returns (tolerance_xy, tolerance_yaw) or None if not found.
    """
    if waypoint_file in _waypoint_tolerance_cache:
        return _waypoint_tolerance_cache[waypoint_file]

    yaml_path = (Path(__file__).parent / 'driving_instructions'
                 / f'{waypoint_file}.yaml')
    if not yaml_path.exists():
        return None

    try:
        with open(yaml_path) as f:
            data = yaml.safe_load(f)
        waypoints = data.get('waypoints', [])
        if not waypoints:
            return None
        # Use the last waypoint's tolerances (that's the goal)
        last_wp = waypoints[-1]
        tol_xy = last_wp.get('tolerance_xy')
        tol_yaw = last_wp.get('tolerance_yaw')
        if tol_xy is not None and tol_yaw is not None:
            result = (float(tol_xy), float(tol_yaw))
            _waypoint_tolerance_cache[waypoint_file] = result
            return result
    except Exception as e:
        print(f"  WARN: Failed to load tolerances from {yaml_path}: {e}",
              file=sys.stderr)
    return None


def set_goal_checker_tolerances(tolerance_xy: float,
                                tolerance_yaw: float) -> bool:
    """Set StoppedGoalChecker tolerances on controller_server.

    Programs xy_goal_tolerance and yaw_goal_tolerance via SetParameters
    service call. Returns True if set and verified.
    """
    params = [
        ('general_goal_checker.xy_goal_tolerance', tolerance_xy),
        ('general_goal_checker.yaw_goal_tolerance', tolerance_yaw),
    ]
    ok = _set_params_batch_service('controller_server', params)
    if not ok:
        print(f"  WARN: Failed to set goal checker tolerances "
              f"(xy={tolerance_xy}, yaw={tolerance_yaw})",
              file=sys.stderr)
    else:
        print(f"  Goal checker tolerances: xy={tolerance_xy}, "
              f"yaw={tolerance_yaw}", file=sys.stderr)
    return ok


# ---------------------------------------------------------------------------
# Test execution
# ---------------------------------------------------------------------------

def run_test(waypoint_file: str, timeout: int, use_amcl: bool = False,
             record: bool = False, cycles: int = 1,
             trust_nav2: bool = True) -> dict:
    """Run Nav2 waypoint test via drive_nav2.py.

    Args:
        cycles: Number of warm-reset cycles to run per test.
        trust_nav2: Trust Nav2 STATUS_SUCCEEDED for goal detection.
            Defaults to True (StoppedGoalChecker provides accuracy signal).

    Returns the parsed JSON result dict, or a failure dict.
    """
    # Program StoppedGoalChecker with per-test tolerances from waypoint YAML
    tolerances = _load_waypoint_tolerances(waypoint_file)
    if tolerances:
        set_goal_checker_tolerances(tolerances[0], tolerances[1])

    cmd = [
        sys.executable, str(Path(__file__).parent / 'drive_nav2.py'),
        '--waypoints', waypoint_file,
        '--cycles', str(cycles),
        '--warm-reset',
        '--json',
        '--timeout', str(timeout),
    ]
    if not use_amcl:
        cmd.append('--no-amcl')
    if record:
        cmd.append('--record')
    if not trust_nav2:
        cmd.append('--no-trust-nav2')

    try:
        # Each cycle needs: warm reset (~10s) + test drive (up to timeout)
        subprocess_timeout = cycles * (timeout + 15) + 30
        result = _run(cmd, timeout=subprocess_timeout)

        stderr_text = result.stderr
        stdout = result.stdout

        try:
            data = json.loads(stdout)
            data['_stderr'] = stderr_text
            return data
        except json.JSONDecodeError:
            # Find the first '{' that starts a valid JSON block
            brace_idx = stdout.find('{')
            if brace_idx >= 0:
                try:
                    data = json.loads(stdout[brace_idx:])
                    data['_stderr'] = stderr_text
                    return data
                except json.JSONDecodeError:
                    pass
            return {'success': False,
                    'message': f'JSON parse error: {stdout[:200]}',
                    '_stderr': stderr_text}

    except subprocess.TimeoutExpired:
        return {'success': False, 'message': 'subprocess timeout'}
    except Exception as e:
        return {'success': False, 'message': str(e)}


def extract_cycle_data(data: dict, fallback_duration: float = 0.0) -> tuple:
    """Unwrap and aggregate cycle results from drive_nav2.py --json output.

    For single-cycle: returns data from results[0].
    For multi-cycle: aggregates across all cycles:
      - success = True only if ALL cycles pass
      - collision = True if ANY cycle has collision
      - duration = average across cycles
      - metrics = element-wise average of numeric metric leaves
      - test_data = worst cycle's test_data (for error_xy, error_yaw)

    Returns (success, collision, duration, metrics, test_data).
    """
    cycle_results = data.get('results', [])
    if not cycle_results:
        test_data = data
        return (test_data.get('success', False),
                test_data.get('collision', False),
                test_data.get('duration', fallback_duration),
                test_data.get('metrics', {}),
                test_data)

    # Single cycle — fast path
    if len(cycle_results) == 1:
        test_data = cycle_results[0].get('test_data', {})
        return (test_data.get('success', False),
                test_data.get('collision', False),
                test_data.get('duration', fallback_duration),
                test_data.get('metrics', {}),
                test_data)

    # Multi-cycle aggregation (median for robustness)
    all_td = [c.get('test_data', {}) for c in cycle_results]

    success = all(td.get('success', False) for td in all_td)
    collision = any(td.get('collision', False) for td in all_td)
    durations = [td.get('duration', 0) for td in all_td]
    duration = _median(durations)

    # Median metrics (recursive for nested dicts)
    all_metrics = [td.get('metrics', {}) for td in all_td]
    metrics = _median_metrics(all_metrics)

    # Representative cycle: the one whose duration is closest to the median
    median_dur = duration
    best_idx = 0
    best_diff = float('inf')
    for i, td in enumerate(all_td):
        diff = abs(td.get('duration', 0) - median_dur)
        if diff < best_diff:
            best_diff = diff
            best_idx = i
    representative_td = all_td[best_idx]

    return success, collision, duration, metrics, representative_td


def _median(vals: list) -> float:
    """Median of a list of numbers."""
    s = sorted(vals)
    n = len(s)
    if n % 2 == 1:
        return s[n // 2]
    return (s[n // 2 - 1] + s[n // 2]) / 2


def select_medoid(reps: list[list[float]]) -> tuple[int, float]:
    """Pick the most representative run from a set of repetitions.

    Each rep is a list of objective values (same length). Returns the index
    of the medoid — the rep with the smallest total distance to all others,
    after min-max normalization so all objectives contribute equally.

    Also returns a consistency score: mean normalized distance from each rep
    to the medoid. 0.0 = all reps identical, higher = more spread.

    Args:
        reps: List of N repetitions, each a list of M objective values.
              N must be odd and >= 3.

    Returns:
        (medoid_index, consistency) — index (0-based) and spread metric.
    """
    n = len(reps)
    assert n >= 3, f"Need at least 3 reps, got {n}"
    m = len(reps[0])
    assert all(len(r) == m for r in reps), "All reps must have same number of objectives"

    # Replace NaN/inf with 0 to avoid poisoning the calculation
    import math
    clean = [
        [0.0 if (math.isnan(v) or math.isinf(v)) else v for v in row]
        for row in reps
    ]

    # Min-max normalize each objective across reps
    mins = [min(clean[i][j] for i in range(n)) for j in range(m)]
    maxs = [max(clean[i][j] for i in range(n)) for j in range(m)]

    normed = []
    for i in range(n):
        row = []
        for j in range(m):
            span = maxs[j] - mins[j]
            row.append((clean[i][j] - mins[j]) / span if span > 0 else 0.0)
        normed.append(row)

    # Pairwise Euclidean distances, sum per rep
    total_dist = [0.0] * n
    for i in range(n):
        for k in range(i + 1, n):
            d = sum((normed[i][j] - normed[k][j]) ** 2 for j in range(m)) ** 0.5
            total_dist[i] += d
            total_dist[k] += d

    medoid = min(range(n), key=lambda i: total_dist[i])

    # Consistency: mean coefficient of variation across objectives.
    # CV = std/|mean| for each objective, averaged. This is dimensionless and
    # comparable across trials (unlike the min-max normalized distances used
    # for medoid selection).
    cvs = []
    for j in range(m):
        vals = [clean[i][j] for i in range(n)]
        mean_val = sum(vals) / n
        if abs(mean_val) < 1e-10:
            continue  # skip near-zero-mean objectives (CV undefined)
        std_val = (sum((v - mean_val) ** 2 for v in vals) / n) ** 0.5
        cvs.append(std_val / abs(mean_val))
    consistency = sum(cvs) / len(cvs) if cvs else 0.0

    return medoid, consistency


def _median_metrics(metrics_list: list) -> dict:
    """Element-wise median of a list of nested metric dicts."""
    if not metrics_list:
        return {}
    result = {}
    keys = set()
    for m in metrics_list:
        keys.update(m.keys())
    for k in keys:
        vals = [m[k] for m in metrics_list if k in m]
        if not vals:
            continue
        if isinstance(vals[0], dict):
            result[k] = _median_metrics(vals)
        elif isinstance(vals[0], (int, float)):
            result[k] = _median(vals)
        else:
            result[k] = vals[0]  # non-numeric: take first
    return result


# ---------------------------------------------------------------------------
# Scoring helpers
# ---------------------------------------------------------------------------

DEFAULT_JITTER_WEIGHTS = {
    'steer_rms': 2.0,
    'steer_flip': 0.5,
    'tv_angular': 50.0,
    'tv_linear': 20.0,
    'tv_wheel': 0.05,
    'reversals': 0.5,
}


def compute_jitter_score(metrics: dict, weights: dict = None) -> float:
    """Jitter-only component from episode metrics. Lower is better.

    Combines steering oscillation, cmd_vel total variation, and wheel
    velocity total variation. No duration or cross-track terms.
    """
    w = weights or DEFAULT_JITTER_WEIGHTS
    steer = metrics.get('steering', {})
    cmdvel = metrics.get('cmd_vel', {})
    wheel = metrics.get('wheel_vel', {})
    # Prefer command-side steering metrics (pre-physics) over Gazebo feedback
    # which is laggy and smooths out real control jitter
    steer_rms = steer.get('cmd_rms_rate', steer.get('rms_rate', 1.0))
    steer_flip = steer.get('cmd_flip_rate_hz', steer.get('flip_rate_hz', 10.0))
    return (
        w['steer_rms'] * steer_rms
        + w['steer_flip'] * steer_flip
        + w['tv_angular'] * cmdvel.get('tv_angular', 0.01)
        + w['tv_linear'] * cmdvel.get('tv_linear', 0.01)
        + w['tv_wheel'] * wheel.get('tv_combined', 5.0)
        + w['reversals'] * cmdvel.get('reversal_count', 0)
    )


def extract_rep_metrics(data: dict, elapsed: float = 0.0) -> dict:
    """Extract a flat metrics dict from a single test rep result.

    Shared extraction logic used by both tuning harnesses and replication
    scripts. Combines fields from extract_cycle_data output (metrics dict,
    test_data waypoint_results, pfc) into one flat dict.

    Args:
        data: Raw JSON result from run_test() / drive_nav2.py --json.
        elapsed: Fallback duration if not present in data.

    Returns:
        Dict with keys: success, collision, duration, error_xy, error_yaw,
        jitter, reversal_count, collisions, steering_rms_rate,
        steering_flip_hz, tv_linear, tv_angular, tv_wheel,
        pfc_mean, pfc_max, pfc_integral, pfc_count.
    """
    success, collision, duration, metrics, test_data = \
        extract_cycle_data(data, fallback_duration=elapsed)

    # Error distances from waypoint results
    wp_results = test_data.get('waypoint_results', [])
    error_xy = None
    error_yaw = None
    if wp_results:
        last_wp = wp_results[-1]
        error_xy = last_wp.get('error_xy')
        error_yaw = last_wp.get('error_yaw')

    # Nested metric groups
    cmdvel = metrics.get('cmd_vel', {})
    steer = metrics.get('steering', {})
    wheel = metrics.get('wheel_vel', {})
    pfc = test_data.get('pfc', {})

    return {
        'success': success,
        'collision': collision,
        'duration': duration,
        'error_xy': error_xy,
        'error_yaw': error_yaw,
        'jitter': compute_jitter_score(metrics),
        'reversal_count': cmdvel.get('reversal_count', 0),
        'collisions': 1 if collision else 0,
        'steering_rms_rate': steer.get('cmd_rms_rate', steer.get('rms_rate')),
        'steering_flip_hz': steer.get('cmd_flip_rate_hz', steer.get('flip_rate_hz')),
        'tv_linear': cmdvel.get('tv_linear'),
        'tv_angular': cmdvel.get('tv_angular'),
        'tv_wheel': wheel.get('tv_combined'),
        'pfc_mean': pfc.get('pfc_mean'),
        'pfc_max': pfc.get('pfc_max'),
        'pfc_integral': pfc.get('pfc_integral'),
        'pfc_count': pfc.get('pfc_count'),
        # Keep raw nested dicts for callers that need them
        '_metrics': metrics,
        '_test_data': test_data,
    }


# ---------------------------------------------------------------------------
# Parameter dump & verification
# ---------------------------------------------------------------------------

PARAM_DUMP_DIR = Path(__file__).parent / 'logs' / 'param_dumps'
DUMP_NODES = ['controller_server', 'planner_server', 'behavior_server']
DUMP_NODE_KEYS = ['controller', 'planner', 'behavior']


def dump_node_params(node: str, timeout: int = 15) -> str | None:
    """Dump all params from a ROS2 node via `ros2 param dump`.

    Returns YAML string or None on failure/timeout. Not fatal.
    """
    ros_cmd = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"ros2 param dump /{node}"
    )
    try:
        result = _docker_exec(ros_cmd, timeout=timeout)
        if result.returncode == 0 and result.stdout.strip():
            return result.stdout
        print(f"  WARN: param dump /{node} failed (rc={result.returncode}): "
              f"{result.stderr.strip()[:200]}", file=sys.stderr)
        return None
    except subprocess.TimeoutExpired:
        print(f"  WARN: param dump /{node} timed out", file=sys.stderr)
        return None
    except Exception as e:
        print(f"  WARN: param dump /{node} error: {e}", file=sys.stderr)
        return None


def dump_all_params() -> dict[str, str]:
    """Dump params from controller_server, planner_server, behavior_server.

    Returns {'controller': yaml_str, 'planner': yaml_str, 'behavior': yaml_str}.
    Values are None for nodes that failed to dump.
    """
    result = {}
    for node, key in zip(DUMP_NODES, DUMP_NODE_KEYS):
        result[key] = dump_node_params(node)
    return result


def save_param_dump(node_type: str, yaml_content: str) -> str:
    """Content-address and save a param dump. Returns md5 hex string.

    Saves to logs/param_dumps/by_hash/<node_type>/<md5>.yaml (skips if exists).
    """
    md5 = hashlib.md5(yaml_content.encode()).hexdigest()
    out_dir = PARAM_DUMP_DIR / 'by_hash' / node_type
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f'{md5}.yaml'
    if not out_path.exists():
        out_path.write_text(yaml_content)
    return md5


def verify_trial_params(dumps: dict, trial_values: dict,
                        all_params: dict) -> dict:
    """Check that trial params actually took effect in the ROS2 nodes.

    Parses controller/planner dump YAML and compares against expected values.
    Returns {param_name: {'expected': x, 'actual': y}} for mismatches.
    Empty dict = all verified OK.
    """
    mismatches = {}

    # Parse dump YAMLs once
    parsed = {}
    for key in DUMP_NODE_KEYS:
        content = dumps.get(key)
        if content:
            try:
                parsed[key] = yaml.safe_load(content)
            except yaml.YAMLError:
                pass

    # Map node names to dump keys
    node_to_key = dict(zip(DUMP_NODES, DUMP_NODE_KEYS))

    for param_name, expected_value in trial_values.items():
        spec = all_params.get(param_name)
        if not spec:
            continue

        ros_path = spec['ros_path']
        node, param_path = parse_ros_path(ros_path)
        dump_key = node_to_key.get(node)
        if not dump_key or dump_key not in parsed:
            continue

        # Navigate the parsed YAML to find the actual value
        # ros2 param dump format: {/node_name: {ros__parameters: {param: value}}}
        node_data = parsed[dump_key]

        # The dump has the node name as top-level key (with leading slash)
        node_key = f'/{node}' if f'/{node}' in node_data else node
        if node_key in node_data:
            params_data = node_data[node_key].get('ros__parameters', {})
        elif 'ros__parameters' in node_data:
            params_data = node_data['ros__parameters']
        else:
            params_data = node_data

        # Navigate dotted path (e.g. 'FollowPath.temperature')
        parts = param_path.split('.')
        actual = params_data
        found = True
        for part in parts:
            if isinstance(actual, dict) and part in actual:
                actual = actual[part]
            else:
                found = False
                break

        if not found:
            continue

        # Compare with tolerance for numeric types (int/float interchangeable)
        if isinstance(expected_value, (int, float)) and isinstance(actual, (int, float)):
            if abs(float(expected_value) - float(actual)) > 1e-6 * max(abs(float(expected_value)), 1.0):
                mismatches[param_name] = {
                    'expected': expected_value, 'actual': actual}
        elif str(expected_value) != str(actual):
            mismatches[param_name] = {
                'expected': expected_value, 'actual': actual}

    return mismatches


def abort_on_mismatch(mismatches: dict) -> None:
    """Abort the entire study if any param mismatches are detected.

    Raises SystemExit (not caught by Optuna's exception handling).
    """
    if not mismatches:
        return
    print(f"\n{'='*60}", file=sys.stderr)
    print(f"FATAL: {len(mismatches)} param mismatches — study aborted!",
          file=sys.stderr)
    print(f"SetParameters reported success but params did not take effect.",
          file=sys.stderr)
    for k, v in mismatches.items():
        print(f"  {k}: expected={v['expected']}, actual={v['actual']}",
              file=sys.stderr)
    print(f"{'='*60}", file=sys.stderr)
    sys.exit(1)


def record_trial_dumps(trial, dumps: dict) -> dict[str, str]:
    """Save param dumps and record hashes in Optuna trial attrs.

    Returns {'controller': hash, 'planner': hash, 'behavior': hash}.
    """
    hashes = {}
    for key in DUMP_NODE_KEYS:
        content = dumps.get(key)
        if content:
            h = save_param_dump(key, content)
            hashes[key] = h
            try:
                trial.set_user_attr(f'param_hash_{key}', h)
            except Exception as e:
                print(f"  WARN: Failed to set param_hash_{key}: {e}",
                      file=sys.stderr)
    return hashes


def save_study_snapshot(study_name: str, all_params: dict,
                        is_resume: bool = False,
                        params_file_path: str = None) -> None:
    """Dump all 3 nodes and save as a study snapshot with provenance info.

    Saves to logs/param_dumps/studies/<study_name>/start/ or resume_N/.
    Generates study_info.md with git commit, command, params file info.
    """
    study_dir = PARAM_DUMP_DIR / 'studies' / study_name

    if is_resume:
        # Auto-increment resume_N
        existing = sorted(study_dir.glob('resume_*'))
        n = len(existing) + 1
        snapshot_dir = study_dir / f'resume_{n}'
    else:
        snapshot_dir = study_dir / 'start'

    snapshot_dir.mkdir(parents=True, exist_ok=True)

    # Dump all nodes
    dumps = dump_all_params()
    for key in DUMP_NODE_KEYS:
        content = dumps.get(key)
        if content:
            (snapshot_dir / f'{key}.yaml').write_text(content)

    # Build study_info.md
    info_lines = [f'# Study: {study_name}', '']

    # Git info
    try:
        git_branch = subprocess.run(
            ['git', 'rev-parse', '--abbrev-ref', 'HEAD'],
            capture_output=True, text=True, timeout=5,
        ).stdout.strip()
        git_commit = subprocess.run(
            ['git', 'rev-parse', '--short', 'HEAD'],
            capture_output=True, text=True, timeout=5,
        ).stdout.strip()
        git_dirty = subprocess.run(
            ['git', 'status', '--porcelain'],
            capture_output=True, text=True, timeout=5,
        ).stdout.strip()
        dirty_flag = ' (dirty)' if git_dirty else ''
        info_lines.append(f'- **Git**: `{git_branch}` @ `{git_commit}`{dirty_flag}')
    except Exception:
        info_lines.append('- **Git**: unavailable')

    # Command
    info_lines.append(f'- **Command**: `{" ".join(sys.argv)}`')
    info_lines.append(f'- **Date**: {datetime.datetime.now().isoformat()}')

    # Params file
    if params_file_path:
        try:
            content = Path(params_file_path).read_text()
            params_hash = hashlib.md5(content.encode()).hexdigest()[:8]
            info_lines.append(f'- **Params file**: `{params_file_path}` (md5: `{params_hash}`)')
        except Exception:
            info_lines.append(f'- **Params file**: `{params_file_path}`')
    else:
        info_lines.append(f'- **Params file**: default (`config/tuning_params.yaml`)')

    # Resume info
    if is_resume:
        info_lines.append(f'- **Resume**: snapshot #{n}')

    info_lines.append('')

    (snapshot_dir / 'study_info.md').write_text('\n'.join(info_lines))
    print(f"  Study snapshot saved: {snapshot_dir}", file=sys.stderr)


# ---------------------------------------------------------------------------
# Trial data storage
# ---------------------------------------------------------------------------

def store_trial_data(trial, test_results: list, study_name: str,
                     score=None, objectives=None,
                     trial_values: dict = None,
                     stderr_snippets: dict = None,
                     param_hashes: dict = None) -> None:
    """Persist comprehensive trial data to Optuna user_attrs + local JSON.

    Works for both shotgun (single score) and laser (multi-objective) runners.
    Old trials without user_attrs continue to work — no migration needed.
    """
    # --- Build per_test detail ---
    per_test = {}
    any_collision = False
    all_passed = True
    tests_passed = 0
    tests_failed = 0

    for r in test_results:
        tid = r.get('test_id', r.get('waypoint_file', 'unknown'))
        success = r.get('success', False)
        collision = r.get('collision', False)

        if collision:
            any_collision = True
        if not success:
            all_passed = False
            tests_failed += 1
        else:
            tests_passed += 1

        metrics = r.get('metrics', {})
        steer = metrics.get('steering', {})
        cmdvel = metrics.get('cmd_vel', {})
        track = metrics.get('tracking', {})

        entry = {
            'success': success,
            'collision': collision,
            'duration': r.get('duration', 0),
        }

        # Jitter score (if metrics available)
        if metrics:
            entry['jitter'] = round(compute_jitter_score(metrics), 3)

        # Accuracy (from waypoint results or top-level)
        if 'error_xy' in r:
            entry['accuracy_xy'] = r['error_xy']
        if 'error_yaw' in r:
            entry['accuracy_yaw'] = r['error_yaw']

        # Reversal count
        if 'reversal_count' in cmdvel:
            entry['reversal_count'] = cmdvel['reversal_count']

        # Key metric subsets (flat for easy querying)
        if 'tv_linear' in cmdvel:
            entry['tv_linear'] = cmdvel['tv_linear']
        if 'tv_angular' in cmdvel:
            entry['tv_angular'] = cmdvel['tv_angular']
        if 'rms_rate' in steer:
            entry['steering_rms_rate'] = steer['rms_rate']
        if 'flip_rate_hz' in steer:
            entry['steering_flip_hz'] = steer['flip_rate_hz']
        if 'rms_xtrack_m' in track:
            entry['rms_xtrack_m'] = track['rms_xtrack_m']

        # Per-cycle breakdown (for multi-cycle runs)
        raw_cycles = r.get('raw_cycles', [])
        if raw_cycles:
            cycles_detail = []
            for ci, cr in enumerate(raw_cycles):
                td = cr.get('test_data', {})
                c_metrics = td.get('metrics', {})
                c_wp = td.get('waypoint_results', [])
                c_entry = {
                    'cycle': ci,
                    'success': td.get('success', False),
                    'collision': td.get('collision', False),
                    'duration': td.get('duration', 0),
                }
                if c_wp:
                    c_entry['error_xy'] = c_wp[-1].get('error_xy')
                    c_entry['error_yaw'] = c_wp[-1].get('error_yaw')
                if c_metrics:
                    c_entry['jitter'] = round(
                        compute_jitter_score(c_metrics), 3)
                    c_steer = c_metrics.get('steering', {})
                    c_cmdvel = c_metrics.get('cmd_vel', {})
                    c_track = c_metrics.get('tracking', {})
                    if 'rms_rate' in c_steer:
                        c_entry['steering_rms_rate'] = c_steer['rms_rate']
                    if 'tv_angular' in c_cmdvel:
                        c_entry['tv_angular'] = c_cmdvel['tv_angular']
                    if 'reversal_count' in c_cmdvel:
                        c_entry['reversal_count'] = c_cmdvel['reversal_count']
                    if 'rms_xtrack_m' in c_track:
                        c_entry['rms_xtrack_m'] = c_track['rms_xtrack_m']
                cycles_detail.append(c_entry)
            entry['cycles'] = cycles_detail
            entry['cycles_passed'] = sum(
                1 for c in cycles_detail if c['success'])
            entry['cycles_total'] = len(cycles_detail)

        per_test[tid] = entry

    # --- Optuna user_attrs (searchable) ---
    try:
        trial.set_user_attr('any_collision', any_collision)
        trial.set_user_attr('all_passed', all_passed)
        trial.set_user_attr('tests_passed', tests_passed)
        trial.set_user_attr('tests_failed', tests_failed)
        trial.set_user_attr('per_test', per_test)
    except Exception as e:
        print(f"  WARN: Failed to set user_attrs: {e}", file=sys.stderr)

    # --- Local JSON log ---
    try:
        log_dir = Path('logs/optuna_trials') / study_name
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f'trial_{trial.number:04d}.json'

        log_data = {
            'trial_number': trial.number,
            'timestamp': datetime.datetime.now().isoformat(),
            'study_name': study_name,
            'params': dict(trial.params) if trial.params else (trial_values or {}),
            'per_test': per_test,
            'any_collision': any_collision,
            'all_passed': all_passed,
            'tests_passed': tests_passed,
            'tests_failed': tests_failed,
        }

        if score is not None:
            log_data['score'] = score
        if objectives is not None:
            log_data['objectives'] = list(objectives)
        if trial_values is not None:
            log_data['trial_values'] = trial_values
        if stderr_snippets:
            log_data['stderr_snippets'] = stderr_snippets
        if param_hashes:
            log_data['param_hashes'] = param_hashes

        with open(log_path, 'w') as f:
            json.dump(log_data, f, indent=2)

    except Exception as e:
        print(f"  WARN: Failed to write trial log: {e}", file=sys.stderr)


# ---------------------------------------------------------------------------
# Optuna helpers
# ---------------------------------------------------------------------------

def sample_trial_values(trial, all_params: dict,
                        baseline_only: bool = False) -> dict:
    """Build trial values: sample free params, use locked values.

    Returns dict of {param_name: value} for all params.
    """
    samplable = get_samplable_params(all_params)
    locked = get_locked_params(all_params)

    trial_values = {}

    # Locked params: always set to locked value
    for name, spec in locked.items():
        trial_values[name] = spec['locked']

    # Samplable params: sample or use baseline
    for name, spec in samplable.items():
        if baseline_only:
            trial_values[name] = spec['baseline']
        else:
            low, high = spec['range']
            log = spec.get('scale', 'linear') == 'log'
            if spec['type'] == 'int':
                trial_values[name] = trial.suggest_int(name, int(low), int(high))
            elif log:
                trial_values[name] = trial.suggest_float(name, low, high, log=True)
            else:
                trial_values[name] = trial.suggest_float(name, low, high)

    return trial_values


def log_trial_params(trial_number: int, trial_values: dict,
                     baseline: dict, locked: dict) -> None:
    """Print sampled trial parameters to stderr with change markers."""
    sampled = {k: v for k, v in trial_values.items() if k not in locked}
    print(f"\n{'='*60}", file=sys.stderr)
    print(f"Trial {trial_number}: {len(sampled)} sampled params "
          f"({len(locked)} locked, set once at start)", file=sys.stderr)
    for k, v in sampled.items():
        baseline_val = baseline.get(k, v)
        marker = ' *' if v != baseline_val else ''
        print(f"  {k}: {v}{marker}", file=sys.stderr)
    print(f"{'='*60}", file=sys.stderr)


def resolve_storage(storage_arg, study_name: str) -> str:
    """Resolve Optuna storage: --storage flag > $OPTUNA_STORAGE > local SQLite."""
    if storage_arg is not None:
        return storage_arg
    env_storage = os.environ.get('OPTUNA_STORAGE')
    if env_storage:
        return env_storage
    db_dir = Path('logs/optuna')
    db_dir.mkdir(parents=True, exist_ok=True)
    return f'sqlite:///{db_dir / study_name}.db'


def create_or_load_study(study_name: str, storage: str, resume: bool,
                         direction: str = None, directions: list = None,
                         baseline: dict = None,
                         samplable: dict = None) -> 'optuna.Study':
    """Create a new Optuna study or load an existing one.

    For single-objective, pass direction='minimize'.
    For multi-objective, pass directions=['minimize', 'minimize', ...].
    If baseline and samplable are provided and the study is new, enqueues
    a baseline trial.
    """
    if resume:
        study = optuna.load_study(
            study_name=study_name, storage=storage)
        print(f"  Resumed study with {len(study.trials)} existing trials",
              file=sys.stderr)
    else:
        kwargs = {
            'study_name': study_name,
            'storage': storage,
            'load_if_exists': True,
        }
        if directions:
            kwargs['directions'] = directions
        elif direction:
            kwargs['direction'] = direction
        study = optuna.create_study(**kwargs)
        if len(study.trials) == 0 and baseline and samplable:
            baseline_enqueue = {name: baseline[name] for name in samplable}
            study.enqueue_trial(baseline_enqueue)
            print(f"  Created new study, baseline enqueued as trial 0",
                  file=sys.stderr)
        else:
            print(f"  Loaded existing study with {len(study.trials)} trials",
                  file=sys.stderr)
    return study


def handle_stack_crash(trial, trial_values: dict, samplable: dict,
                       stack_target: str) -> None:
    """Requeue trial params, restart stack, raise RuntimeError.

    Raises RuntimeError (not TrialPruned) so Optuna marks the trial as FAIL,
    not PRUNED. PRUNED tells the sampler "these params are bad", but a stack
    crash means the params were never actually tested.
    """
    print("  STACK CRASH detected — requeueing trial params and restarting",
          file=sys.stderr)
    trial.set_user_attr('prune_reason', 'stack_crash')
    enqueue_values = {name: trial_values[name] for name in samplable}
    trial.study.enqueue_trial(enqueue_values)
    full_restart_stack(stack_target)
    raise RuntimeError("Stack crash — trial requeued, marked FAIL not PRUNED")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def add_common_args(parser, default_study: str = 'mppi_tune') -> None:
    """Add CLI arguments shared by all tuning runners."""
    parser.add_argument('--trials', type=int, default=100,
                        help='Number of Optuna trials (default: 100)')
    parser.add_argument('--tier', type=int, default=1,
                        help='Max parameter tier to include (default: 1)')
    parser.add_argument('--study-name', type=str, default=default_study,
                        help=f'Optuna study name (default: {default_study})')
    parser.add_argument('--storage', type=str, default=None,
                        help='Optuna storage URL (default: $OPTUNA_STORAGE, '
                             'fallback: sqlite:///logs/optuna/<study-name>.db)')
    parser.add_argument('--resume', action='store_true',
                        help='Resume existing study instead of creating new')
    parser.add_argument('--baseline-only', action='store_true',
                        help='Run trials with baseline params only (verify scoring)')
    parser.add_argument('--params', type=str, default=None,
                        help='Comma-separated param names to tune (others held '
                             'at baseline). e.g. "PathAlignCritic.cost_weight,wz_std"')
    parser.add_argument('--params-file', type=str, default=None,
                        help='Path to tuning params YAML (default: config/tuning_params.yaml)')
    parser.add_argument('--amcl', action='store_true',
                        help='Use AMCL localization (default: odometry-only)')
    parser.add_argument('--stack', type=str, default='jetacker:nav2_odom',
                        help='Stack target for restarts (default: jetacker:nav2_odom)')
    parser.add_argument('--full-reset-interval', type=int, default=30,
                        help='Full stack restart every N trials (default: 30, 0=disabled)')
    parser.add_argument('--cycles', type=int, default=1,
                        help='Warm-reset cycles per test per trial (default: 1). '
                             'Higher = more stable signal, longer trials.')


def print_param_summary(all_params: dict) -> None:
    """Print parameter summary table to stderr."""
    for name, spec in sorted(all_params.items()):
        if 'locked' in spec:
            print(f"    T{spec.get('tier', '?')} {name}: LOCKED={spec['locked']}, "
                  f"{spec['type']}", file=sys.stderr)
        else:
            print(f"    T{spec['tier']} {name}: baseline={spec['baseline']}, "
                  f"range={spec['range']}, {spec['type']}/{spec.get('scale', 'linear')}",
                  file=sys.stderr)
