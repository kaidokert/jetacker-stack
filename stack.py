#!/usr/bin/env python3
"""
YAML-driven stack manager for ROS2 simulation environment.

Reads stacks.yaml for all topology — component names, service names, and
stack compositions — instead of hardcoded Python dicts.

CLI usage:
    python stack.py status
    python stack.py start jetacker:nav2
    python stack.py stop jetacker:nav2
    python stack.py restart jetacker:nav2
    python stack.py soft-reset
    python stack.py clean
    python stack.py force-clean
    python stack.py audit

Module usage:
    from stack import load_manifest, resolve_stack, resolve_services
"""

import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

# ── Colors ───────────────────────────────────────────────────────────────────

GREEN = '\033[0;32m'
YELLOW = '\033[1;33m'
RED = '\033[0;31m'
NC = '\033[0m'

# ── YAML Manifest ───────────────────────────────────────────────────────────

_manifest_cache: Optional[Dict[str, Any]] = None
_manifest_path = Path(__file__).parent / 'stacks.yaml'
_env_path = Path(__file__).parent / '.env'


def _load_dotenv():
    """Load .env file into os.environ (won't overwrite existing vars)."""
    if not _env_path.exists():
        return
    for line in _env_path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#') or '=' not in line:
            continue
        key, _, val = line.partition('=')
        os.environ.setdefault(key.strip(), val.strip())

_load_dotenv()


def get_mode() -> str:
    """Determine hw/sim mode from environment.

    Priority:
      1. MODE env var (explicit: 'hw' or 'sim')
      2. USE_SIM_TIME env var ('true' → sim, 'false' → hw)
      3. Default: 'hw'
    """
    mode = os.environ.get('MODE', '').lower()
    if mode in ('hw', 'sim'):
        return mode
    use_sim_time = os.environ.get('USE_SIM_TIME', 'false').lower()
    return 'sim' if use_sim_time == 'true' else 'hw'


def load_manifest(path: Optional[Path] = None) -> Dict[str, Any]:
    """Load and cache stacks.yaml. Importable by other modules."""
    global _manifest_cache
    if _manifest_cache is not None and path is None:
        return _manifest_cache
    p = path or _manifest_path
    with open(p) as f:
        data = yaml.safe_load(f)
    if path is None:
        _manifest_cache = data
    return data


# ── Resolvers (importable) ──────────────────────────────────────────────────

def resolve_components(manifest: Dict, stack_name: str, mode: Optional[str] = None) -> List[str]:
    """Resolve a stack's full component list, expanding extends recursively.

    If mode is provided ('hw' or 'sim'), filters out components whose modes
    list does not include the current mode.
    """
    stacks = manifest['stacks']
    if stack_name not in stacks:
        raise ValueError(f"Unknown stack: '{stack_name}'")

    stack = stacks[stack_name]
    components = []

    # Resolve extends first
    for parent in stack.get('extends', []):
        components.extend(resolve_components(manifest, parent, mode=mode))

    # Add own includes
    components.extend(stack.get('include', []))

    # Deduplicate preserving order
    seen = set()
    result = []
    for c in components:
        if c not in seen:
            seen.add(c)
            result.append(c)

    # Apply stack-level excludes (removes inherited components)
    excludes = set(stack.get('exclude', []))
    result = [c for c in result if c not in excludes]

    # Filter out disabled components
    result = [c for c in result if not get_component(manifest, c).get('disabled', False)]

    # Filter by mode if specified
    if mode is not None:
        result = [
            c for c in result
            if mode in get_component(manifest, c).get('modes', ['sim', 'hw'])
        ]

    return result


def resolve_own_components(manifest: Dict, stack_name: str) -> List[str]:
    """Return only this stack's own include list (not inherited). Used for unique service detection."""
    stacks = manifest['stacks']
    if stack_name not in stacks:
        return []
    return list(stacks[stack_name].get('include', []))


def component_to_service(manifest: Dict, robot_name: str, component: str) -> str:
    """Map a component name to its docker-compose service name for a given robot.

    Convention: underscores in component names become hyphens in service names.
    E.g. clock_bridge → jetacker-clock-bridge
    """
    robots = manifest['robots']
    robot = robots[robot_name]

    # Check service_map first (explicit overrides)
    service_map = robot.get('service_map', {})
    if component in service_map:
        return service_map[component]

    # Apply naming pattern, then convert underscores to hyphens
    pattern = robot.get('naming', manifest['config']['naming'])
    return pattern.format(robot=robot_name, component=component).replace('_', '-')


def resolve_services(manifest: Dict, robot_name: str, stack_name: str,
                     mode: Optional[str] = None) -> List[str]:
    """Resolve robot:stack → list of docker-compose service names.
    Filters out virtual components (no container) and mode-excluded components."""
    robot = manifest['robots'].get(robot_name)
    if robot is None:
        raise ValueError(f"Unknown robot: '{robot_name}'")
    if stack_name not in robot.get('stacks', []):
        raise ValueError(f"Robot '{robot_name}' does not support stack '{stack_name}'")

    if mode is None:
        mode = get_mode()

    components = resolve_components(manifest, stack_name, mode=mode)

    # Apply robot's component_excludes
    excludes = set(robot.get('component_excludes', []))
    components = [c for c in components if c not in excludes]

    # Filter out virtual components (no container, just DAG validation)
    components = [c for c in components if get_component(manifest, c).get('kind') != 'virtual']

    return [component_to_service(manifest, robot_name, c) for c in components]


def resolve_unique_services(manifest: Dict, robot_name: str, stack_name: str,
                            mode: Optional[str] = None) -> List[str]:
    """Return services unique to this stack (not from extends). Used for conflict detection.
    Filters out virtual components (no container) and mode-excluded components."""
    robot = manifest['robots'].get(robot_name)
    if robot is None:
        return []

    if mode is None:
        mode = get_mode()

    own_components = resolve_own_components(manifest, stack_name)
    excludes = set(robot.get('component_excludes', []))
    own_components = [c for c in own_components if c not in excludes]
    own_components = [c for c in own_components if get_component(manifest, c).get('kind') != 'virtual']
    own_components = [c for c in own_components
                      if mode in get_component(manifest, c).get('modes', ['sim', 'hw'])]

    return [component_to_service(manifest, robot_name, c) for c in own_components]


def get_component(manifest: Dict, component_name: str) -> Dict[str, Any]:
    """Get a component's full config with defaults applied."""
    defaults = manifest['config']['defaults']
    raw = manifest['components'].get(component_name, {})
    merged = dict(defaults)
    merged.update(raw)
    # Node name defaults to component name
    if 'node' not in merged:
        merged['node'] = component_name
    return merged


def get_stack_dependencies(manifest: Dict, robot_name: str, stack_name: str) -> List[Tuple[str, str]]:
    """Return (robot, stack) pairs that must be running before this stack.
    A stack with extends implies the parent must be running."""
    stacks = manifest['stacks']
    stack = stacks.get(stack_name, {})
    deps = []
    for parent in stack.get('extends', []):
        robot = manifest['robots'].get(robot_name, {})
        if parent in robot.get('stacks', []):
            deps.append((robot_name, parent))
    return deps


def get_conflicts(manifest: Dict, robot_name: str, stack_name: str) -> List[Tuple[str, str]]:
    """Return (robot, stack) pairs that conflict with this target."""
    target = f"{robot_name}:{stack_name}"
    conflicts = []
    for pair in manifest.get('conflicts', []):
        if target == pair[0]:
            r, s = pair[1].split(':')
            conflicts.append((r, s))
        elif target == pair[1]:
            r, s = pair[0].split(':')
            conflicts.append((r, s))
    return conflicts


def list_targets(manifest: Dict) -> List[str]:
    """List all valid robot:stack targets."""
    targets = []
    for robot_name, robot in manifest['robots'].items():
        for stack_name in robot.get('stacks', []):
            targets.append(f"{robot_name}:{stack_name}")
    # Add non-robot stacks (infra, test)
    for stack_name, stack in manifest['stacks'].items():
        if not stack.get('extends') and stack_name not in ['base']:
            # Check if any robot claims this stack
            claimed = any(
                stack_name in r.get('stacks', [])
                for r in manifest['robots'].values()
            )
            if not claimed:
                targets.append(stack_name)
    return targets


def parse_target(target: str) -> Tuple[str, str]:
    """Parse 'robot:stack' into (robot_name, stack_name)."""
    if ':' not in target:
        raise ValueError(
            f"Target must be 'robot:stack' format, got '{target}'"
        )
    robot, stack = target.split(':', 1)
    return robot, stack


def build_reset_definition(manifest: Dict, robot_name: str, stack_name: str) -> Dict[str, Dict[str, Any]]:
    """Build a reset orchestrator definition dict from stacks.yaml.

    Resolves a robot:stack into the dict format ResetOrchestrator expects:
    {
        "node_name": {
            "service": "docker-compose-service-name" or None,
            "gates": ["gate_spec", ...],
            "depends": ["parent_node_name", ...],
            "timeout": 30.0,
            "critical": True,
            "is_bridge": True/False
        }
    }
    """
    robot = manifest['robots'].get(robot_name)
    if robot is None:
        raise ValueError(f"Unknown robot: '{robot_name}'")
    if stack_name not in robot.get('stacks', []):
        raise ValueError(f"Robot '{robot_name}' does not support stack '{stack_name}'")

    mode = get_mode()
    components = resolve_components(manifest, stack_name, mode=mode)

    # Apply robot's component_excludes
    excludes = set(robot.get('component_excludes', []))
    components = [c for c in components if c not in excludes]

    # Collect stack overrides (walk the extends chain)
    overrides = _collect_overrides(manifest, stack_name)

    component_set = set(components)

    result = {}
    for comp_name in components:
        comp = get_component(manifest, comp_name)

        # Apply stack overrides for this component
        comp_overrides = overrides.get(comp_name, {})

        kind = comp.get('kind', 'node')
        service = None if kind == 'virtual' else component_to_service(manifest, robot_name, comp_name)
        after = comp_overrides.get('after', comp.get('after', []))

        # Strip depends that were excluded (e.g. slam_bot excludes controller_manager
        # but controller_spawner still lists it in after:)
        after = [dep for dep in after if dep in component_set]

        result[comp_name] = {
            'service': service,
            'gates': list(comp.get('gates', [])),
            'depends': after,
            'timeout': float(comp.get('timeout', 30)),
            'critical': comp.get('critical', True),
            'is_bridge': kind == 'bridge',
        }

    # Always add test_drive if not already present and not disabled
    td_comp = get_component(manifest, 'test_drive')
    if 'test_drive' not in result and not td_comp.get('disabled', False):
        result['test_drive'] = {
            'service': 'test-drive',
            'gates': [],
            'depends': ['ekf_localization'],
            'timeout': 10.0,
            'critical': True,
        }

    return result


def _collect_overrides(manifest: Dict, stack_name: str) -> Dict[str, Dict]:
    """Walk the extends chain and merge overrides (child wins)."""
    stacks = manifest['stacks']
    stack = stacks.get(stack_name, {})
    merged = {}
    # Parent overrides first
    for parent in stack.get('extends', []):
        merged.update(_collect_overrides(manifest, parent))
    # Own overrides last (child wins)
    merged.update(stack.get('overrides', {}))
    return merged


def get_reset_conflicts(manifest: Dict, robot_name: str, stack_name: str) -> List[str]:
    """Derive conflicting docker-compose services for a robot:stack.

    For each conflicting stack (same robot only), returns services that are
    in the conflicting stack but NOT in our stack. These need `docker compose rm -f`.
    Extra items are harmless (rm of non-existent containers is a no-op).
    """
    our_services = set(resolve_services(manifest, robot_name, stack_name))
    conflict_services = []

    for conf_robot, conf_stack in get_conflicts(manifest, robot_name, stack_name):
        if conf_robot != robot_name:
            continue  # Only same-robot conflicts matter for service cleanup
        try:
            their_services = resolve_services(manifest, conf_robot, conf_stack)
        except ValueError:
            continue
        for svc in their_services:
            if svc not in our_services and svc not in conflict_services:
                conflict_services.append(svc)

    return conflict_services


def build_all_stacks(manifest: Dict) -> Dict[str, List[str]]:
    """Build a dict of target → service list for all robot:stack combos.
    Used by status and clean commands."""
    mode = get_mode()
    result = {}
    for robot_name, robot in manifest['robots'].items():
        for stack_name in robot.get('stacks', []):
            key = f"{robot_name}:{stack_name}"
            try:
                result[key] = resolve_services(manifest, robot_name, stack_name, mode=mode)
            except ValueError:
                pass

    # Add non-robot stacks (no robot prefix, just hyphenated component names)
    for stack_name, stack in manifest['stacks'].items():
        claimed = any(
            stack_name in r.get('stacks', [])
            for r in manifest['robots'].values()
        )
        if not claimed:
            components = resolve_components(manifest, stack_name, mode=mode)
            # Filter out virtual, convert underscores to hyphens
            services = []
            for c in components:
                comp = get_component(manifest, c)
                if comp.get('kind') != 'virtual':
                    services.append(c.replace('_', '-'))
            result[stack_name] = services
    return result


# ── Docker helpers ──────────────────────────────────────────────────────────

def run_cmd(cmd: List[str], check: bool = True) -> subprocess.CompletedProcess:
    """Run a command and return the result."""
    return subprocess.run(cmd, capture_output=True, text=True, check=check)


def get_running_services() -> List[str]:
    """Get list of currently running docker compose services."""
    result = run_cmd(['docker', 'compose', 'ps', '--format', 'json'], check=False)
    if result.returncode != 0:
        return []

    services = []
    for line in result.stdout.strip().split('\n'):
        if not line:
            continue
        try:
            data = json.loads(line)
            if data.get('State') == 'running':
                services.append(data.get('Service', ''))
        except json.JSONDecodeError:
            continue
    return services


def verify_containers_running(services: List[str]) -> Tuple[bool, List[str]]:
    """Verify containers are actually running using docker ps."""
    result = run_cmd(['docker', 'ps', '--format', '{{.Names}}'], check=False)
    if result.returncode != 0:
        return False, services
    running = set(result.stdout.strip().split('\n'))
    not_running = [s for s in services if s not in running]
    return len(not_running) == 0, not_running


# ── Stack operations ────────────────────────────────────────────────────────


def get_infra_services(manifest: Dict) -> List[str]:
    """Derive infrastructure service names from the infra stack in YAML."""
    stacks = manifest.get('stacks', {})
    infra = stacks.get('infra', {})
    components = infra.get('include', [])
    return [c.replace('_', '-') for c in components]


def print_status():
    """Print current stack status."""
    manifest = load_manifest()
    all_stacks = build_all_stacks(manifest)
    running = get_running_services()

    print(f"\n{YELLOW}=== Stack Status ==={NC}\n")

    for target, services in all_stacks.items():
        running_in_stack = [s for s in services if s in running]
        if running_in_stack:
            print(f"{GREEN}[OK] {target}{NC}: {', '.join(running_in_stack)}")
        else:
            print(f"  {target}: (stopped)")

    # Show unknown running services
    known = set(s for services in all_stacks.values() for s in services)
    unknown = [s for s in running if s not in known]
    if unknown:
        print(f"\n{YELLOW}Other services:{NC} {', '.join(unknown)}")

    print()


def start_stack(target: str):
    """Start a robot:stack target."""
    manifest = load_manifest()
    robot, stack = parse_target(target)

    services = resolve_services(manifest, robot, stack)
    print(f"{YELLOW}Starting {target} ({len(services)} services)...{NC}")

    # Apply stack-level env vars early (before dependency starts, so containers
    # like test-drive pick up NEUTER=1 from docker compose env substitution)
    stack_def = manifest['stacks'][stack]
    for key, val in stack_def.get('env', {}).items():
        os.environ[key] = str(val)

    # Collect services excluded by the target stack (so parents don't start them)
    target_services = set(services)

    # Start dependencies first (parent stacks)
    for dep_robot, dep_stack in get_stack_dependencies(manifest, robot, stack):
        dep_target = f"{dep_robot}:{dep_stack}"
        dep_services = resolve_services(manifest, dep_robot, dep_stack)
        # Filter out services excluded by the target stack
        dep_services = [s for s in dep_services if s in target_services]
        running = get_running_services()
        if not any(s in running for s in dep_services):
            print(f"{YELLOW}Starting dependency {dep_target}...{NC}")
            cmd = ['docker', 'compose', 'up', '-d'] + dep_services
            result = run_cmd(cmd, check=False)
            if result.returncode != 0:
                print(f"{RED}ERROR: Failed to start dependency {dep_target}{NC}")
                print(result.stderr)
                sys.exit(1)
            time.sleep(2)

    # Check for conflicts
    running = get_running_services()
    for conf_robot, conf_stack in get_conflicts(manifest, robot, stack):
        unique = resolve_unique_services(manifest, conf_robot, conf_stack)
        running_conflicts = [s for s in unique if s in running]
        if running_conflicts:
            conf_target = f"{conf_robot}:{conf_stack}"
            print(f"{RED}ERROR: Cannot start '{target}' - conflicts with running '{conf_target}'{NC}")
            print(f"{YELLOW}Conflicting services: {', '.join(running_conflicts)}{NC}")
            print(f"\n{YELLOW}Solution: python stack.py stop {conf_target}{NC}")
            sys.exit(1)

    # Check if already running
    already_running = [s for s in services if s in running]
    if already_running:
        print(f"{YELLOW}Note: Some services already running: {', '.join(already_running)}{NC}")

    # Start
    cmd = ['docker', 'compose', 'up', '-d'] + services
    result = run_cmd(cmd, check=False)
    if result.returncode != 0:
        print(f"{RED}ERROR: Failed to start {target}{NC}")
        print(result.stderr)
        sys.exit(1)

    print(f"{GREEN}[OK] {target} started{NC}")
    time.sleep(2)
    print_status()

    # Verify
    all_running, not_running = verify_containers_running(services)
    if not all_running:
        print(f"{RED}WARNING: Some services failed to start:{NC}")
        for svc in not_running:
            print(f"{RED}  - {svc}{NC}")


def stop_stack(target: str):
    """Stop a robot:stack target."""
    manifest = load_manifest()
    robot, stack = parse_target(target)

    services = resolve_services(manifest, robot, stack)
    print(f"{YELLOW}Stopping {target}...{NC}")

    # For stacks with extends, only stop own services (not the base)
    stacks_def = manifest['stacks'].get(stack, {})
    if stacks_def.get('extends'):
        own_services = resolve_unique_services(manifest, robot, stack)
        if own_services:
            # Stop only the additive services, preserve base
            cmd = ['docker', 'compose', 'stop'] + own_services
        else:
            cmd = ['docker', 'compose', 'stop'] + services
    else:
        cmd = ['docker', 'compose', 'stop'] + services

    result = run_cmd(cmd, check=False)
    if result.returncode != 0:
        print(f"{RED}ERROR: Failed to stop {target}{NC}")
        print(result.stderr)
        sys.exit(1)

    # Check for stragglers
    time.sleep(1)
    running = get_running_services()
    infra = set(get_infra_services(manifest))
    stragglers = [s for s in running if s not in infra]

    if stragglers:
        print(f"{YELLOW}WARNING: Stragglers detected:{NC}")
        for svc in stragglers:
            print(f"  - {svc}")
    else:
        print(f"{GREEN}[OK] {target} stopped{NC}")


def restart_stack(target: str):
    """Restart a robot:stack target."""
    print(f"{YELLOW}Restarting {target}...{NC}\n")
    stop_stack(target)
    print()
    start_stack(target)


def soft_reset():
    """Warm-reset the running sim via teleport (Option 2).

    Uses set_pose to teleport the model back to origin instead of
    reset:{all:true} (which destroys entities/plugins/sim time) or
    reset:{model_only:true} (which is a NO-OP in gz-sim8 Harmonic).

    Preserves: entity IDs, plugin instances, sim time continuity.

    Phases:
      1. Pause Gazebo physics (freezes /clock, all sim_time nodes stop)
      2. Teleport model to initial pose via set_pose service
      3. Zero all joint commands (steering position + wheel velocity)
      4. Unpause Gazebo physics (PID drives steering joints to zero)
    """
    manifest = load_manifest()

    # Detect running stack to find Gazebo container
    detected = _detect_running_stack(manifest)
    if detected is None:
        print(f"{RED}ERROR: No running stack detected{NC}")
        sys.exit(1)

    robot_name, stack_name, _ = detected
    gazebo_service = component_to_service(manifest, robot_name, 'gazebo')

    # Resolve world name from .env
    _load_dotenv()
    world_file = os.environ.get('JETACKER_WORLD', 'jetacker/jetacker.sdf')
    # World name in Gazebo = filename stem of the .sdf
    world_name = Path(world_file).stem  # e.g. "jetacker" from "jetacker/jetacker.sdf"
    # Our world element is named jetacker_world (defined in the SDF)
    gz_world = f'{world_name}_world'
    model_name = robot_name  # SDF <model name="jetacker"> matches robot name

    # Windows Git Bash needs MSYS_NO_PATHCONV to avoid mangling /world/... paths
    env = os.environ.copy()
    env['MSYS_NO_PATHCONV'] = '1'

    def gz_exec(gz_cmd: str, desc: str) -> bool:
        """Run a gz command inside the Gazebo container."""
        t_step = time.time()
        cmd = [
            'docker', 'compose', 'exec', '-T', gazebo_service,
            'bash', '-c',
            f'source /opt/ros/jazzy/setup.bash && {gz_cmd}'
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, env=env)
        dt = time.time() - t_step
        if result.returncode == 0:
            print(f"  {GREEN}[OK]{NC} {desc} ({dt:.1f}s)")
            return True
        else:
            stderr = result.stderr[:200] if result.stderr else '(no output)'
            print(f"  {RED}[X]{NC} {desc} ({dt:.1f}s): {stderr}")
            return False

    def world_control(req: str, desc: str) -> bool:
        """Call world control service (pause/unpause)."""
        return gz_exec(
            f'gz service '
            f'-s /world/{gz_world}/control '
            f'--reqtype gz.msgs.WorldControl '
            f'--reptype gz.msgs.Boolean '
            f'--timeout 5000 '
            f'--req "{req}"',
            desc
        )

    # Path to joint_reset binary (built alongside WorldStatePublisher plugin)
    joint_reset_bin = '/workspace/gz_plugins/world_state_publisher/build/joint_reset'

    print(f"{YELLOW}Soft reset ({robot_name}:{stack_name})...{NC}")
    t0 = time.time()

    # All phases in a single docker exec to minimize overhead (~1.7s total)
    gz_cmds = ' && '.join([
        # Phase 1: Pause physics
        f'gz service -s /world/{gz_world}/control '
        f'--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean '
        f'--timeout 5000 --req "pause: true"',
        # Phase 2: Teleport model to initial pose (from SDF: 0 0 0.05 0 0 0)
        f'gz service -s /world/{gz_world}/set_pose '
        f'--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean '
        f"""--timeout 5000 --req 'name: "{model_name}" """
        f"""position {{ x: 0 y: 0 z: 0.05 }} orientation {{ w: 1.0 }}'""",
        # Phase 3: Zero all joint commands via compiled helper
        f'{joint_reset_bin} {model_name}',
        # Phase 4: Unpause physics
        f'gz service -s /world/{gz_world}/control '
        f'--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean '
        f'--timeout 5000 --req "pause: false"',
    ])

    ok = gz_exec(gz_cmds, 'Pause > teleport > zero joints > unpause')
    if not ok:
        # Try to unpause in case we failed mid-sequence
        gz_exec(
            f'gz service -s /world/{gz_world}/control '
            f'--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean '
            f'--timeout 5000 --req "pause: false"',
            'Unpause (recovery)'
        )
        print(f"{RED}Soft reset failed{NC}")
        sys.exit(1)

    # Reset downstream state in parallel (EKF, SLAM, Nav2 costmaps)
    # These are independent services in separate containers.
    _, _, running_services = detected
    downstream_procs = []  # (Popen, description)
    t_downstream = time.time()

    # EKF pose reset
    ekf_service = component_to_service(manifest, robot_name, 'ekf_localization')
    if ekf_service in running_services:
        p = subprocess.Popen(
            ['docker', 'compose', 'exec', '-T', ekf_service, 'bash', '-c',
             'source /opt/ros/jazzy/setup.bash && '
             'source /overlay_ws/install/setup.bash && '
             'ros2 service call /set_pose robot_localization/srv/SetPose '
             '"{pose: {header: {frame_id: odom}, pose: {pose: '
             '{position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}}"'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)
        downstream_procs.append((p, 'Reset EKF pose'))

    # slam_toolbox map reset
    slam_service = component_to_service(manifest, robot_name, 'slam_toolbox')
    if slam_service in running_services:
        p = subprocess.Popen(
            ['docker', 'compose', 'exec', '-T', slam_service, 'bash', '-c',
             'source /opt/ros/jazzy/setup.bash && '
             'ros2 service call /slam_toolbox/reset slam_toolbox/srv/Reset '
             '"{pause_new_measurements: false}"'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)
        downstream_procs.append((p, 'Reset slam_toolbox map'))

    # AMCL particle filter reset — deferred until after EKF/TF settle (see below)
    amcl_service = component_to_service(manifest, robot_name, 'amcl')

    # Nav2: clear costmaps
    # Goal cancellation is handled by nav2_waypoint_follower.py at startup
    # (using its existing DDS participant — zero discovery overhead).
    bt_service = component_to_service(manifest, robot_name, 'bt_navigator')
    if bt_service in running_services:
        p = subprocess.Popen(
            ['docker', 'compose', 'exec', '-T', bt_service, 'bash', '-c',
             'source /opt/ros/jazzy/setup.bash && '
             'ros2 service call /global_costmap/clear_entirely_global_costmap '
             'nav2_msgs/srv/ClearEntireCostmap "{}" & '
             'ros2 service call /local_costmap/clear_entirely_local_costmap '
             'nav2_msgs/srv/ClearEntireCostmap "{}" & '
             'wait'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)
        downstream_procs.append((p, 'Clear costmaps'))

    # Wait for all downstream resets
    for p, desc in downstream_procs:
        p.wait()
        dt = time.time() - t_downstream
        if p.returncode == 0:
            print(f"  {GREEN}[OK]{NC} {desc} ({dt:.1f}s)")
        else:
            print(f"  {YELLOW}WARNING{NC} {desc} failed ({dt:.1f}s)")

    # AMCL particle filter reset — MUST run after EKF/TF settle.
    # If published in parallel, AMCL snaps to origin then immediately drifts
    # back to the pre-teleport position when it gets a scan with stale TF.
    if amcl_service in running_services:
        time.sleep(1.0)  # Let EKF publish corrected odom->base_link TF
        t_amcl = time.time()
        amcl_cmd = [
            'docker', 'compose', 'exec', '-T', amcl_service, 'bash', '-c',
            'source /opt/ros/jazzy/setup.bash && '
            'ros2 topic pub --once /initialpose '
            'geometry_msgs/msg/PoseWithCovarianceStamped '
            '"{header: {frame_id: map}, pose: {pose: '
            '{position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"']
        result = subprocess.run(amcl_cmd, capture_output=True, text=True, env=env)
        dt = time.time() - t_amcl
        if result.returncode == 0:
            print(f"  {GREEN}[OK]{NC} Reset AMCL pose (deferred) ({dt:.1f}s)")
        else:
            print(f"  {YELLOW}WARNING{NC} Reset AMCL pose failed ({dt:.1f}s)")

    elapsed = time.time() - t0
    print(f"\n{GREEN}[OK] Soft reset complete ({elapsed:.1f}s){NC}")


def clean_all():
    """Stop all stacks except infrastructure."""
    manifest = load_manifest()
    all_stacks = build_all_stacks(manifest)

    print(f"{YELLOW}Stopping all stacks (preserving infrastructure)...{NC}")

    for target, services in all_stacks.items():
        if target == 'infra':
            continue
        if services:
            run_cmd(['docker', 'compose', 'stop'] + services, check=False)

    time.sleep(1)
    running = get_running_services()
    infra_svcs = set(get_infra_services(manifest))
    non_infra = [s for s in running if s not in infra_svcs]

    if non_infra:
        print(f"{RED}WARNING: Some services still running: {', '.join(non_infra)}{NC}")
    else:
        print(f"{GREEN}[OK] All stacks stopped (infrastructure preserved){NC}")

    print_status()


def force_clean():
    """Force stop and remove all non-infrastructure containers."""
    manifest = load_manifest()
    print(f"{YELLOW}Force cleaning all stacks (preserving infrastructure)...{NC}")

    result = run_cmd(['docker', 'ps', '-a', '--format', '{{.Names}}'], check=False)
    if result.returncode != 0:
        print(f"{RED}ERROR: Failed to list containers{NC}")
        return

    all_containers = result.stdout.strip().split('\n')
    infra_svcs = set(get_infra_services(manifest))

    # Build set of all known stack services from YAML
    all_stacks = build_all_stacks(manifest)
    known_services = set(s for services in all_stacks.values() for s in services)

    to_stop = [
        c for c in all_containers
        if c and c not in infra_svcs
        and c in known_services
    ]

    if to_stop:
        print(f"{YELLOW}Stopping containers: {', '.join(to_stop)}{NC}")
        run_cmd(['docker', 'stop'] + to_stop, check=False)
        time.sleep(2)
        print(f"{YELLOW}Removing containers...{NC}")
        run_cmd(['docker', 'rm', '-f'] + to_stop, check=False)
        print(f"{GREEN}[OK] Force clean complete{NC}")
    else:
        print(f"{GREEN}No containers to clean{NC}")

    print_status()


# ── Audit ──────────────────────────────────────────────────────────────────

def _detect_running_stack(manifest: Dict) -> Optional[Tuple[str, str, List[str]]]:
    """Detect which robot:stack is currently running.

    Returns (robot, stack, running_services) or None if nothing detected."""
    running = get_running_services()
    if not running:
        return None

    running_set = set(running)
    best_match = None
    best_count = 0

    for robot_name, robot in manifest['robots'].items():
        for stack_name in robot.get('stacks', []):
            try:
                services = resolve_services(manifest, robot_name, stack_name)
            except ValueError:
                continue
            match_count = len(running_set & set(services))
            if match_count > best_count:
                best_count = match_count
                best_match = (robot_name, stack_name, running)

    return best_match


def audit():
    """Compare expected ROS2 node names (from stacks.yaml) with actual running nodes."""
    manifest = load_manifest()

    # Detect running stack
    detected = _detect_running_stack(manifest)
    if detected is None:
        print(f"{RED}ERROR: No running stack detected{NC}")
        sys.exit(1)

    robot_name, stack_name, running_services = detected
    print(f"{YELLOW}Detected stack: {robot_name}:{stack_name}{NC}\n")

    # Resolve components for this stack
    components = resolve_components(manifest, stack_name)
    excludes = set(manifest['robots'][robot_name].get('component_excludes', []))
    components = [c for c in components if c not in excludes]

    # Query actual ROS2 nodes via test-drive container
    result = run_cmd(
        ['docker', 'compose', 'exec', '-T', 'test-drive',
         'bash', '-c', 'source /opt/ros/jazzy/setup.bash && ros2 node list 2>/dev/null'],
        check=False
    )
    if result.returncode != 0:
        # Fallback: try debug container
        result = run_cmd(
            ['docker', 'compose', 'exec', '-T', 'debug',
             'bash', '-c', 'source /opt/ros/jazzy/setup.bash && ros2 node list 2>/dev/null'],
            check=False
        )

    if result.returncode != 0:
        print(f"{RED}ERROR: Could not query ros2 node list (is a container running?){NC}")
        sys.exit(1)

    actual_nodes = set()
    for line in result.stdout.strip().split('\n'):
        line = line.strip()
        if line.startswith('/'):
            actual_nodes.add(line.lstrip('/'))

    # Build expected → actual comparison
    matched_actual = set()
    rows = []

    for comp_name in components:
        comp = get_component(manifest, comp_name)
        kind = comp.get('kind', 'node')

        if kind in ('virtual', 'infra'):
            continue

        expected_node = comp.get('node', comp_name)
        service = component_to_service(manifest, robot_name, comp_name)

        # Check if service is running
        if service not in running_services:
            rows.append((comp_name, service, expected_node, '-', 'STOPPED'))
            continue

        # Check if node exists in ros2 node list
        if expected_node in actual_nodes:
            rows.append((comp_name, service, expected_node, expected_node, 'OK'))
            matched_actual.add(expected_node)
        else:
            # Check for partial match (node might have namespace prefix)
            found = None
            for actual in actual_nodes:
                if actual.endswith(expected_node):
                    found = actual
                    break
            if found:
                rows.append((comp_name, service, expected_node, found, 'OK'))
                matched_actual.add(found)
            else:
                rows.append((comp_name, service, expected_node, '???', 'MISSING'))

    # Print table
    hdr = f"{'Component':<28} {'Service':<35} {'Expected Node':<30} {'Actual Node':<30} {'Status'}"
    print(hdr)
    print('─' * len(hdr))

    ok_count = 0
    issue_count = 0
    for comp_name, service, expected, actual, status in rows:
        if status == 'OK':
            color = GREEN
            ok_count += 1
        elif status == 'STOPPED':
            color = YELLOW
        else:
            color = RED
            issue_count += 1
        print(f"{comp_name:<28} {service:<35} {expected:<30} {actual:<30} {color}{status}{NC}")

    # List unmatched ROS2 nodes
    unmatched = actual_nodes - matched_actual
    if unmatched:
        print(f"\n{YELLOW}Unmatched ROS2 nodes (internal/controllers):{NC}")
        for node in sorted(unmatched):
            print(f"  /{node}")

    print(f"\n{GREEN}{ok_count} OK{NC}", end='')
    if issue_count:
        print(f", {RED}{issue_count} issues{NC}")
    else:
        print()


# ── CLI ─────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        manifest = load_manifest()
        print(f"Available targets: {', '.join(list_targets(manifest))}")
        sys.exit(1)

    command = sys.argv[1].lower()

    if command == 'status':
        print_status()
    elif command == 'soft-reset':
        soft_reset()
    elif command == 'clean':
        clean_all()
    elif command == 'force-clean':
        force_clean()
    elif command == 'audit':
        audit()
    elif command in ('start', 'stop', 'restart'):
        if len(sys.argv) < 3:
            manifest = load_manifest()
            print(f"{RED}ERROR: Target required{NC}")
            print(f"Usage: python stack.py {command} robot:stack")
            print(f"Available: {', '.join(list_targets(manifest))}")
            sys.exit(1)

        target = sys.argv[2].lower()

        if command == 'start':
            start_stack(target)
        elif command == 'stop':
            stop_stack(target)
        elif command == 'restart':
            restart_stack(target)
    else:
        print(f"{RED}ERROR: Unknown command '{command}'{NC}")
        print(__doc__)
        sys.exit(1)


if __name__ == '__main__':
    main()
