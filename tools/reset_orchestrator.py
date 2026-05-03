#!/usr/bin/env python3
"""
Reset Orchestrator - Host-side reset coordination.

Manages docker compose lifecycle and calls container orchestrator for gates.

Architecture:
- Validates dependency graph (DAG) at initialization
- Computes topological sort for deterministic ordering
- Starts/stops docker compose services in dependency order
- Calls reset_gates.py for readiness verification
- Supports Gazebo world reset without container restart

Usage:
    from reset_orchestrator import ResetOrchestrator
    from stack import load_manifest, build_reset_definition, get_reset_conflicts

    manifest = load_manifest()
    stack_def = build_reset_definition(manifest, 'jetacker', 'nav2')
    conflicts = get_reset_conflicts(manifest, 'jetacker', 'nav2')
    orchestrator = ResetOrchestrator(stack_def, conflicting_services=conflicts)
    success = orchestrator.reset_stack_full()
"""

import subprocess
import time
import json
from typing import Dict, List, Any
from collections import deque


class ResetOrchestrator:
    """
    Host-side reset orchestrator.

    Responsibilities:
    - Start/stop docker compose services in dependency order
    - Call Gazebo world reset service
    - Invoke container orchestrator for readiness gates
    - Handle errors and provide structured logging
    """

    def __init__(self, stack_definition: Dict[str, Dict[str, Any]], stack_name: str = None,
                 conflicting_services: List[str] = None):
        """
        Initialize orchestrator with stack definition.

        Args:
            stack_definition: Dict mapping node names to config
            stack_name: Optional stack name for conflict resolution (e.g., "JETACKER_NAV2_ODOM")
            conflicting_services: List of conflicting service names to stop before reset.

        Raises:
            ValueError: If stack definition has circular dependencies
        """
        self.stack = stack_definition
        self.stack_name = stack_name
        self.validate_dag()

        # Extract bridge services from stack definition
        self.bridge_services = [
            node['service'] for node in self.stack.values()
            if node.get('is_bridge', False) and node.get('service') is not None
        ]

        # Conflicting services must be passed explicitly by the caller
        self.conflicting_services = conflicting_services or []

    def validate_dag(self):
        """
        Validate dependency graph has no cycles.

        Raises:
            ValueError: If circular dependency or unknown dependency found
        """
        visited = set()
        rec_stack = set()

        def has_cycle(node):
            visited.add(node)
            rec_stack.add(node)

            for dep in self.stack[node].get('depends', []):
                if dep not in self.stack:
                    raise ValueError(f"Unknown dependency: {node} depends on {dep}")
                if dep not in visited:
                    if has_cycle(dep):
                        return True
                elif dep in rec_stack:
                    raise ValueError(f"Circular dependency: {node} → {dep}")

            rec_stack.remove(node)
            return False

        for node in self.stack:
            if node not in visited:
                has_cycle(node)

    def topological_sort(self) -> List[str]:
        """
        Compute dependency-respecting startup order using Kahn's algorithm.

        Returns:
            List of node names in dependency order (dependencies first)

        Raises:
            ValueError: If cycle detected in dependency graph
        """
        in_degree = {node: len(self.stack[node].get('depends', []))
                    for node in self.stack}

        queue = deque([node for node in self.stack if in_degree[node] == 0])
        sorted_nodes = []

        while queue:
            node = queue.popleft()
            sorted_nodes.append(node)

            for other_node in self.stack:
                if node in self.stack[other_node].get('depends', []):
                    in_degree[other_node] -= 1
                    if in_degree[other_node] == 0:
                        queue.append(other_node)

        if len(sorted_nodes) != len(self.stack):
            raise ValueError("Cycle detected in dependency graph")

        return sorted_nodes

    def docker_compose_start(self, service: str) -> bool:
        """
        Start single docker compose service (creates container if it doesn't exist).

        Args:
            service: Docker compose service name

        Returns:
            True if service started successfully
        """
        # Use 'up -d' instead of 'start' to create containers if they don't exist
        # This is needed for profile-based services that may not have been created yet
        cmd = ['docker', 'compose', 'up', '-d', '--no-recreate', service]
        result = subprocess.run(cmd, capture_output=True, text=True)
        return result.returncode == 0

    def docker_compose_stop(self, service: str) -> bool:
        """
        Stop single docker compose service.

        Args:
            service: Docker compose service name

        Returns:
            True if service stopped successfully
        """
        cmd = ['docker', 'compose', 'stop', service]
        result = subprocess.run(cmd, capture_output=True, text=True)
        return result.returncode == 0

    def check_gates_for_node(self, node_name: str) -> bool:
        """
        Check all gates for a node.

        Routes TF gates to TF service, others to reset_gates.py (temporary).

        Args:
            node_name: Node name from stack definition

        Returns:
            True if all gates passed, False otherwise
        """
        node = self.stack[node_name]
        gates_spec = node.get('gates', [])

        if not gates_spec:
            return True  # No gates to check

        timeout = node.get('timeout', 30.0)

        # Check each gate
        for gate_spec in gates_spec:
            parts = gate_spec.split(':')
            gate_name = parts[0]
            gate_args = parts[1:] if len(parts) > 1 else []

            # Print what we're checking BEFORE checking it
            gate_desc = f"{gate_name}:{':'.join(gate_args)}" if gate_args else gate_name
            print(f"      Checking: {gate_desc} (timeout: {timeout:.0f}s)")

            # Route TF gates to TF service
            if gate_name in ['tf_transform', 'tf_static']:
                success = self._check_tf_gate(gate_name, gate_args, timeout)
            else:
                # Fall back to old reset_gates.py for non-TF gates (temporary)
                success = self._check_gate_legacy(gate_spec, timeout)

            if not success:
                return False

        return True

    def _check_tf_gate(self, gate_name: str, gate_args: list, timeout: float) -> bool:
        """
        Check TF gate via TF gate checker service.

        Args:
            gate_name: 'tf_transform' or 'tf_static'
            gate_args: Gate arguments (e.g., ['odom', 'base_link'])
            timeout: Timeout in seconds

        Returns:
            True if gate passed, False otherwise
        """
        if gate_name == 'tf_transform':
            if len(gate_args) != 2:
                print(f"    [X] tf_transform requires 2 args (parent, child), got {len(gate_args)}")
                return False

            parent, child = gate_args
            return self._call_tf_service(parent, child, timeout)

        elif gate_name == 'tf_static':
            # tf_static not yet migrated to service
            # TODO: Implement tf_static service check
            print(f"    [WARN] tf_static not yet migrated to service, skipping")
            return True

        return False

    def _call_tf_service(self, parent: str, child: str, timeout: float) -> bool:
        """
        Check TF transform via fresh process invocation.

        Uses check_tf_once.py which spawns a fresh node per check.
        This avoids persistent service warmup issues while maintaining TF isolation.

        Args:
            parent: Parent frame
            child: Child frame
            timeout: Timeout in seconds

        Returns:
            True if transform available, False otherwise
        """
        import os

        # Use MSYS_NO_PATHCONV for Windows Git Bash compatibility
        env = os.environ.copy()
        env['MSYS_NO_PATHCONV'] = '1'

        cmd = [
            'docker', 'compose', 'exec', '-T', 'test-drive',
            'bash', '-c',
            f'source /opt/ros/jazzy/setup.bash && '
            f'python3 /workspace/ros/check_tf_once.py '
            f'--parent {parent} --child {child} --timeout {timeout} --json'
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True,
                                   timeout=timeout + 5.0, env=env)

            if result.returncode == 0:
                # Parse JSON response (skip RTPS/DDS C++ error lines that leak to stdout)
                stdout = result.stdout
                json_start = stdout.index('{')
                output = json.loads(stdout[json_start:])
                success = output.get('success', False)
                duration = output.get('duration', 0.0)

                if success:
                    print(f"      [OK] tf_transform:{parent}:{child} ({duration:.1f}s)")
                    return True
                else:
                    print(f"      [X] tf_transform:{parent}:{child} ({duration:.1f}s)")
                    return False
            else:
                print(f"    [X] TF check failed with exit code {result.returncode}")
                if result.stderr:
                    print(f"      Error: {result.stderr[:200]}")
                return False

        except subprocess.TimeoutExpired:
            print(f"    [X] TF check timeout (>{timeout}s)")
            return False
        except (json.JSONDecodeError, ValueError) as e:
            print(f"    [X] TF check JSON parse error: {e}")
            return False
        except Exception as e:
            print(f"    [X] TF check error: {e}")
            return False

    def _check_gate_legacy(self, gate_spec: str, timeout: float) -> bool:
        """
        Check gate using legacy reset_gates.py (temporary).

        Args:
            gate_spec: Gate specification string
            timeout: Timeout in seconds

        Returns:
            True if gate passed, False otherwise
        """
        import os

        # Use MSYS_NO_PATHCONV for Windows Git Bash compatibility
        env = os.environ.copy()
        env['MSYS_NO_PATHCONV'] = '1'

        cmd = [
            'docker', 'compose', 'exec', '-T', 'test-drive',
            'bash', '-c',
            f'source /opt/ros/jazzy/setup.bash && '
            f'python3 /workspace/ros/reset_gates.py --gates {gate_spec} --timeout {timeout} --json'
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True,
                                   timeout=timeout + 10.0, env=env)

            # Parse JSON output (skip RTPS/DDS C++ error lines that leak to stdout)
            stdout = result.stdout
            try:
                json_start = stdout.index('{')
                output = json.loads(stdout[json_start:])
                success = output.get('success', False)

                # Log gate result
                if output.get('gates'):
                    gate_result = output['gates'][0]
                    status = '[OK]' if gate_result['success'] else '[X]'
                    gate_name = gate_result['name']
                    gate_args_str = ':'.join(gate_result.get('args', []))
                    gate_desc = f"{gate_name}:{gate_args_str}" if gate_args_str else gate_name
                    duration = gate_result.get('duration', 0)
                    print(f"      {status} {gate_desc} ({duration:.1f}s)")

                return success
            except (json.JSONDecodeError, ValueError):
                # No valid JSON — treat as infrastructure failure
                print(f"    [X] Gate check failed with exit code {result.returncode}")
                if result.stderr:
                    lines = result.stderr.strip().splitlines()[-3:]
                    for line in lines:
                        print(f"        {line}")
                return False

        except subprocess.TimeoutExpired:
            print(f"    [X] Gate check timeout (>{timeout}s)")
            return False
        except json.JSONDecodeError as e:
            print(f"    [X] Gate check JSON parse error: {e}")
            return False
        except Exception as e:
            print(f"    [X] Gate check error: {e}")
            return False

    def reset_gazebo_world(self) -> bool:
        """
        Reset Gazebo world using gz service call.

        Uses 'reset: {all: true}' which resets all models and physics state.
        Experiments confirmed all reset strategies work reliably:
        - baseline (reset: {all: true}): PASS
        - stop_bridges: PASS
        - pause_first: PASS
        - softer_reset (model_only): PASS
        - combined: PASS

        Using baseline for simplicity and speed (~1-2s vs 5.5s container restart).

        Returns:
            True if Gazebo reset successfully
        """
        import os

        print("  Calling gz service reset...")

        # Use MSYS_NO_PATHCONV for Windows Git Bash compatibility
        env = os.environ.copy()
        env['MSYS_NO_PATHCONV'] = '1'

        cmd = [
            'docker', 'compose', 'exec', '-T', 'jetacker-gazebo',
            'bash', '-c',
            'source /opt/ros/jazzy/setup.bash && '
            'gz service '
            '-s /world/jetacker_world/control '
            '--reqtype gz.msgs.WorldControl '
            '--reptype gz.msgs.Boolean '
            '--timeout 5000 '
            '--req "reset: {all: true}"'
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True,
                                   timeout=10.0, env=env)

            if result.returncode == 0:
                print("  [OK] Gazebo world reset")
                # Brief settling time for physics to stabilize
                time.sleep(1.0)
                return True
            else:
                print(f"  [X] Gazebo reset failed: {result.stderr[:200]}")
                return False

        except subprocess.TimeoutExpired:
            print("  [X] Gazebo reset timeout (>10s)")
            return False

    def restart_bridge_services(self) -> bool:
        """
        Restart bridge services after Gazebo reset.

        Uses bridge_services extracted from stack definition.
        Stops all bridges in parallel, then starts all bridges in parallel.

        Returns:
            True if all bridge services restarted successfully
        """
        if not self.bridge_services:
            return True  # No bridges defined

        print(f"  Restarting {len(self.bridge_services)} bridge service(s)...")

        # Stop all bridges in parallel (order doesn't matter)
        print(f"    Stopping {len(self.bridge_services)} bridges in parallel...")
        cmd = ['docker', 'compose', 'stop'] + self.bridge_services
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30.0)
        if result.returncode != 0:
            print(f"    [X] Failed to stop bridges: {result.stderr[:200]}")
            return False

        # Start all bridges in parallel (docker compose handles depends_on)
        print(f"    Starting {len(self.bridge_services)} bridges in parallel...")
        # Use 'up -d' instead of 'start' to create containers if they don't exist
        cmd = ['docker', 'compose', 'up', '-d', '--no-recreate'] + self.bridge_services
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30.0)
        if result.returncode != 0:
            print(f"    [X] Failed to start bridges: {result.stderr[:200]}")
            return False

        # Brief settling time for bridges to initialize
        time.sleep(2.0)

        print("  [OK] Bridge services restarted")
        return True

    def start_node_with_gates(self, node_name: str) -> bool:
        """
        Start node and check its readiness gates.

        Args:
            node_name: Node name from stack definition

        Returns:
            True if node started and gates passed, False otherwise
        """
        node = self.stack[node_name]
        service = node.get('service')
        gates = node.get('gates', [])
        critical = node.get('critical', True)

        # Virtual nodes have no service (just gates)
        if service is not None:
            print(f"  Starting {node_name} ({service})...")
            if not self.docker_compose_start(service):
                print(f"    [X] Failed to start {service}")
                return False

        # Check readiness gates if specified
        if gates:
            print(f"    Checking {len(gates)} gate(s)...")
            success = self.check_gates_for_node(node_name)

            if success:
                print(f"    [OK] All gates passed")
                return True
            else:
                print(f"    [X] Gate checks failed")
                if critical:
                    print(f"    [X] Critical node {node_name} failed, aborting")
                    return False
                else:
                    print(f"    [WARN] Non-critical node {node_name} failed, continuing")
                    return True

        return True

    def start_nodes_parallel(self) -> bool:
        """
        Start nodes in parallel by dependency level.

        Groups nodes into levels where all nodes at same level have dependencies satisfied.
        Starts all nodes at each level in parallel, then checks gates sequentially.

        Returns:
            True if all nodes started successfully
        """
        # Compute dependency levels
        levels = self.compute_dependency_levels()

        node_counter = 0
        total_nodes = sum(len(level) for level in levels)

        for level_idx, level_nodes in enumerate(levels):
            if not level_nodes:
                continue

            # Skip test-drive if already started
            level_nodes = [n for n in level_nodes if n != 'test_drive' or level_idx == 0]

            # Start all nodes at this level in parallel
            services_to_start = []
            for node_name in level_nodes:
                node_counter += 1
                node = self.stack[node_name]
                service = node.get('service')

                if service is not None:
                    services_to_start.append((node_name, service))

            # Parallel docker compose start for all services at this level
            if services_to_start:
                service_names = [s[1] for s in services_to_start]
                print(f"  Starting {len(services_to_start)} services in parallel: {', '.join([s[0] for s in services_to_start])}")

                # Use 'up -d' instead of 'start' to create containers if they don't exist
                cmd = ['docker', 'compose', 'up', '-d', '--no-recreate'] + service_names
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=60.0)
                if result.returncode != 0:
                    print(f"    [X] Failed to start services: {result.stderr[:200]}")
                    return False

            # Now check gates for all nodes at this level (sequential is OK)
            for node_name in level_nodes:
                if node_name == 'test_drive' and level_idx > 0:
                    print(f"  [{node_counter}/{total_nodes}] {node_name} (already started)")
                    continue

                node_start = time.time()
                print(f"  [{node_counter}/{total_nodes}] {node_name}")

                # Check gates for this node
                node = self.stack[node_name]
                gates = node.get('gates', [])
                if gates:
                    print(f"    Checking {len(gates)} gate(s)...")
                    success = self.check_gates_for_node(node_name)
                    if not success:
                        print(f"    [X] Gate checks failed")
                        if node.get('critical', True):
                            print(f"    [X] Critical node {node_name} failed, aborting")
                            return False
                        else:
                            print(f"    [WARN] Non-critical node {node_name} failed, continuing")
                    else:
                        print(f"    [OK] All gates passed")

                node_duration = time.time() - node_start
                if node_duration > 1.0:  # Only print if took more than 1 second
                    print(f"    (node startup: {node_duration:.1f}s)")

        return True

    def compute_dependency_levels(self) -> list:
        """
        Group nodes into dependency levels for parallel startup.

        Returns list of lists, where each sublist contains nodes that can start in parallel.
        Level 0 = no dependencies, Level 1 = depends only on Level 0, etc.
        """
        # Build dependency graph
        remaining = set(self.stack.keys())
        levels = []

        while remaining:
            # Find all nodes whose dependencies are satisfied
            ready = []
            for node in remaining:
                deps = set(self.stack[node].get('depends', []))
                # Node is ready if all its deps are in previous levels
                already_started = set()
                for prev_level in levels:
                    already_started.update(prev_level)

                if deps.issubset(already_started) or not deps:
                    ready.append(node)

            if not ready:
                # Circular dependency or missing node
                print(f"[X] Dependency resolution failed. Remaining nodes: {remaining}")
                break

            levels.append(ready)
            remaining -= set(ready)

        return levels

    def stop_conflicting_services(self) -> bool:
        """
        Stop and remove services that conflict with this stack.

        Uses 'docker compose rm' to prevent Docker Compose from
        automatically restarting them when other services start.

        Returns:
            True if successful, False otherwise
        """
        if not self.conflicting_services:
            return True  # No conflicts to resolve

        print(f"\n[Pre-Reset] Stopping {len(self.conflicting_services)} conflicting service(s)...")
        for service in self.conflicting_services:
            print(f"  Removing {service}...")
            # Stop first
            subprocess.run(
                ["docker", "compose", "stop", service],
                capture_output=True,
                text=True
            )
            # Then remove to prevent auto-restart
            result = subprocess.run(
                ["docker", "compose", "rm", "-f", service],
                capture_output=True,
                text=True
            )
            if result.returncode != 0:
                print(f"  [WARN] Failed to remove {service}: {result.stderr}")
        print("  [OK] Conflicting services removed")
        return True

    def reset_stack_full(self) -> bool:
        """
        Full deterministic reset sequence.

        0. Stop conflicting services (if any)
        1. Stop all nodes (reverse dependency order)
        2. Reset Gazebo world (while nodes stopped)
        3. Restart bridge services if defined in stack
        4. Start all nodes (dependency order with gates)

        Returns:
            True if all steps successful, False otherwise
        """
        reset_start_time = time.time()

        print("=" * 60)
        print("FULL STACK RESET")
        print("=" * 60)

        # Stop conflicting services first
        self.stop_conflicting_services()

        # Phase 1: Stop and remove all nodes (except Gazebo)
        phase1_start = time.time()
        print("\n[Phase 1/4] Stopping all nodes...")

        # Collect all services to stop (order doesn't matter for shutdown)
        # Skip Gazebo — Phase 2 needs it running for gz service world reset
        services_to_stop = []
        for node in self.stack:
            service = self.stack[node].get('service')
            if service is not None and node != 'gazebo':
                services_to_stop.append(service)

        if services_to_stop:
            # Stop and remove containers so Phase 4 creates fresh ones.
            # Without removal, stateful services (e.g. controller_manager) retain
            # their internal state across restarts, causing controller_spawner to
            # fail with "controller already loaded".
            print(f"  Stopping {len(services_to_stop)} services in parallel...")
            cmd = ['docker', 'compose', 'rm', '-f', '-s'] + services_to_stop
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=60.0)
            if result.returncode == 0:
                print("[OK] All nodes stopped")
            else:
                print(f"[WARN] Some services may have failed to stop: {result.stderr[:200]}")
        else:
            print("[OK] No services to stop")

        phase1_duration = time.time() - phase1_start
        print(f"[Phase 1 complete: {phase1_duration:.1f}s]")

        # Phase 2: Reset Gazebo world
        phase2_start = time.time()
        print("\n[Phase 2/4] Resetting Gazebo world...")
        if not self.reset_gazebo_world():
            print("[X] Gazebo reset failed")
            return False

        # Brief settling time
        time.sleep(1.0)

        phase2_duration = time.time() - phase2_start
        print(f"[Phase 2 complete: {phase2_duration:.1f}s]")

        # Phase 3: Restart bridge services if configured
        phase3_start = time.time()
        if self.bridge_services:
            print("\n[Phase 3/4] Restarting bridge services...")
            if not self.restart_bridge_services():
                print("[X] Bridge restart failed")
                return False
        else:
            print("\n[Phase 3/4] No bridge services to restart")

        # Start test-drive early so gate checks can run
        print("\n  Starting test-drive for gate checking...")
        if not self.docker_compose_start('test-drive'):
            print("  [X] Failed to start test-drive")
            return False
        print("  [OK] test-drive started")
        time.sleep(2.0)  # Brief settling time

        phase3_duration = time.time() - phase3_start
        print(f"[Phase 3 complete: {phase3_duration:.1f}s]")

        # Phase 4: Start all nodes with gates (parallelized by dependency level)
        phase4_start = time.time()
        print("\n[Phase 4/4] Starting nodes in dependency order (parallel per level)...")

        if not self.start_nodes_parallel():
            print(f"\n[X] Stack startup failed")
            return False

        phase4_duration = time.time() - phase4_start
        print(f"[Phase 4 complete: {phase4_duration:.1f}s]")

        total_duration = time.time() - reset_start_time

        print("\n" + "=" * 60)
        print(f"[OK] FULL RESET COMPLETE ({total_duration:.1f}s total)")
        print("=" * 60)
        print(f"\nTiming breakdown:")
        print(f"  Phase 1 (Stop nodes):       {phase1_duration:6.1f}s")
        print(f"  Phase 2 (Reset Gazebo):     {phase2_duration:6.1f}s")
        print(f"  Phase 3 (Restart bridges):  {phase3_duration:6.1f}s")
        print(f"  Phase 4 (Start nodes):      {phase4_duration:6.1f}s")
        print(f"  {'-' * 40}")
        print(f"  Total:                      {total_duration:6.1f}s")
        print("=" * 60)
        return True
