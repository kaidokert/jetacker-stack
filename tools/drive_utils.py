#!/usr/bin/env python3
"""
Drive utilities - shared functions for drive_simple.py and drive_nav2.py.

Extracted from the legacy test_drive_simple.py to avoid dragging in
unused code (run_sim_reset, run_drive_test, run_cycles, etc.).
"""

import subprocess
import time
from typing import Optional, Tuple


# ANSI colors
RED = '\033[31m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
BLUE = '\033[34m'
CYAN = '\033[36m'
NC = '\033[0m'

# Timeout constants
NODE_RESTART_WAIT_SECONDS = 15.0


def detect_stack() -> Optional[str]:
    """
    Detect which robot stack is currently running.

    Returns:
        'jetacker' (sim), 'slam_bot' (sim), 'jetacker_real' (physical robot), or None
    """
    try:
        # Check for jetacker-gazebo (simulation)
        result = subprocess.run(
            ['docker', 'ps', '--filter', 'name=jetacker-gazebo', '--format', '{{.Names}}'],
            capture_output=True,
            text=True,
            check=False
        )
        if 'jetacker-gazebo' in result.stdout:
            return 'jetacker'

        # Check for slam gazebo (simulation)
        result = subprocess.run(
            ['docker', 'ps', '--filter', 'name=gazebo', '--format', '{{.Names}}'],
            capture_output=True,
            text=True,
            check=False
        )
        if 'gazebo' in result.stdout:
            return 'slam_bot'

        # Check for jetacker-hardware (physical robot)
        result = subprocess.run(
            ['docker', 'ps', '--filter', 'name=jetacker-hardware', '--format', '{{.Names}}'],
            capture_output=True,
            text=True,
            check=False
        )
        if 'jetacker-hardware' in result.stdout:
            return 'jetacker_real'

        return None

    except Exception:
        return None


def verify_controller_loaded(stack: str, controller_name: str = 'tricycle_steering_controller',
                            timeout: float = 10.0) -> Tuple[bool, dict]:
    """
    Verify that controller is loaded and active.

    Args:
        stack: 'jetacker' (sim), 'slam_bot' (sim), or 'jetacker_real' (physical) - for container naming
        controller_name: Name of controller to check
        timeout: Timeout in seconds

    Returns:
        (success, info_dict)
    """
    # Determine which container has controller-manager
    if stack in ('jetacker', 'jetacker_real'):
        container = 'jetacker-controller-manager'
    else:
        container = 'controller-manager'  # slam_bot legacy naming

    cmd = [
        'docker', 'compose', 'exec', '-T', container,
        'bash', '-c',
        'source /opt/ros/jazzy/setup.bash && ros2 control list_controllers'
    ]

    try:
        result = subprocess.run(cmd, capture_output=True, text=True,
                              check=False, timeout=timeout)

        # Parse output looking for controller_name with state "active"
        for line in result.stdout.split('\n'):
            if controller_name in line:
                if 'active' in line:
                    return True, {'state': 'active', 'controller': controller_name}
                else:
                    # Controller loaded but not active
                    return False, {'state': 'loaded_but_not_active',
                                 'controller': controller_name,
                                 'line': line.strip()}

        # Controller not found in list
        return False, {'state': 'not_found',
                      'controller': controller_name,
                      'output': result.stdout}

    except subprocess.TimeoutExpired:
        return False, {'state': 'timeout', 'controller': controller_name}
    except Exception as e:
        return False, {'state': 'error', 'controller': controller_name, 'error': str(e)}


def run_robot_reset(stack: str, quiet: bool = False, cycle_num: int = 0) -> Tuple[bool, dict]:
    """
    Reset physical robot state (no Gazebo, just ROS2 nodes).

    Args:
        stack: Must be 'jetacker_real'
        quiet: Suppress progress output
        cycle_num: Current cycle number for logging (0 = reset only)

    Returns:
        (success, result_dict)
    """
    if stack != 'jetacker_real':
        return False, {'success': False, 'message': f'Invalid stack for robot reset: {stack}'}

    # Nodes to restart for clean state.
    # hardware node is NOT restarted — it holds the STM32 serial connection.
    # controller-spawner is transient (already exited), so we re-up it separately.
    nodes_to_restart = ['jetacker-controller-manager', 'jetacker-ekf-localization']

    # ===== STEP 1: Restart ROS2 nodes =====
    if not quiet:
        print(f"{YELLOW}      [1/2] Restarting nodes: {', '.join(nodes_to_restart)}{NC}")

    # Stop nodes
    stop_cmd = ['docker', 'compose', 'stop'] + nodes_to_restart
    result = subprocess.run(stop_cmd, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        return False, {'success': False, 'message': f'Failed to stop nodes: {result.stderr}'}

    # Start nodes
    start_cmd = ['docker', 'compose', 'start'] + nodes_to_restart
    result = subprocess.run(start_cmd, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        return False, {'success': False, 'message': f'Failed to start nodes: {result.stderr}'}

    # Re-run controller spawner (transient — must use `up -d` since it already exited)
    spawn_cmd = ['docker', 'compose', 'up', '-d', 'jetacker-controller-spawner']
    subprocess.run(spawn_cmd, capture_output=True, text=True, check=False)

    # ===== STEP 2: Wait for controller ready =====
    if not quiet:
        print(f"{YELLOW}      [2/2] Waiting for controller ready...{NC}")

    # Give nodes time to initialize
    time.sleep(NODE_RESTART_WAIT_SECONDS)

    success, info = verify_controller_loaded(stack, 'tricycle_steering_controller')
    if not success:
        return False, {
            'success': False,
            'message': f'Controller verification failed: {info}'
        }

    if not quiet:
        print(f"{GREEN}      Controller active: {info['controller']}{NC}")

    return True, {'success': True, 'message': 'Reset complete', 'controller': info}
