"""Subprocess helpers with mandatory timeouts and container orphan prevention.

Policy: every subprocess.run call MUST have a timeout.  Import `run` from this
module instead of using subprocess.run directly.  For commands inside Docker
containers, use `docker_exec` which additionally wraps the inner command with
the `timeout` utility to kill container-side processes on hang (preventing
orphans that survive after the host-side subprocess is killed).
"""

import os
import shlex
import subprocess
import sys

# Default timeout: 5 minutes.  Generous enough for any single operation,
# short enough to catch hangs before they waste hours.
DEFAULT_TIMEOUT = 300

# Container to use when none specified
DEFAULT_CONTAINER = 'test-drive'


def _default_env():
    """Build env dict with MSYS_NO_PATHCONV set (Git Bash path mangling fix)."""
    env = os.environ.copy()
    env['MSYS_NO_PATHCONV'] = '1'
    return env


def run(cmd, timeout=DEFAULT_TIMEOUT, **kwargs):
    """subprocess.run with a mandatory default timeout.

    Drop-in replacement for subprocess.run.  Defaults: capture_output=True,
    text=True, timeout=300s, MSYS_NO_PATHCONV=1.

    Args:
        cmd: Command list or string.
        timeout: Seconds before TimeoutExpired.  Pass None to disable
                 (not recommended — document why in the calling code).
        **kwargs: Forwarded to subprocess.run.

    Returns:
        subprocess.CompletedProcess
    """
    kwargs.setdefault('capture_output', True)
    kwargs.setdefault('text', True)
    if 'env' not in kwargs:
        kwargs['env'] = _default_env()
    return subprocess.run(cmd, timeout=timeout, **kwargs)


def docker_exec(cmd, container=DEFAULT_CONTAINER, timeout=60, **kwargs):
    """Run a shell command inside a Docker container with orphan-safe timeout.

    Wraps the inner command with the Linux `timeout` utility so the
    container-side process is killed even if the host-side subprocess.run
    timeout fires first (which only kills `docker compose exec`, leaving
    the inner process as an orphan).

    Args:
        cmd: Shell command string to run inside the container.
              Example: "source /opt/ros/jazzy/setup.bash && ros2 topic list"
        container: Docker Compose service name (default: 'test-drive').
        timeout: Timeout in seconds.  Applied both inside the container
                 (inner_timeout = timeout - 5) and on the host (outer).
        **kwargs: Extra args forwarded to run().

    Returns:
        subprocess.CompletedProcess
    """
    # Inner timeout slightly less than outer so the container process dies
    # cleanly before the host subprocess.run fires SIGTERM.
    inner_timeout = max(1, timeout - 5)

    # shlex.quote handles embedded quotes in cmd (e.g. YAML param strings)
    wrapped = f'timeout {inner_timeout} bash -c {shlex.quote(cmd)}'

    full_cmd = [
        'docker', 'compose', 'exec', '-T', container,
        'bash', '-c', wrapped,
    ]
    return run(full_cmd, timeout=timeout, **kwargs)
