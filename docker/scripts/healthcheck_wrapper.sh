#!/bin/bash
# Generic health check wrapper with grace period
#
# Usage: healthcheck_wrapper.sh "<check_command>" <grace_seconds>
#
# The wrapper:
# 1. Runs the check command until it succeeds
# 2. Once it succeeds, waits for grace_seconds
# 3. Then reports healthy
#
# This prevents services from starting immediately after check passes,
# giving time for full initialization.

CHECK_CMD="$1"
GRACE_SECONDS="${2:-0}"
STATE_FILE="/tmp/healthcheck_$(basename "$0" .sh)_state"

# If check command is empty, fail
if [ -z "$CHECK_CMD" ]; then
    echo "ERROR: No check command provided"
    exit 1
fi

# Run the actual health check
if ! eval "$CHECK_CMD" > /dev/null 2>&1; then
    # Check failed - remove state file and report unhealthy
    rm -f "$STATE_FILE"
    exit 1
fi

# Check passed - now handle grace period
if [ "$GRACE_SECONDS" -eq 0 ]; then
    # No grace period needed, report healthy immediately
    exit 0
fi

# Check if this is the first time check passed
if [ ! -f "$STATE_FILE" ]; then
    # First time passing - record timestamp
    date +%s > "$STATE_FILE"
    echo "Health check passed, starting ${GRACE_SECONDS}s grace period..."
    exit 1  # Still report unhealthy during grace period
fi

# Check has been passing - see if grace period elapsed
FIRST_PASS=$(cat "$STATE_FILE")
NOW=$(date +%s)
ELAPSED=$((NOW - FIRST_PASS))

if [ "$ELAPSED" -ge "$GRACE_SECONDS" ]; then
    # Grace period elapsed - report healthy
    echo "Grace period complete, reporting healthy"
    exit 0
else
    # Still in grace period
    REMAINING=$((GRACE_SECONDS - ELAPSED))
    echo "Grace period: ${REMAINING}s remaining..."
    exit 1
fi
