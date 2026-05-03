"""Ackermann steering command conditioning filter.

Shared between the Gazebo hardware bridge and the real robot hardware node.
Addresses the atan(wz/vx) singularity and low-speed steering noise.

Two mechanisms in series:
  1. Alpha-blend guard: holds steering near last angle when vx ~ 0
     (singularity fence for atan(wz/vx) at exact zero)
  2. Speed-scaled rate limiter: clamps steering slew rate in the
     low-but-nonzero vx regime where atan sensitivity is high
     but the guard is transparent

At the vx_steer_threshold boundary both are active; the combined
effect is benign (both push toward smoothness) but the effective
limiting is not the simple sum of either alone.

"""

import math
import time as _time


class SteeringFilter:
    """Stateful steering command conditioning filter.

    Pipeline per tick:
      1. Sanitize non-finite input
      2. Alpha-blend guard (hold at vx ~ 0)
      3. Clamp to physical angle bounds
      4. Speed-scaled rate limiter toward bounded target
      5. Output = new last_cmd (tracks final post-everything value)

    The output state (last_cmd) tracks the final conditioned value.
    This is load-bearing: if it tracked a pre-saturation value, the
    rate limiter's delta reference would drift from the actual servo
    position during sustained saturation. Do not change this.

    Args:
        vx_threshold: speed below which alpha-blend activates (m/s)
        max_angle: physical steering angle limit (rad)
        rate_base: rate limit at vx=0 (rad/s)
        rate_gain: additional rate per unit speed ((rad/s)/(m/s))
        rate_max: absolute cap on steering rate (rad/s)
        dt_min: minimum dt clamp to reject spurious zero-dt ticks (s)
        dt_max: maximum dt clamp to prevent timing hiccups from
                defeating the limiter for one tick (s)
    """

    def __init__(
        self,
        vx_threshold: float = 0.05,
        max_angle: float = 0.4887,
        rate_base: float = 0.5,
        rate_gain: float = 10.0,
        rate_max: float = 6.0,
        dt_min: float = 0.005,
        dt_max: float = 0.15,
    ):
        self.vx_threshold = vx_threshold
        self.max_angle = max_angle
        self.rate_base = rate_base
        self.rate_gain = rate_gain
        self.rate_max = rate_max
        self.dt_min = dt_min
        self.dt_max = dt_max

        self._last_cmd: float = 0.0
        self._last_time: float | None = None

    @property
    def last_cmd(self) -> float:
        """Last conditioned output (read-only)."""
        return self._last_cmd

    def reset(self):
        """Reset filter state (e.g. after teleport / warm reset)."""
        self._last_cmd = 0.0
        self._last_time = None

    def update(self, raw_cmd: float, vx_approx: float,
               now: float | None = None) -> float:
        """Run one tick of the conditioning pipeline.

        Args:
            raw_cmd: raw steering angle command from the controller (rad)
            vx_approx: estimated forward speed magnitude (m/s, >= 0)
            now: monotonic timestamp (s). If None, uses time.monotonic().

        Returns:
            Conditioned steering angle command (rad).
        """
        if now is None:
            now = _time.monotonic()

        # Step 1: sanitize
        cmd = 0.0 if not math.isfinite(raw_cmd) else raw_cmd

        # Step 2: alpha-blend guard (singularity fence at vx ~ 0)
        if self.vx_threshold > 0:
            alpha = min(1.0, vx_approx / self.vx_threshold)
        else:
            alpha = 1.0
        cmd = alpha * cmd + (1.0 - alpha) * self._last_cmd

        # Step 3: clamp to physical angle bounds BEFORE rate limiting.
        # The rate limiter must chase the real achievable target, not an
        # intermediate value — otherwise it burns rate budget approaching
        # an unreachable position near full lock.
        target = max(-self.max_angle, min(self.max_angle, cmd))

        # Step 4: speed-scaled rate limiter
        # max_rate = min(cap, base + gain * |vx|)
        # Uses |vx| (vx_approx is already non-negative) so reverse is
        # treated identically to forward.
        max_rate = min(self.rate_max,
                       self.rate_base + self.rate_gain * vx_approx)

        if self._last_time is not None:
            dt = now - self._last_time
            # Sanity clamp: prevent timing hiccups (pause, scheduler
            # stall) from defeating the limiter for one tick.
            dt = max(self.dt_min, min(dt, self.dt_max))
            max_delta = max_rate * dt
            delta = target - self._last_cmd
            delta = max(-max_delta, min(max_delta, delta))
            output = self._last_cmd + delta
        else:
            output = target

        self._last_time = now
        self._last_cmd = output
        return output
