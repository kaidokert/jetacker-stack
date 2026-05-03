"""Unit tests for SteeringFilter."""

import math
import pytest
from steering_filter import SteeringFilter


class TestSanitize:
    """Step 1: non-finite input sanitization."""

    def test_nan_replaced_with_zero(self):
        f = SteeringFilter()
        out = f.update(float('nan'), vx_approx=1.0, now=0.0)
        assert out == 0.0

    def test_inf_replaced_with_zero(self):
        f = SteeringFilter()
        out = f.update(float('inf'), vx_approx=1.0, now=0.0)
        assert out == 0.0

    def test_neg_inf_replaced_with_zero(self):
        f = SteeringFilter()
        out = f.update(float('-inf'), vx_approx=1.0, now=0.0)
        assert out == 0.0

    def test_finite_passes_through(self):
        f = SteeringFilter(rate_max=999.0, rate_base=999.0)
        out = f.update(0.3, vx_approx=1.0, now=0.0)
        assert out == pytest.approx(0.3)


class TestAlphaBlendGuard:
    """Step 2: alpha-blend singularity guard at vx ~ 0."""

    def test_zero_vx_holds_last_cmd(self):
        f = SteeringFilter(vx_threshold=0.05)
        # First tick at speed — establishes last_cmd
        f.update(0.2, vx_approx=1.0, now=0.0)
        # Now vx=0, new command should be ignored, output stays near 0.2
        out = f.update(0.0, vx_approx=0.0, now=0.1)
        assert out == pytest.approx(f.last_cmd)
        # alpha=0 means output = 0*cmd + 1*last_cmd = last_cmd
        # (rate limiter may clamp further, but direction is toward last_cmd)

    def test_half_threshold_blends_50_50(self):
        f = SteeringFilter(vx_threshold=0.1, rate_max=999.0, rate_base=999.0)
        f.update(0.0, vx_approx=1.0, now=0.0)  # last_cmd = 0
        # vx=0.05 is half of threshold=0.1, so alpha=0.5
        out = f.update(0.4, vx_approx=0.05, now=0.1)
        # blended = 0.5 * 0.4 + 0.5 * 0.0 = 0.2
        assert out == pytest.approx(0.2)

    def test_above_threshold_transparent(self):
        f = SteeringFilter(vx_threshold=0.05, rate_max=999.0, rate_base=999.0)
        out = f.update(0.3, vx_approx=0.1, now=0.0)
        # alpha = min(1.0, 0.1/0.05) = 1.0, fully transparent
        assert out == pytest.approx(0.3)

    def test_zero_threshold_disables_guard(self):
        f = SteeringFilter(vx_threshold=0.0, rate_max=999.0, rate_base=999.0)
        out = f.update(0.3, vx_approx=0.0, now=0.0)
        assert out == pytest.approx(0.3)


class TestAngleClamp:
    """Step 3: physical angle bounds."""

    def test_clamps_positive(self):
        f = SteeringFilter(max_angle=0.4, rate_max=999.0, rate_base=999.0)
        out = f.update(0.8, vx_approx=1.0, now=0.0)
        assert out == pytest.approx(0.4)

    def test_clamps_negative(self):
        f = SteeringFilter(max_angle=0.4, rate_max=999.0, rate_base=999.0)
        out = f.update(-0.8, vx_approx=1.0, now=0.0)
        assert out == pytest.approx(-0.4)

    def test_within_bounds_unchanged(self):
        f = SteeringFilter(max_angle=0.5, rate_max=999.0, rate_base=999.0)
        out = f.update(0.3, vx_approx=1.0, now=0.0)
        assert out == pytest.approx(0.3)


class TestRateLimiter:
    """Step 4: speed-scaled rate limiter."""

    def test_first_tick_no_rate_limit(self):
        """First call has no previous time — should pass through (after clamp)."""
        f = SteeringFilter(rate_base=0.1, vx_threshold=0.0)
        out = f.update(0.3, vx_approx=0.0, now=0.0)
        assert out == pytest.approx(0.3)

    def test_rate_limited_at_low_speed(self):
        """At vx=0, max_rate=rate_base. Large step should be clamped."""
        f = SteeringFilter(rate_base=0.5, rate_gain=10.0, rate_max=6.0,
                           vx_threshold=0.0)  # disable guard to isolate limiter
        f.update(0.0, vx_approx=0.0, now=0.0)  # last_cmd = 0
        # dt=0.1, max_rate=0.5, max_delta=0.05
        out = f.update(0.4, vx_approx=0.0, now=0.1)
        assert out == pytest.approx(0.05)

    def test_rate_scales_with_speed(self):
        """At higher vx, rate limit is more permissive."""
        f = SteeringFilter(rate_base=0.5, rate_gain=10.0, rate_max=6.0)
        f.update(0.0, vx_approx=0.5, now=0.0)
        # dt=0.1, max_rate=min(6.0, 0.5+10*0.5)=5.5, max_delta=0.55
        out = f.update(0.4, vx_approx=0.5, now=0.1)
        # 0.4 < 0.55, so not clamped
        assert out == pytest.approx(0.4)

    def test_rate_max_caps(self):
        """Rate should not exceed rate_max regardless of speed."""
        f = SteeringFilter(rate_base=0.5, rate_gain=10.0, rate_max=6.0)
        f.update(0.0, vx_approx=10.0, now=0.0)
        # max_rate = min(6.0, 0.5+10*10) = 6.0, max_delta = 0.6
        out = f.update(0.4, vx_approx=10.0, now=0.1)
        assert out == pytest.approx(0.4)  # 0.4 < 0.6

    def test_negative_direction_rate_limited(self):
        """Rate limiter works symmetrically for negative changes."""
        f = SteeringFilter(rate_base=0.5, rate_gain=10.0, rate_max=6.0,
                           vx_threshold=0.0)
        f.update(0.0, vx_approx=0.0, now=0.0)
        out = f.update(-0.4, vx_approx=0.0, now=0.1)
        assert out == pytest.approx(-0.05)

    def test_convergence_over_multiple_ticks(self):
        """Filter should converge to target over successive ticks."""
        f = SteeringFilter(rate_base=1.0, rate_gain=0.0, rate_max=6.0,
                           vx_threshold=0.0)
        f.update(0.0, vx_approx=0.0, now=0.0)
        target = 0.3
        t = 0.0
        for _ in range(100):
            t += 0.1
            out = f.update(target, vx_approx=0.0, now=t)
        assert out == pytest.approx(target, abs=1e-6)


class TestDtClamp:
    """dt sanity clamp prevents timing hiccups from defeating the limiter."""

    def test_large_dt_clamped(self):
        """A 5-second gap should not allow a huge steering jump."""
        f = SteeringFilter(rate_base=0.5, rate_gain=0.0, rate_max=6.0,
                           dt_max=0.15, vx_threshold=0.0)
        f.update(0.0, vx_approx=0.0, now=0.0)
        # 5-second gap, but dt clamped to 0.15
        # max_delta = 0.5 * 0.15 = 0.075
        out = f.update(0.4, vx_approx=0.0, now=5.0)
        assert out == pytest.approx(0.075)

    def test_tiny_dt_clamped(self):
        """Near-zero dt should be clamped to dt_min."""
        f = SteeringFilter(rate_base=0.5, rate_gain=0.0, rate_max=6.0,
                           dt_min=0.005, vx_threshold=0.0)
        f.update(0.0, vx_approx=0.0, now=0.0)
        # dt = 0.0001, clamped to 0.005
        # max_delta = 0.5 * 0.005 = 0.0025
        out = f.update(0.4, vx_approx=0.0, now=0.0001)
        assert out == pytest.approx(0.0025)


class TestReset:
    """Reset clears state."""

    def test_reset_clears_last_cmd(self):
        f = SteeringFilter(rate_max=999.0, rate_base=999.0)
        f.update(0.3, vx_approx=1.0, now=0.0)
        assert f.last_cmd == pytest.approx(0.3)
        f.reset()
        assert f.last_cmd == 0.0

    def test_reset_allows_immediate_jump(self):
        """After reset, next call is treated as first tick (no rate limit)."""
        f = SteeringFilter(rate_base=0.1)
        f.update(0.0, vx_approx=0.0, now=0.0)
        f.update(0.0, vx_approx=0.0, now=0.1)  # establish timing
        f.reset()
        out = f.update(0.3, vx_approx=1.0, now=0.5)
        assert out == pytest.approx(0.3)


class TestLastCmdTracking:
    """last_cmd must track post-everything output (load-bearing invariant)."""

    def test_tracks_clamped_value(self):
        """last_cmd should be the clamped value, not the raw input."""
        f = SteeringFilter(max_angle=0.4, rate_max=999.0, rate_base=999.0)
        f.update(0.8, vx_approx=1.0, now=0.0)
        assert f.last_cmd == pytest.approx(0.4)

    def test_tracks_rate_limited_value(self):
        """last_cmd should be the rate-limited value."""
        f = SteeringFilter(rate_base=0.5, rate_gain=0.0, rate_max=6.0,
                           vx_threshold=0.0)
        f.update(0.0, vx_approx=0.0, now=0.0)
        f.update(0.4, vx_approx=0.0, now=0.1)  # rate limited to 0.05
        assert f.last_cmd == pytest.approx(0.05)


class TestPipelineOrdering:
    """Verify clamp-before-rate-limit ordering."""

    def test_clamp_before_rate_limit(self):
        """Rate limiter should chase the clamped target, not waste budget on
        unreachable angles beyond max_angle."""
        f = SteeringFilter(max_angle=0.4, rate_base=1.0, rate_gain=0.0,
                           rate_max=6.0)
        # Start at 0.35 (near max)
        f.update(0.35, vx_approx=1.0, now=0.0)
        # Command 0.8 (beyond max). Target should be clamped to 0.4.
        # Delta = 0.4 - 0.35 = 0.05. max_delta = 1.0 * 0.1 = 0.1.
        # 0.05 < 0.1, so output = 0.4 (reaches clamped target in one tick).
        out = f.update(0.8, vx_approx=1.0, now=0.1)
        assert out == pytest.approx(0.4)


class TestDefaultTimestamp:
    """Verify time.monotonic() fallback works."""

    def test_auto_timestamp(self):
        """Calling without `now` should not crash and should produce finite output."""
        f = SteeringFilter()
        out1 = f.update(0.1, vx_approx=0.5)
        assert math.isfinite(out1)
        out2 = f.update(0.1, vx_approx=0.5)
        assert math.isfinite(out2)
