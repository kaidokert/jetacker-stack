"""Tests for select_medoid() in tune_common.py."""

import math
import pytest
import sys
from pathlib import Path

# Ensure repo root is importable
sys.path.insert(0, str(Path(__file__).parent.parent))

from tune_common import select_medoid


# ---------------------------------------------------------------------------
# Basic behavior
# ---------------------------------------------------------------------------

def test_example_table():
    """The example from the design discussion — rep 3 (index 2) is the medoid."""
    reps = [
        [10.2, 0.15, 0.10, 3.1, 2],   # rep 1
        [11.5, 0.24, 0.19, 2.2, 0],   # rep 2
        [10.8, 0.18, 0.14, 2.8, 1],   # rep 3 — expected medoid
        [10.4, 0.16, 0.12, 3.0, 1],   # rep 4
        [14.1, 0.31, 0.25, 5.1, 4],   # rep 5 — outlier
    ]
    idx, consistency = select_medoid(reps)
    assert idx == 2
    assert consistency > 0


def test_three_reps():
    """Minimum valid case — 3 reps, middle one wins."""
    reps = [
        [1.0, 10.0],
        [2.0, 20.0],   # middle
        [3.0, 30.0],
    ]
    idx, consistency = select_medoid(reps)
    assert idx == 1
    assert consistency > 0


def test_outlier_rejected():
    """A single extreme outlier should never be selected."""
    reps = [
        [10.0, 0.2, 1.0],
        [10.5, 0.3, 1.5],
        [10.2, 0.25, 1.2],  # medoid — central
        [10.3, 0.22, 1.1],
        [50.0, 5.0, 99.0],  # extreme outlier
    ]
    idx, consistency = select_medoid(reps)
    assert idx != 4, "Outlier should not be selected as medoid"
    assert consistency > 0.2, "Outlier should cause high inconsistency"


def test_identical_reps():
    """All reps identical — any index is valid, consistency = 0."""
    reps = [
        [5.0, 0.1, 0.05],
        [5.0, 0.1, 0.05],
        [5.0, 0.1, 0.05],
    ]
    idx, consistency = select_medoid(reps)
    assert idx in [0, 1, 2]
    assert consistency < 1e-10


def test_single_objective():
    """Works with 1D objectives — should pick the median value."""
    reps = [
        [100.0],
        [1.0],
        [50.0],   # median value
        [51.0],
        [49.0],
    ]
    idx, consistency = select_medoid(reps)
    # The three central values (49, 50, 51) are all close; medoid should be one of them
    assert idx in [2, 3, 4]


def test_constant_objective_ignored():
    """If one objective is constant across all reps, it doesn't affect the result."""
    reps = [
        [1.0, 999.0],
        [2.0, 999.0],
        [3.0, 999.0],   # middle on first dim
        [4.0, 999.0],
        [5.0, 999.0],
    ]
    idx, consistency = select_medoid(reps)
    assert idx == 2


def test_seven_reps():
    """7 reps — odd, should work."""
    reps = [
        [10.0, 0.2],
        [10.5, 0.25],
        [10.2, 0.22],
        [10.3, 0.21],  # central cluster
        [10.4, 0.23],
        [10.1, 0.19],
        [15.0, 0.80],  # outlier
    ]
    idx, consistency = select_medoid(reps)
    assert idx != 6, "Outlier should not be selected"


def test_consistency_returns_float():
    """Consistency is always a non-negative float."""
    reps = [
        [10.0, 0.20],
        [10.1, 0.21],
        [10.05, 0.205],
    ]
    idx, consistency = select_medoid(reps)
    assert isinstance(consistency, float)
    assert consistency >= 0.0


def test_high_spread_high_consistency():
    """Widely spread reps should have high consistency score (CV-based)."""
    reps = [
        [1.0, 0.1],
        [50.0, 0.5],
        [100.0, 1.0],
    ]
    idx, consistency = select_medoid(reps)
    assert consistency > 0.5


# ---------------------------------------------------------------------------
# Validation / assertions
# ---------------------------------------------------------------------------

def test_rejects_single_rep():
    with pytest.raises(AssertionError, match="at least 3"):
        select_medoid([[1.0, 2.0]])


def test_rejects_two_reps():
    with pytest.raises(AssertionError, match="at least 3"):
        select_medoid([[1.0], [2.0]])


def test_rejects_four_reps():
    with pytest.raises(AssertionError, match="odd number"):
        select_medoid([[1.0], [2.0], [3.0], [4.0]])


def test_rejects_mismatched_lengths():
    with pytest.raises(AssertionError, match="same number"):
        select_medoid([[1.0, 2.0], [3.0], [4.0, 5.0]])


# ---------------------------------------------------------------------------
# Real-world replication scenarios (from pareto_3critic_M4_v4 study)
# ---------------------------------------------------------------------------
# These test cases are based on actual replication results observed during
# the v4 study. They validate that consistency scoring correctly separates
# reliable parameter configs from lucky single-run flukes.

def test_replication_inconsistent_low_wz_std():
    """Trial 148 pattern: low wz_std=0.025, 0/5 replication passes.

    Original scored 6.0s/0.21m/0.06rad — looked excellent on paper.
    Replication: all timeouts with wildly scattered times/errors.
    Consistency should be very high (bad).
    """
    # [time, error_xy, error_yaw, jitter, reversals]
    reps = [
        [41.0, 1.50, 3.14, 8.5, 12],   # timeout, nowhere near goal
        [30.0, 0.80, 2.10, 5.2, 8],     # timeout, drifted
        [13.0, 0.45, 1.20, 3.1, 4],     # failed but got closer
        [30.0, 1.10, 2.80, 6.8, 10],    # timeout
        [10.0, 0.30, 0.90, 2.5, 3],     # closest to original but still failed
    ]
    idx, consistency = select_medoid(reps)
    # Huge spread — consistency should be very high
    assert consistency > 0.3, (
        f"Inconsistent trial should have high consistency score, got {consistency:.3f}")


def test_replication_consistent_high_wz_std():
    """Trial 471 pattern: high wz_std=0.50, 5/5 replication passes.

    Original scored 6.3s/0.21m/0.02rad. Replication: all pass with
    similar times/errors. High jitter/reversals but stable behavior.
    Consistency should be low (good).
    """
    # [time, error_xy, error_yaw, jitter, reversals]
    reps = [
        [7.1, 0.22, 0.04, 25.0, 3],
        [6.8, 0.19, 0.03, 27.0, 4],
        [7.3, 0.24, 0.05, 24.0, 3],
        [6.5, 0.20, 0.02, 26.0, 3],
        [7.0, 0.21, 0.04, 28.0, 4],
    ]
    idx, consistency = select_medoid(reps)
    # Tight cluster — mean CV should be low (< 0.15)
    assert consistency < 0.15, (
        f"Consistent trial should have low consistency score, got {consistency:.3f}")


def test_replication_moderate_consistency():
    """Trial 292 pattern: medium wz_std=0.20, 4/5 replication passes.

    Original scored 5.8s. Replication: mostly passes, one outlier failure.
    Consistency should be moderate — between the extremes.
    """
    # [time, error_xy, error_yaw, jitter, reversals]
    reps = [
        [8.2, 0.25, 0.10, 18.0, 3],
        [9.5, 0.30, 0.15, 20.0, 5],
        [8.8, 0.28, 0.12, 19.0, 4],
        [30.0, 1.20, 2.50, 6.0, 8],    # failed rep — timeout
        [9.0, 0.26, 0.11, 17.0, 3],
    ]
    idx, consistency = select_medoid(reps)
    # One outlier drives high CV — consistency should be elevated
    assert consistency > 0.3, (
        f"4/5 pass with 1 timeout should have elevated consistency, got {consistency:.3f}")
    # Medoid should NOT be the failed rep
    assert idx != 3, "Failed rep should not be selected as medoid"


def test_consistency_ordering():
    """Consistent trials should score lower than inconsistent ones.

    This directly validates the key insight from the replication crisis:
    trial 471 (wz_std=0.50) should have better (lower) consistency than
    trial 148 (wz_std=0.025) despite worse jitter/reversals.
    """
    # Trial 471-like: tight cluster, all pass
    consistent_reps = [
        [7.1, 0.22, 0.04, 25.0, 3],
        [6.8, 0.19, 0.03, 27.0, 4],
        [7.3, 0.24, 0.05, 24.0, 3],
        [6.5, 0.20, 0.02, 26.0, 3],
        [7.0, 0.21, 0.04, 28.0, 4],
    ]

    # Trial 148-like: scattered, all fail
    inconsistent_reps = [
        [41.0, 1.50, 3.14, 8.5, 12],
        [30.0, 0.80, 2.10, 5.2, 8],
        [13.0, 0.45, 1.20, 3.1, 4],
        [30.0, 1.10, 2.80, 6.8, 10],
        [10.0, 0.30, 0.90, 2.5, 3],
    ]

    _, c_good = select_medoid(consistent_reps)
    _, c_bad = select_medoid(inconsistent_reps)

    assert c_good < c_bad, (
        f"Consistent trial ({c_good:.3f}) should score lower than "
        f"inconsistent ({c_bad:.3f})")
    # Quantitative check: good should be at least 2x better
    assert c_good < c_bad / 2, (
        f"Gap should be large: {c_good:.3f} vs {c_bad:.3f}")


def test_consistency_medoid_picks_representative_not_best():
    """Medoid should pick the most typical rep, not the best-performing one.

    In trial 292's replication, the fastest rep (8.2s) isn't necessarily
    the medoid — the rep closest to the cluster center is.
    """
    # [time, error_xy, error_yaw, jitter, reversals]
    reps = [
        [6.0, 0.15, 0.02, 22.0, 2],   # suspiciously good (lucky)
        [9.0, 0.28, 0.10, 19.0, 4],
        [8.5, 0.25, 0.09, 20.0, 3],   # cluster center
        [9.2, 0.30, 0.12, 18.0, 4],
        [8.8, 0.27, 0.11, 19.0, 3],
    ]
    idx, consistency = select_medoid(reps)
    # The "best" rep (index 0) is an outlier — medoid should pick from cluster
    assert idx != 0, (
        "Lucky outlier should not be selected as medoid")


# ---------------------------------------------------------------------------
# NaN / inf handling
# ---------------------------------------------------------------------------

def test_nan_values_treated_as_zero():
    """NaN values (e.g. jitter=nan from ROS) should not crash or produce nan."""
    reps = [
        [10.0, 0.20, float('nan')],
        [10.5, 0.25, 1.5],
        [10.2, 0.22, 1.2],
    ]
    idx, consistency = select_medoid(reps)
    assert isinstance(consistency, float)
    assert not math.isnan(consistency), "Consistency must not be nan"
    assert not math.isinf(consistency), "Consistency must not be inf"
    assert idx in [0, 1, 2]


def test_inf_values_treated_as_zero():
    """Inf values should be sanitized like NaN."""
    reps = [
        [10.0, float('inf'), 1.0],
        [10.5, 0.25, 1.5],
        [10.2, 0.22, 1.2],
    ]
    idx, consistency = select_medoid(reps)
    assert not math.isnan(consistency)
    assert not math.isinf(consistency)


def test_all_nan_column():
    """If one objective is NaN for all reps, it's treated as all-zero (constant)."""
    reps = [
        [10.0, float('nan')],
        [20.0, float('nan')],
        [15.0, float('nan')],
    ]
    idx, consistency = select_medoid(reps)
    # Only first column matters, middle value (15.0) is medoid
    assert idx == 2
    assert not math.isnan(consistency)
