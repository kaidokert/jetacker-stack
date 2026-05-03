"""Episode-level metrics collection for test_drive.

Lightweight in-memory metrics computed per episode, replacing rosbag +
post-processing.  No numpy dependency -- plain math over small lists at
10-50 Hz sampling rates.

Metrics computed at episode end (finalize):
    A1: RMS steering rate (rad/s)      -- from /robot_joint_states
    A2: Steering flip rate (Hz)        -- zero-crossings above deadband
    B1: TV(linear)                     -- total variation of cmd_vel.linear.x
    B2: TV(angular)                    -- total variation of cmd_vel.angular.z
    B3: Reversal count                 -- sign changes in cmd_vel.linear.x (deadband filtered)
    C1: RMS cross-track error (m)      -- perpendicular distance from drive line
    C2: Max cross-track error (m)      -- peak of above
    D1: RMS heading deviation (rad)    -- deviation from heading at drive start
    E1: TV(wheel velocity)             -- rear wheel velocity jitter from /robot_joint_states
"""

import math
from typing import List, Tuple, Optional, Dict, Any


class EpisodeMetrics:
    """Collect and compute per-episode telemetry metrics."""

    def __init__(self):
        self.reset()

    def reset(self):
        """Clear all collected data between episodes."""
        self._episode_start: float = 0.0

        # Steering data: (timestamp, angle) pairs
        self._steering_samples: List[Tuple[float, float]] = []

        # Commanded steering data: (timestamp, angle) from /robot_joint_commands
        self._steering_cmd_samples: List[Tuple[float, float]] = []

        # Cmd_vel data: (timestamp, vx, wz) triples
        self._cmd_vel_samples: List[Tuple[float, float, float]] = []

        # Rear wheel velocity data: (timestamp, left_vel, right_vel) triples
        self._wheel_vel_samples: List[Tuple[float, float, float]] = []

        # Drive phase data: completed phases
        self._drive_phases: List[Dict[str, Any]] = []
        self._current_drive_phase: Optional[Dict[str, Any]] = None

    def begin(self, stamp: float):
        """Mark episode start."""
        self._episode_start = stamp

    def record_joint_state(self, stamp: float, steering_angle: float,
                          left_wheel_vel: float = 0.0,
                          right_wheel_vel: float = 0.0):
        """Record steering angle and rear wheel velocities from /robot_joint_states."""
        self._steering_samples.append((stamp, steering_angle))
        self._wheel_vel_samples.append((stamp, left_wheel_vel, right_wheel_vel))

    def record_steering_command(self, stamp: float, steering_angle: float):
        """Record commanded steering angle from /robot_joint_commands."""
        self._steering_cmd_samples.append((stamp, steering_angle))

    def record_cmd_vel(self, stamp: float, vx: float, wz: float):
        """Record a cmd_vel sample."""
        self._cmd_vel_samples.append((stamp, vx, wz))

    def begin_drive_phase(self, x: float, y: float, yaw: float):
        """Start collecting cross-track / heading data for a DRIVE instruction."""
        self._current_drive_phase = {
            'start_x': x,
            'start_y': y,
            'start_yaw': yaw,
            'samples': [(x, y, yaw)],
        }

    def record_drive_sample(self, x: float, y: float, yaw: float):
        """Record a pose sample during DRIVE phase."""
        if self._current_drive_phase is not None:
            self._current_drive_phase['samples'].append((x, y, yaw))

    def end_drive_phase(self):
        """Finalize current drive phase."""
        if self._current_drive_phase is not None:
            self._drive_phases.append(self._current_drive_phase)
            self._current_drive_phase = None

    def finalize(self, stamp: float) -> Dict[str, Any]:
        """Compute all metrics and return as dict."""
        duration = stamp - self._episode_start if self._episode_start > 0 else 0.0

        metrics: Dict[str, Any] = {
            'duration': round(duration, 2),
            'sample_counts': {
                'steering': len(self._steering_samples),
                'cmd_vel': len(self._cmd_vel_samples),
                'wheel_vel': len(self._wheel_vel_samples),
                'drive_phases': len(self._drive_phases),
            },
        }

        # A: Steering metrics (actual + commanded)
        steer = self._compute_steering_metrics()
        if steer:
            metrics['steering'] = steer
        metrics['sample_counts']['steering_cmd'] = len(self._steering_cmd_samples)

        # B: Cmd vel metrics
        cmdvel = self._compute_cmd_vel_metrics()
        if cmdvel:
            metrics['cmd_vel'] = cmdvel

        # C, D: Cross-track and heading metrics
        track = self._compute_tracking_metrics()
        if track:
            metrics['tracking'] = track

        # E: Wheel velocity jitter
        wheel = self._compute_wheel_vel_metrics()
        if wheel:
            metrics['wheel_vel'] = wheel

        return metrics

    # ------------------------------------------------------------------
    # Internal computation methods
    # ------------------------------------------------------------------

    @staticmethod
    def _compute_steering_from_samples(samples: List[Tuple[float, float]]) -> Optional[Dict[str, float]]:
        """Compute RMS steering rate and flip rate from (timestamp, angle) samples."""
        if len(samples) < 2:
            return None

        rates: List[float] = []
        for i in range(1, len(samples)):
            t0, a0 = samples[i - 1]
            t1, a1 = samples[i]
            dt = t1 - t0
            if dt > 0:
                rates.append((a1 - a0) / dt)

        if not rates:
            return None

        # RMS steering rate
        rms_rate = math.sqrt(sum(r * r for r in rates) / len(rates))

        # Steering flip rate -- zero-crossings above deadband
        DEADBAND = 0.02  # rad/s
        flips = 0
        prev_sign = 0  # 0 = in deadband
        for r in rates:
            if r > DEADBAND:
                sign = 1
            elif r < -DEADBAND:
                sign = -1
            else:
                continue  # in deadband, don't update
            if prev_sign != 0 and sign != prev_sign:
                flips += 1
            prev_sign = sign

        total_time = samples[-1][0] - samples[0][0]
        flip_rate = flips / total_time if total_time > 0 else 0.0

        return {
            'rms_rate': round(rms_rate, 4),
            'flip_rate_hz': round(flip_rate, 2),
        }

    def _compute_steering_metrics(self) -> Optional[Dict[str, float]]:
        """A1: RMS steering rate, A2: steering flip rate (actual + commanded)."""
        actual = self._compute_steering_from_samples(self._steering_samples)
        if actual is None:
            return None

        result = {
            'rms_rate': actual['rms_rate'],
            'flip_rate_hz': actual['flip_rate_hz'],
        }

        # Add commanded steering metrics if available
        cmd = self._compute_steering_from_samples(self._steering_cmd_samples)
        if cmd is not None:
            result['cmd_rms_rate'] = cmd['rms_rate']
            result['cmd_flip_rate_hz'] = cmd['flip_rate_hz']

        return result

    def _compute_cmd_vel_metrics(self) -> Optional[Dict[str, float]]:
        """B1: TV(linear), B2: TV(angular)."""
        if len(self._cmd_vel_samples) < 2:
            return None

        tv_linear = 0.0
        tv_angular = 0.0
        for i in range(1, len(self._cmd_vel_samples)):
            _, vx0, wz0 = self._cmd_vel_samples[i - 1]
            _, vx1, wz1 = self._cmd_vel_samples[i]
            tv_linear += (vx1 - vx0) ** 2
            tv_angular += (wz1 - wz0) ** 2

        total_time = self._cmd_vel_samples[-1][0] - self._cmd_vel_samples[0][0]
        if total_time > 0:
            tv_linear /= total_time
            tv_angular /= total_time

        # B3: Reversal count — sign changes in vx with deadband
        VX_DEADBAND = 0.01  # m/s — ignore zero-crossing noise
        reversal_count = 0
        prev_sign = 0  # 0 = in deadband
        for _, vx, _ in self._cmd_vel_samples:
            if vx > VX_DEADBAND:
                sign = 1
            elif vx < -VX_DEADBAND:
                sign = -1
            else:
                continue
            if prev_sign != 0 and sign != prev_sign:
                reversal_count += 1
            prev_sign = sign

        return {
            'tv_linear': round(tv_linear, 6),
            'tv_angular': round(tv_angular, 6),
            'reversal_count': reversal_count,
        }

    def _compute_tracking_metrics(self) -> Optional[Dict[str, float]]:
        """C1/C2: cross-track error, D1: heading deviation.

        Aggregated across all drive phases.
        """
        all_xtrack: List[float] = []
        all_heading_dev: List[float] = []

        for phase in self._drive_phases:
            x0 = phase['start_x']
            y0 = phase['start_y']
            yaw0 = phase['start_yaw']
            cos_yaw = math.cos(yaw0)
            sin_yaw = math.sin(yaw0)

            for x, y, yaw in phase['samples'][1:]:  # skip first (start) sample
                # Perpendicular distance from start+heading line
                dx = x - x0
                dy = y - y0
                xtrack = abs(dy * cos_yaw - dx * sin_yaw)
                all_xtrack.append(xtrack)

                # Heading deviation (normalized to [-pi, pi])
                hdiff = yaw - yaw0
                while hdiff > math.pi:
                    hdiff -= 2 * math.pi
                while hdiff < -math.pi:
                    hdiff += 2 * math.pi
                all_heading_dev.append(hdiff)

        if not all_xtrack:
            return None

        rms_xtrack = math.sqrt(sum(x * x for x in all_xtrack) / len(all_xtrack))
        max_xtrack = max(all_xtrack)
        rms_heading = math.sqrt(sum(h * h for h in all_heading_dev) / len(all_heading_dev))

        return {
            'rms_xtrack_m': round(rms_xtrack, 4),
            'max_xtrack_m': round(max_xtrack, 4),
            'rms_heading_dev_rad': round(rms_heading, 4),
        }

    def _compute_wheel_vel_metrics(self) -> Optional[Dict[str, float]]:
        """E1: TV of rear wheel velocities (motor-level jitter)."""
        if len(self._wheel_vel_samples) < 2:
            return None

        tv_left = 0.0
        tv_right = 0.0
        for i in range(1, len(self._wheel_vel_samples)):
            _, vl0, vr0 = self._wheel_vel_samples[i - 1]
            _, vl1, vr1 = self._wheel_vel_samples[i]
            tv_left += (vl1 - vl0) ** 2
            tv_right += (vr1 - vr0) ** 2

        total_time = self._wheel_vel_samples[-1][0] - self._wheel_vel_samples[0][0]
        if total_time > 0:
            tv_left /= total_time
            tv_right /= total_time

        return {
            'tv_left': round(tv_left, 6),
            'tv_right': round(tv_right, 6),
            'tv_combined': round((tv_left + tv_right) / 2, 6),
        }
