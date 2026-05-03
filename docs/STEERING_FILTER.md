# Steering Filter

Shared Ackermann steering command conditioning filter. Addresses the `atan(wz/vx)` singularity and low-speed steering noise.

Used by both the Gazebo hardware bridge (`ros/gazebo_hardware_bridge.py`) and the real robot hardware node.

## Problem

The `tricycle_steering_controller` computes steering angle via `atan(wheelbase * wz / vx)`. As vx approaches zero, the same small perturbation in vx or wz produces increasingly violent steering swings. At vx=0 exactly, it's a division-by-zero singularity producing instant full-lock flips.

The controller itself has no filtering (`// TODO(destogl): add limiter for the velocities` in source). So filtering must happen downstream, in the hardware bridge.

## Pipeline

`SteeringFilter` (`ros/steering_filter.py`) applies four stages in series:

```
raw_cmd -> [1. Sanitize] -> [2. Alpha-blend guard] -> [3. Angle clamp] -> [4. Rate limiter] -> output
```

1. **Sanitize**: Replace NaN/Inf with 0.0
2. **Alpha-blend guard**: At vx < `vx_threshold`, blend toward last output (`alpha = vx / vx_threshold`). Prevents full-lock flips at exact zero.
3. **Angle clamp**: Clamp to `[-max_angle, +max_angle]` *before* rate limiting. Critical: the rate limiter must chase the achievable target, not waste budget on unreachable angles.
4. **Speed-scaled rate limiter**: `max_rate = min(rate_max, rate_base + rate_gain * |vx|)`. Tight at low speed (where atan sensitivity is high), transparent at driving speed.

The output state (`last_cmd`) tracks the final post-everything value. This is load-bearing: if it tracked a pre-saturation value, the rate limiter's delta reference would drift from the actual servo position during sustained saturation.

## Parameters

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `vx_threshold` | 0.05 | m/s | Speed below which alpha-blend guard activates |
| `max_angle` | 0.4887 | rad | Physical steering angle limit (28 deg, from URDF) |
| `rate_base` | 0.5 | rad/s | Steering rate limit at vx=0 |
| `rate_gain` | 10.0 | (rad/s)/(m/s) | Additional rate per unit speed |
| `rate_max` | 6.0 | rad/s | Absolute cap on steering rate |
| `dt_min` | 0.005 | s | Minimum dt clamp (reject zero-dt ticks) |
| `dt_max` | 0.15 | s | Maximum dt clamp (prevent timing hiccups from defeating limiter) |

Effective rate limit at various speeds:

| vx (m/s) | max_rate (rad/s) | Behavior |
|-----------|-----------------|----------|
| 0.0 | 0.5 | Heavy limiting (complements alpha-blend) |
| 0.1 | 1.5 | Damps low-speed oscillation |
| 0.3 | 3.5 | Responsive (0->full lock in 0.13s) |
| 0.55+ | 6.0 | Capped at servo physical limit |

## Integration

### Gazebo bridge

```python
from steering_filter import SteeringFilter

# In __init__:
self._steering_filter = SteeringFilter(
    vx_threshold=self.get_parameter('vx_steer_threshold').value,
    max_angle=self.get_parameter('max_steering_angle_rad').value,
    rate_base=self.get_parameter('steering_rate_base').value,
    rate_gain=self.get_parameter('steering_rate_gain_per_mps').value,
    rate_max=self.get_parameter('steering_rate_max').value,
)

# In command_callback:
pos_cmd = self._steering_filter.update(pos_cmd, vx_approx)
```

ROS parameter changes are synced to the filter via `add_on_set_parameters_callback`. The ROS parameter names map to filter attributes:

| ROS Parameter | Filter Attribute |
|---------------|-----------------|
| `vx_steer_threshold` | `vx_threshold` |
| `max_steering_angle_rad` | `max_angle` |
| `steering_rate_base` | `rate_base` |
| `steering_rate_gain_per_mps` | `rate_gain` |
| `steering_rate_max` | `rate_max` |

### Real robot hardware node

Same import, same `update()` call. The real robot should use `max_angle=0.576` (33 deg mechanical limit) instead of the sim's 0.4887 (28 deg URDF limit).

## Sim Baseline Measurements

Singularity isolation test (`drive_singularity_test.py`): 3 cycles of forward/reverse at vx=0.1 m/s with near-max steering (24.8 deg).

| Condition | Actual RMS rate | Actual flip rate | Cmd RMS rate | Cmd flip rate |
|-----------|----------------|-----------------|-------------|--------------|
| No filter (both off) | 0.565 rad/s | 3.5 Hz | 1.213 rad/s | 0.4 Hz |
| Full filter (defaults) | 0.567 rad/s | 3.5 Hz | 1.235 rad/s | 0.4 Hz |

The sim numbers are nearly identical because Gazebo physics damping absorbs most steering jitter before it reaches the joint state feedback. The filter's effect will be more visible on real hardware, where there is no physics engine between the commanded angle and the servo.

**Metrics explained:**
- **Actual** = from `/robot_joint_states` (Gazebo physics response, damped/lagged)
- **Cmd** = from `/robot_joint_commands` (raw controller output, pre-bridge). Unchanged by bridge-side filtering by design -- this is the input signal.

## Files

| File | Purpose |
|------|---------|
| `ros/steering_filter.py` | The filter module (no ROS dependencies) |
| `ros/test_steering_filter.py` | Unit tests (25 tests, pytest) |
| `ros/gazebo_hardware_bridge.py` | Sim integration |
| `drive_singularity_test.py` | Host-side singularity isolation test |
| `driving_instructions/singularity_3cycles.yaml` | Test pattern (3x forward/reverse cycles) |

## Testing

```bash
# Unit tests (no ROS needed, runs on host)
cd ros && python -m pytest test_steering_filter.py -v

# Singularity isolation test (requires jetacker:base stack)
python drive_singularity_test.py              # filter ON
python drive_singularity_test.py --guard-off  # alpha-blend guard OFF (rate limiter still active)
```

## Known Limitations

- At very low rate limits, small deltas may fall into servo deadband or static friction, creating a staircase instead of smooth motion. Gazebo won't reveal this.
- The `rate_max` cap means the filter offers no protection above ~0.5 m/s. This is intentional -- MPPI noise in the atan is not the dominant problem at normal driving speed.
- Forward/reverse may need asymmetric gains if chatter differs by direction. Not implemented yet.

