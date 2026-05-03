#!/usr/bin/env python3
"""Verify episode_metrics output against rosbag ground truth."""

import math
import json
import sys
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def analyze_bag(bag_path):
    reader = SequentialReader()
    storage = StorageOptions(uri=bag_path, storage_id="mcap")
    converter = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage, converter)

    steering_samples = []  # (timestamp_ns, angle)
    cmdvel_samples = []    # (timestamp_ns, vx, wz)
    episode_metrics = None

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()

        if topic == "/joint_states":
            msg = deserialize_message(data, JointState)
            names = list(msg.name)
            if "front_steering_joint" in names:
                idx = names.index("front_steering_joint")
                steering_samples.append((timestamp_ns, msg.position[idx]))

        elif topic == "/cmd_vel":
            msg = deserialize_message(data, Twist)
            cmdvel_samples.append((timestamp_ns, msg.linear.x, msg.angular.z))

        elif topic == "/test_drive/result":
            msg = deserialize_message(data, String)
            result = json.loads(msg.data)
            episode_metrics = result.get("metrics", {})

    # -- Compute steering metrics from bag --
    rates = []
    for i in range(1, len(steering_samples)):
        t0, a0 = steering_samples[i - 1]
        t1, a1 = steering_samples[i]
        dt = (t1 - t0) / 1e9
        if dt > 0:
            rates.append((a1 - a0) / dt)

    rms_rate = math.sqrt(sum(r * r for r in rates) / len(rates)) if rates else 0.0

    DEADBAND = 0.02
    flips = 0
    prev_sign = 0
    for r in rates:
        if r > DEADBAND:
            sign = 1
        elif r < -DEADBAND:
            sign = -1
        else:
            continue
        if prev_sign != 0 and sign != prev_sign:
            flips += 1
        prev_sign = sign

    total_steer_time = (
        (steering_samples[-1][0] - steering_samples[0][0]) / 1e9
        if len(steering_samples) > 1
        else 0
    )
    flip_rate = flips / total_steer_time if total_steer_time > 0 else 0.0

    # -- Compute cmd_vel TV from bag --
    tv_linear = 0.0
    tv_angular = 0.0
    for i in range(1, len(cmdvel_samples)):
        _, vx0, wz0 = cmdvel_samples[i - 1]
        _, vx1, wz1 = cmdvel_samples[i]
        tv_linear += (vx1 - vx0) ** 2
        tv_angular += (wz1 - wz0) ** 2

    total_cmd_time = (
        (cmdvel_samples[-1][0] - cmdvel_samples[0][0]) / 1e9
        if len(cmdvel_samples) > 1
        else 0
    )
    if total_cmd_time > 0:
        tv_linear /= total_cmd_time
        tv_angular /= total_cmd_time

    # -- Print comparison --
    bag_name = bag_path.rstrip("/").split("/")[-1]
    em_steer = episode_metrics.get("steering", {}) if episode_metrics else {}
    em_cmd = episode_metrics.get("cmd_vel", {}) if episode_metrics else {}
    em_counts = episode_metrics.get("sample_counts", {}) if episode_metrics else {}

    print(f"\n=== {bag_name} ===")
    print(f"  Bag samples:     {len(steering_samples)} steering, {len(cmdvel_samples)} cmd_vel")
    print(f"  Episode samples: {em_counts.get('steering', '?')} steering, {em_counts.get('cmd_vel', '?')} cmd_vel")
    print(f"  Duration:        {total_steer_time:.1f}s steering, {total_cmd_time:.1f}s cmd_vel")
    print()

    hdr = f"  {'Metric':<22}  {'Rosbag':>12}  {'EpisodeMetrics':>14}  {'Delta':>10}"
    print(hdr)
    print(f"  {'-'*22}  {'-'*12}  {'-'*14}  {'-'*10}")

    rows = [
        ("Steer RMS (rad/s)", rms_rate, em_steer.get("rms_rate")),
        ("Steer flip (Hz)", flip_rate, em_steer.get("flip_rate_hz")),
        ("TV(linear)", tv_linear, em_cmd.get("tv_linear")),
        ("TV(angular)", tv_angular, em_cmd.get("tv_angular")),
    ]
    for label, bag_val, em_val in rows:
        if em_val is not None:
            delta = bag_val - em_val
            print(f"  {label:<22}  {bag_val:>12.6f}  {em_val:>14.6f}  {delta:>+10.6f}")
        else:
            print(f"  {label:<22}  {bag_val:>12.6f}  {'N/A':>14}  {'':>10}")


if __name__ == "__main__":
    bags = sys.argv[1:] or [
        "/workspace/logs/rosbags/test_cycle001_20260302_034508",
        "/workspace/logs/rosbags/test_cycle002_20260302_034532",
        "/workspace/logs/rosbags/test_cycle003_20260302_034556",
    ]
    for b in bags:
        analyze_bag(b)
    print()
