#!/usr/bin/env python3
"""Analyze critics_stats from a rosbag MCAP file."""

import sys
import os

# Preload critics_msgs
_critics_base = '/workspace/critics_install/nav2_critics_msgs'
_pypath = f'{_critics_base}/lib/python3.12/site-packages'
_libpath = f'{_critics_base}/lib'
if os.path.isdir(_pypath):
    if _pypath not in sys.path:
        sys.path.insert(0, _pypath)
    import ctypes
    for _so in sorted(os.listdir(_libpath)):
        if _so.endswith('.so'):
            try:
                ctypes.cdll.LoadLibrary(os.path.join(_libpath, _so))
            except OSError:
                pass

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav2_critics_msgs.msg import CriticsStats
from geometry_msgs.msg import Twist
import statistics


def read_bag(bag_path):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage_options, converter_options)

    critics_msgs = []
    cmd_vel_msgs = []

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic == '/controller_server/critics_stats':
            msg = deserialize_message(data, CriticsStats)
            critics_msgs.append((timestamp_ns, msg))
        elif topic == '/cmd_vel':
            msg = deserialize_message(data, Twist)
            cmd_vel_msgs.append((timestamp_ns, msg))

    return critics_msgs, cmd_vel_msgs


def has_costs_best(msg):
    """Check if message has costs_best field (new format)."""
    return hasattr(msg, 'costs_best') and len(msg.costs_best) > 0


def analyze_critics(critics_msgs):
    if not critics_msgs:
        print("No critics_stats messages found!")
        return

    print(f"=== Critics Stats Analysis ===")
    print(f"Total messages: {len(critics_msgs)}")

    t0 = critics_msgs[0][0]
    t1 = critics_msgs[-1][0]
    duration = (t1 - t0) / 1e9
    print(f"Duration: {duration:.1f}s")
    print(f"Rate: {len(critics_msgs) / duration:.1f} Hz")

    first_msg = critics_msgs[0][1]
    critic_names = list(first_msg.critics)
    have_best = has_costs_best(first_msg)
    print(f"\nCritics ({len(critic_names)}): {critic_names}")
    if have_best:
        print("(costs_best available — showing BEST trajectory costs)")

    # Collect per-critic costs (sum and best)
    per_critic_sum = {name: [] for name in critic_names}
    per_critic_best = {name: [] for name in critic_names}
    active_count_best = {name: 0 for name in critic_names}

    for _, msg in critics_msgs:
        for i, name in enumerate(msg.critics):
            per_critic_sum[name].append(msg.costs_sum[i])
            if have_best:
                best_cost = msg.costs_best[i]
                per_critic_best[name].append(best_cost)
                if best_cost > 0:
                    active_count_best[name] += 1

    total_msgs = len(critics_msgs)

    # === BEST trajectory costs (the one that matters) ===
    if have_best:
        print(f"\n{'Critic':<35} {'Mean':>8} {'Median':>8} {'Min':>8} {'Max':>8} {'Active%':>8}")
        print("-" * 85)
        for name in critic_names:
            costs = per_critic_best[name]
            if not costs:
                continue
            mean_c = statistics.mean(costs)
            med_c = statistics.median(costs)
            min_c = min(costs)
            max_c = max(costs)
            active_pct = 100.0 * active_count_best[name] / total_msgs
            print(f"{name:<35} {mean_c:>8.2f} {med_c:>8.2f} {min_c:>8.2f} {max_c:>8.2f} {active_pct:>7.1f}%")

    # === Batch sum costs (for reference) ===
    print(f"\n--- Batch sum (all {2000} trajectories) ---")
    print(f"{'Critic':<35} {'Mean':>8} {'Median':>8} {'Min':>8} {'Max':>8}")
    print("-" * 75)
    for name in critic_names:
        costs = per_critic_sum[name]
        if not costs:
            continue
        mean_c = statistics.mean(costs)
        med_c = statistics.median(costs)
        min_c = min(costs)
        max_c = max(costs)
        print(f"{name:<35} {mean_c:>8.0f} {med_c:>8.0f} {min_c:>8.0f} {max_c:>8.0f}")

    # Time series (best trajectory costs)
    cost_field = 'costs_best' if have_best else 'costs_sum'
    print(f"\n=== First 5 ticks ({cost_field}) ===")
    for ts, msg in critics_msgs[:5]:
        t_rel = (ts - t0) / 1e9
        costs = msg.costs_best if have_best else msg.costs_sum
        costs_str = "  ".join(f"{n}={c:.2f}" for n, c in zip(msg.critics, costs))
        print(f"  t={t_rel:6.2f}s  {costs_str}")

    print(f"\n=== Last 5 ticks ({cost_field}) ===")
    for ts, msg in critics_msgs[-5:]:
        t_rel = (ts - t0) / 1e9
        costs = msg.costs_best if have_best else msg.costs_sum
        costs_str = "  ".join(f"{n}={c:.2f}" for n, c in zip(msg.critics, costs))
        print(f"  t={t_rel:6.2f}s  {costs_str}")

    # Dominant critic (best trajectory)
    print(f"\n=== Dominant Critic per Tick ({cost_field}) ===")
    dominant_counts = {name: 0 for name in critic_names}
    for _, msg in critics_msgs:
        costs = msg.costs_best if have_best else msg.costs_sum
        max_cost = -1
        max_name = ""
        for n, c in zip(msg.critics, costs):
            if c > max_cost:
                max_cost = c
                max_name = n
        if max_name:
            dominant_counts[max_name] += 1

    for name in sorted(dominant_counts, key=dominant_counts.get, reverse=True):
        cnt = dominant_counts[name]
        if cnt > 0:
            pct = 100.0 * cnt / total_msgs
            print(f"  {name:<35} {cnt:>4} ticks ({pct:>5.1f}%)")


def analyze_cmd_vel(cmd_vel_msgs, critics_msgs):
    if not cmd_vel_msgs:
        print("\nNo cmd_vel messages found.")
        return

    t0 = critics_msgs[0][0] if critics_msgs else cmd_vel_msgs[0][0]
    print(f"\n=== cmd_vel Summary ===")
    print(f"Total messages: {len(cmd_vel_msgs)}")

    vx_vals = [m.linear.x for _, m in cmd_vel_msgs]
    wz_vals = [m.angular.z for _, m in cmd_vel_msgs]

    print(f"linear.x:  mean={statistics.mean(vx_vals):.3f}  min={min(vx_vals):.3f}  max={max(vx_vals):.3f}")
    print(f"angular.z: mean={statistics.mean(wz_vals):.3f}  min={min(wz_vals):.3f}  max={max(wz_vals):.3f}")

    sign_changes = 0
    for i in range(1, len(vx_vals)):
        if vx_vals[i] * vx_vals[i-1] < 0:
            sign_changes += 1
    print(f"Direction reversals (linear.x sign changes): {sign_changes}")


def main():
    bag_path = sys.argv[1] if len(sys.argv) > 1 else '/workspace/logs/rosbags/nav2_20260312_071759'
    print(f"Reading bag: {bag_path}")
    critics_msgs, cmd_vel_msgs = read_bag(bag_path)
    analyze_critics(critics_msgs)
    analyze_cmd_vel(cmd_vel_msgs, critics_msgs)


if __name__ == '__main__':
    main()
