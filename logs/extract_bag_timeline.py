#!/usr/bin/env python3
"""Extract a second-by-second timeline from a Nav2 rosbag.

Outputs JSON with ground truth, cmd_vel, TGP evolution, optimal trajectory,
and critic costs at 0.5s intervals.

Runs INSIDE the test-drive container (needs ROS2 + rosbag2_py).

Usage (from host, with critics_msgs):
    cp tools/extract_bag_timeline.py logs/
    docker compose exec -T test-drive bash -c \
      "source /opt/ros/jazzy/setup.bash && \
       source /workspace/critics_install/nav2_critics_msgs/share/nav2_critics_msgs/local_setup.bash && \
       python3 /workspace/logs/extract_bag_timeline.py \
       /workspace/logs/rosbags/BAG_NAME" > logs/timeline.json
"""

import json
import math
import os
import sys

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path

# Optional: critics stats (custom overlay package)
try:
    from nav2_critics_msgs.msg import CriticsStats
    HAS_CRITICS = True
except ImportError:
    HAS_CRITICS = False
    print("WARN: nav2_critics_msgs not available, skipping critic costs", file=sys.stderr)


def quat_to_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def path_length(pts):
    if len(pts) < 2:
        return 0.0
    return sum(math.sqrt((pts[j+1][0]-pts[j][0])**2 + (pts[j+1][1]-pts[j][1])**2)
               for j in range(len(pts)-1))


def segment_plan(plan):
    """Split plan into FWD/REV segments based on travel direction vs heading."""
    segments = []
    current_fwd = None
    seg_start = 0
    for i in range(1, len(plan)):
        dx = plan[i][0] - plan[i-1][0]
        dy = plan[i][1] - plan[i-1][1]
        d = math.sqrt(dx*dx + dy*dy)
        if d < 1e-6:
            continue
        travel = math.atan2(dy, dx)
        diff = travel - plan[i-1][2]
        while diff > math.pi: diff -= 2*math.pi
        while diff < -math.pi: diff += 2*math.pi
        fwd = abs(diff) < math.pi/2
        if current_fwd is not None and fwd != current_fwd:
            dist = path_length([(plan[j][0], plan[j][1]) for j in range(seg_start, i)])
            segments.append({
                'dir': 'FWD' if current_fwd else 'REV',
                'pts': i - seg_start,
                'dist_m': round(dist, 3),
                'yaw_start': round(math.degrees(plan[seg_start][2]), 1),
                'yaw_end': round(math.degrees(plan[i-1][2]), 1),
                'start_xy': (round(plan[seg_start][0], 3), round(plan[seg_start][1], 3)),
                'end_xy': (round(plan[i-1][0], 3), round(plan[i-1][1], 3)),
            })
            seg_start = i
        current_fwd = fwd

    if seg_start < len(plan) - 1:
        dist = path_length([(plan[j][0], plan[j][1]) for j in range(seg_start, len(plan))])
        segments.append({
            'dir': 'FWD' if current_fwd else 'REV',
            'pts': len(plan) - seg_start,
            'dist_m': round(dist, 3),
            'yaw_start': round(math.degrees(plan[seg_start][2]), 1),
            'yaw_end': round(math.degrees(plan[-1][2]), 1),
            'start_xy': (round(plan[seg_start][0], 3), round(plan[seg_start][1], 3)),
            'end_xy': (round(plan[-1][0], 3), round(plan[-1][1], 3)),
        })
    return segments


def extract(bag_path, interval=0.5):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id='mcap'),
        ConverterOptions(input_serialization_format='cdr',
                         output_serialization_format='cdr'),
    )

    cmd_vel, gt, plan_pts, tgp_all, opt_all, critics_raw = [], [], [], [], [], []
    t0 = None

    while reader.has_next():
        topic, data, ts = reader.read_next()
        t_sec = ts / 1e9
        if t0 is None:
            t0 = t_sec
        t = t_sec - t0

        if topic == '/cmd_vel':
            msg = deserialize_message(data, Twist)
            cmd_vel.append((t, msg.linear.x, msg.angular.z))

        elif topic == '/jetacker/ground_truth':
            msg = deserialize_message(data, PoseStamped)
            yaw = quat_to_yaw(msg.pose.orientation)
            gt.append((t, msg.pose.position.x, msg.pose.position.y, yaw))

        elif topic == '/plan' and not plan_pts:
            msg = deserialize_message(data, Path)
            for p in msg.poses:
                yaw = quat_to_yaw(p.pose.orientation)
                plan_pts.append((p.pose.position.x, p.pose.position.y, yaw))

        elif topic == '/transformed_global_plan':
            msg = deserialize_message(data, Path)
            pts = []
            for p in msg.poses:
                yaw = quat_to_yaw(p.pose.orientation)
                pts.append((p.pose.position.x, p.pose.position.y, yaw))
            tgp_all.append((t, pts))

        elif topic == '/optimal_trajectory':
            msg = deserialize_message(data, Path)
            pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            opt_all.append((t, pts))

        elif topic == '/controller_server/critics_stats' and HAS_CRITICS:
            msg = deserialize_message(data, CriticsStats)
            critics_raw.append((t, list(msg.critics),
                                list(msg.costs_best), list(msg.costs_sum)))

    # Compute duration
    t_max = max(
        cmd_vel[-1][0] if cmd_vel else 0,
        gt[-1][0] if gt else 0,
    )

    # Build timeline
    timeline = []
    n_samples = int(t_max / interval) + 1

    def closest(lst, target_t, max_dt=0.3):
        if not lst:
            return None
        c = min(lst, key=lambda x: abs(x[0] - target_t))
        return c if abs(c[0] - target_t) < max_dt else None

    for i in range(n_samples):
        target_t = i * interval
        row = {'t': target_t}

        cgt = closest(gt, target_t)
        if cgt:
            row.update({
                'x': round(cgt[1], 4), 'y': round(cgt[2], 4),
                'yaw_deg': round(math.degrees(cgt[3]), 1),
            })

        ccv = closest(cmd_vel, target_t)
        if ccv:
            row.update({'vx': round(ccv[1], 5), 'wz': round(ccv[2], 5)})

        ctgp = closest(tgp_all, target_t)
        if ctgp:
            pts = ctgp[1]
            row['tgp_n'] = len(pts)
            if pts:
                row['tgp_dist'] = round(path_length(pts), 3)
                row['tgp_last_yaw'] = round(math.degrees(pts[-1][2]), 0)

        copt = closest(opt_all, target_t)
        if copt:
            pts = copt[1]
            row['opt_n'] = len(pts)
            if len(pts) > 1:
                row['opt_dist'] = round(path_length(pts), 3)

        ccr = closest(critics_raw, target_t) if critics_raw else None
        if ccr:
            row['critics'] = {n: round(c, 2) for n, c in zip(ccr[1], ccr[2])}
            row['critics_sum'] = {n: round(c, 2) for n, c in zip(ccr[1], ccr[3])}

        if 'x' in row:
            timeline.append(row)

    # Plan segments
    segments = segment_plan(plan_pts) if plan_pts else []

    return {
        'bag': bag_path,
        'duration': round(t_max, 1),
        'counts': {
            'cmd_vel': len(cmd_vel),
            'ground_truth': len(gt),
            'plan_pts': len(plan_pts),
            'tgp_snapshots': len(tgp_all),
            'optimal_traj': len(opt_all),
            'critics_stats': len(critics_raw),
        },
        'critic_names': critics_raw[0][1] if critics_raw else [],
        'plan_segments': segments,
        'timeline': timeline,
    }


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <bag_path> [interval_sec]", file=sys.stderr)
        sys.exit(1)

    bag_path = sys.argv[1]
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5

    result = extract(bag_path, interval)
    json.dump(result, sys.stdout, indent=2)
