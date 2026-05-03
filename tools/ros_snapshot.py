#!/usr/bin/env python3
"""ros_snapshot.py — Dump running ROS2 system topology to YAML.

Run inside a container with ROS2 sourced:
  python3 /workspace/tools/ros_snapshot.py
  python3 /workspace/tools/ros_snapshot.py --hz
  python3 /workspace/tools/ros_snapshot.py --params -o snapshot.yaml
  python3 /workspace/tools/ros_snapshot.py --sample -o state.yaml

Or via docker exec from host:
  docker exec debug bash -c 'source /opt/ros/jazzy/setup.bash && \\
      python3 /workspace/tools/ros_snapshot.py --sample'

Output structure:
  snapshot:
    ros_domain_id: 37
    node_count: 12
    topic_count: 34
  nodes:                       # flat list by default
    - /ekf_filter_node
    - /slam_toolbox
  nodes:                       # dict form with --params
    /ekf_filter_node:
      parameters:
        frequency: 10.0
  topics:
    /odometry/filtered:
      type: nav_msgs/msg/Odometry
      hz: 9.99                 # only with --hz
      sample: {header: ...}    # only with --sample
      publishers:
        - /ekf_filter_node
      subscribers:             # omitted when empty; duplicates shown as "node [xN]"
        - /foxglove_bridge [x3]
"""

import argparse
import concurrent.futures
import fnmatch
import os
import subprocess
import sys
import time
import yaml
from collections import Counter
from datetime import datetime

# -------------------------------------------------------------------
# Default exclusion patterns
# These filter out ROS2 internals and noisy infrastructure topics/nodes.
# Override with --no-default-excludes or extend with --exclude-*/--exclude-nodes.
# -------------------------------------------------------------------

DEFAULT_EXCLUDE_TOPICS = [
    '/rosout',
    '/parameter_events',
    '*/_action/*',              # ROS2 action internal topics
    '*/transition_event',       # Lifecycle node transitions
    '*/introspection_data/*',   # Controller manager introspection
    '*/statistics/*',           # Controller manager statistics
]

DEFAULT_EXCLUDE_NODES = [
    '*/ros2cli_*',              # Transient ros2 CLI nodes
    '*/transform_listener_impl_*',  # Internal TF listener nodes
    '*/launch_ros_*',           # Launch system internal nodes
    '/ros_snapshot',            # This script itself
]


def parse_args():
    p = argparse.ArgumentParser(
        description='Dump running ROS2 system topology to YAML.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
examples:
  # Basic topology (nodes + topics with pub/sub)
  python3 ros_snapshot.py

  # Include publish rates (3s measurement window)
  python3 ros_snapshot.py --hz

  # Include node parameters
  python3 ros_snapshot.py --params

  # Everything, written to file
  python3 ros_snapshot.py --hz --params -o snapshot.yaml

  # Exclude noisy extra topics beyond defaults
  python3 ros_snapshot.py --exclude-topics '/clicked_point,/goal_pose'

  # Show raw internals too (no filtering)
  python3 ros_snapshot.py --no-default-excludes
        """,
    )
    p.add_argument('-o', '--output', metavar='FILE',
                   help='Write YAML to file instead of stdout')
    p.add_argument('--hz', action='store_true',
                   help='Measure publish rate (Hz) for each topic')
    p.add_argument('--hz-duration', type=float, default=3.0, metavar='SECS',
                   help='How long to measure Hz (default: 3.0)')
    p.add_argument('--params', action='store_true',
                   help='Include node parameters (spawns ros2 param dump per node, slower)')
    p.add_argument('--exclude-topics', metavar='PATTERNS',
                   help='Comma-separated glob patterns for extra topic exclusions')
    p.add_argument('--exclude-nodes', metavar='PATTERNS',
                   help='Comma-separated glob patterns for extra node exclusions')
    p.add_argument('--no-default-excludes', action='store_true',
                   help='Disable built-in exclusion patterns (show everything)')
    p.add_argument('--no-types', action='store_true',
                   help='Omit message types from topic entries')
    p.add_argument('--sample', action='store_true',
                   help='Capture one message sample from each active topic (via ros2 topic echo)')
    p.add_argument('--sample-timeout', type=float, default=2.0, metavar='SECS',
                   help='Timeout per topic when sampling (default: 2.0)')
    p.add_argument('--domain-id', type=int, metavar='ID',
                   help='ROS_DOMAIN_ID override (default: from ROS_DOMAIN_ID env var)')
    return p.parse_args()


def matches_any(name, patterns):
    return any(fnmatch.fnmatch(name, pat) for pat in patterns)


def make_full_name(name, namespace):
    """Build fully-qualified ROS2 node name from name + namespace."""
    ns = namespace.rstrip('/')
    return f'{ns}/{name}' if ns else f'/{name}'


def sample_topic(topic_name, timeout=2.0):
    """Capture one message from a topic via `ros2 topic echo --once`.

    Returns parsed YAML dict, or None on failure/timeout.
    Uses ros2 topic echo because create_generic_subscription is not
    available in rclpy on Jazzy.
    """
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'echo', '--once', topic_name],
            capture_output=True, text=True, timeout=timeout,
        )
        if result.returncode != 0:
            return None
        # ros2 topic echo outputs YAML-like text with --- separators
        text = result.stdout.strip()
        if not text:
            return None
        # Take only first message (before first ---)
        first_msg = text.split('\n---')[0].strip()
        if not first_msg:
            return None
        return yaml.safe_load(first_msg)
    except subprocess.TimeoutExpired:
        return None
    except Exception:
        return None


def sample_topics_parallel(topic_names, timeout=2.0, max_workers=8):
    """Sample multiple topics in parallel. Returns dict of topic_name -> sample."""
    results = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = {
            executor.submit(sample_topic, name, timeout): name
            for name in topic_names
        }
        for future in concurrent.futures.as_completed(futures):
            name = futures[future]
            try:
                results[name] = future.result()
            except Exception:
                results[name] = None
    return results


def get_node_params(node_full_name):
    """Fetch parameters for one node via `ros2 param dump`. Returns dict or None."""
    try:
        result = subprocess.run(
            ['ros2', 'param', 'dump', node_full_name],
            capture_output=True, text=True, timeout=8,
        )
        if result.returncode != 0:
            return None
        data = yaml.safe_load(result.stdout)
        if not isinstance(data, dict):
            return None
        # ros2 param dump wraps in {node_name: {ros__parameters: {...}}}
        for _key, val in data.items():
            if isinstance(val, dict):
                return val.get('ros__parameters', val)
        return None
    except Exception:
        return None


def main():
    args = parse_args()

    # Set domain ID before importing rclpy
    if args.domain_id is not None:
        os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)

    # Build exclusion lists
    exclude_topics = list(DEFAULT_EXCLUDE_TOPICS) if not args.no_default_excludes else []
    exclude_nodes  = list(DEFAULT_EXCLUDE_NODES)  if not args.no_default_excludes else []

    if args.exclude_topics:
        exclude_topics.extend(p.strip() for p in args.exclude_topics.split(','))
    if args.exclude_nodes:
        exclude_nodes.extend(p.strip() for p in args.exclude_nodes.split(','))

    # ---------------------------------------------------------------
    # Import rclpy here so --help works even without ROS sourced
    # ---------------------------------------------------------------
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (
            QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        )
    except ImportError:
        print('ERROR: rclpy not found. Source ROS2 setup.bash before running.', file=sys.stderr)
        sys.exit(1)

    # ---------------------------------------------------------------
    # Create snapshot node and query the graph
    # ---------------------------------------------------------------
    rclpy.init()
    node = Node('ros_snapshot')

    eprint = lambda msg: print(msg, file=sys.stderr)

    eprint('Collecting ROS2 graph...')

    # Poll until node count stabilises (DDS discovery trickles in)
    prev_count, stable_for = 0, 0
    for _ in range(40):  # max 10s
        time.sleep(0.25)
        rclpy.spin_once(node, timeout_sec=0)
        count = len(node.get_node_names_and_namespaces())
        if count == prev_count:
            stable_for += 1
            if stable_for >= 4:  # stable for 1s
                break
        else:
            stable_for = 0
        prev_count = count

    # --- Nodes ---
    raw_nodes = {}
    for name, ns in node.get_node_names_and_namespaces():
        full = make_full_name(name, ns)
        raw_nodes[full] = {}

    # --- Topics ---
    raw_topics = {}
    for topic, types in node.get_topic_names_and_types():
        pubs = node.get_publishers_info_by_topic(topic)
        subs = node.get_subscriptions_info_by_topic(topic)
        def resolve_endpoints(endpoints):
            known, unknown = [], 0
            for e in endpoints:
                if e.node_name == '_NODE_NAME_UNKNOWN_':
                    unknown += 1
                else:
                    full = make_full_name(e.node_name, e.node_namespace)
                    if not matches_any(full, exclude_nodes):
                        known.append(full)
            result = sorted(set(known))
            if unknown:
                result.append(f'<{unknown} unresolved>')
            return result

        pub_list = resolve_endpoints(pubs)
        sub_list = resolve_endpoints(subs)
        info = {'type': types[0] if types else 'unknown'}
        if pub_list:
            info['publishers'] = pub_list
        if sub_list:
            info['subscribers'] = sub_list
        raw_topics[topic] = info

    # ---------------------------------------------------------------
    # Apply filters
    # ---------------------------------------------------------------
    nodes = {
        n: info for n, info in raw_nodes.items()
        if not matches_any(n, exclude_nodes)
    }
    topics = {
        t: info for t, info in raw_topics.items()
        if not matches_any(t, exclude_topics)
    }

    # ---------------------------------------------------------------
    # Hz measurement
    # ---------------------------------------------------------------
    if args.hz:
        eprint(f'Measuring Hz over {args.hz_duration}s (subscribing to {len(topics)} topics)...')

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        counts = {}
        subs = []

        def make_cb(topic_name):
            def cb(_msg):
                counts[topic_name] = counts.get(topic_name, 0) + 1
            return cb

        skipped = []
        for topic_name, info in topics.items():
            msg_type = info.get('type', '')
            if not msg_type or msg_type == 'unknown':
                skipped.append(topic_name)
                continue
            # Only subscribe to topics that actually have publishers
            if not info.get('publishers'):
                counts[topic_name] = 0
                continue
            try:
                sub = node.create_generic_subscription(
                    topic_name, msg_type, best_effort_qos, make_cb(topic_name),
                )
                subs.append(sub)
            except Exception as e:
                eprint(f'  Warning: could not subscribe to {topic_name}: {e}')
                skipped.append(topic_name)

        # Spin for the measurement window
        deadline = time.monotonic() + args.hz_duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        for sub in subs:
            node.destroy_subscription(sub)

        for topic_name in topics:
            hz = counts.get(topic_name, 0) / args.hz_duration
            topics[topic_name]['hz'] = round(hz, 2)

        if skipped:
            eprint(f'  Skipped {len(skipped)} topics (unknown type or no publishers)')

    # ---------------------------------------------------------------
    # Topic sampling (one message per active topic)
    # ---------------------------------------------------------------
    if args.sample:
        # Only sample topics that have publishers
        sampleable = [t for t, info in topics.items() if info.get('publishers')]
        eprint(f'Sampling {len(sampleable)} topics (timeout {args.sample_timeout}s each)...')

        samples = sample_topics_parallel(sampleable, timeout=args.sample_timeout)

        sampled = 0
        for topic_name, sample in samples.items():
            if sample is not None:
                topics[topic_name]['sample'] = sample
                sampled += 1

        eprint(f'  Sampled {sampled}/{len(sampleable)} topics')

    # ---------------------------------------------------------------
    # Node parameters
    # ---------------------------------------------------------------
    if args.params:
        eprint(f'Fetching parameters for {len(nodes)} nodes...')
        for node_name in sorted(nodes):
            eprint(f'  {node_name}')
            params = get_node_params(node_name)
            if params:
                nodes[node_name]['parameters'] = params

    node.destroy_node()
    rclpy.shutdown()

    # ---------------------------------------------------------------
    # Optionally strip message types
    # ---------------------------------------------------------------
    if args.no_types:
        for info in topics.values():
            info.pop('type', None)

    # ---------------------------------------------------------------
    # Build output
    # ---------------------------------------------------------------
    # Clean up topics: convert single-item publisher lists to scalar for readability
    # (keep as list regardless — consistent structure is easier to parse)

    if args.params:
        node_out = {n: {'parameters': info['parameters']} for n, info in sorted(nodes.items()) if 'parameters' in info}
        # Include nodes without params too (empty dict)
        for n in sorted(nodes):
            node_out.setdefault(n, {})
    else:
        node_out = sorted(nodes.keys())

    output = {
        'snapshot': {
            'timestamp': datetime.now().isoformat(timespec='seconds'),
            'ros_domain_id': int(os.environ.get('ROS_DOMAIN_ID', 0)),
            'node_count': len(nodes),
            'topic_count': len(topics),
        },
        'nodes': node_out,
        'topics': dict(sorted(topics.items())),
    }

    yaml_str = yaml.dump(output, default_flow_style=False, sort_keys=False, allow_unicode=True)

    if args.output:
        with open(args.output, 'w') as f:
            f.write(yaml_str)
        eprint(f'Written to {args.output} ({len(nodes)} nodes, {len(topics)} topics)')
    else:
        print(yaml_str)


if __name__ == '__main__':
    main()
