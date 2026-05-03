#!/usr/bin/env python3
"""
In-process rosbag recorder using rosbag2_py.SequentialWriter.

Attaches typed subscriptions to an existing ROS2 node.
No subprocess, no extra DDS participant, no zombie risk.

Usage:
    from bag_recorder import BagRecorder, NAV2_RECORD_TOPICS

    recorder = BagRecorder(node, '/workspace/logs/rosbags/nav2_001', NAV2_RECORD_TOPICS)
    # ... run node ...
    recorder.close()
"""

import os
import time

from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ---------------------------------------------------------------------------
# QoS profiles for recording
# ---------------------------------------------------------------------------
# BEST_EFFORT receives from both RELIABLE and BEST_EFFORT publishers
QOS_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)

# TRANSIENT_LOCAL for latched static transforms
QOS_STATIC_TF = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10,
)

# ---------------------------------------------------------------------------
# Message imports (deferred into functions to avoid import errors when
# a message package is missing from the container)
# ---------------------------------------------------------------------------

def _nav2_record_topics():
    """Return topic list for Nav2 recording."""
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, PolygonStamped
    from nav_msgs.msg import Path, OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    topics = [
        ('/amcl_pose', 'geometry_msgs/msg/PoseWithCovarianceStamped', PoseWithCovarianceStamped, QOS_SENSOR),
        ('/goal_pose', 'geometry_msgs/msg/PoseStamped', PoseStamped, QOS_SENSOR),
        ('/cmd_vel', 'geometry_msgs/msg/Twist', Twist, QOS_SENSOR),
        ('/cmd_vel_smoothed', 'geometry_msgs/msg/Twist', Twist, QOS_SENSOR),
        ('/jetacker/ground_truth', 'geometry_msgs/msg/PoseStamped', PoseStamped, QOS_SENSOR),
        ('/tf', 'tf2_msgs/msg/TFMessage', TFMessage, QOS_SENSOR),
        ('/tf_static', 'tf2_msgs/msg/TFMessage', TFMessage, QOS_STATIC_TF),
        ('/scan', 'sensor_msgs/msg/LaserScan', LaserScan, QOS_SENSOR),
        ('/tof_rear_left', 'sensor_msgs/msg/LaserScan', LaserScan, QOS_SENSOR),
        ('/tof_rear_right', 'sensor_msgs/msg/LaserScan', LaserScan, QOS_SENSOR),
        ('/plan', 'nav_msgs/msg/Path', Path, QOS_SENSOR),
        ('/optimal_trajectory', 'nav_msgs/msg/Path', Path, QOS_SENSOR),
        ('/transformed_global_plan', 'nav_msgs/msg/Path', Path, QOS_SENSOR),
        ('/unsmoothed_plan', 'nav_msgs/msg/Path', Path, QOS_SENSOR),
        ('/local_costmap/costmap', 'nav_msgs/msg/OccupancyGrid', OccupancyGrid, QOS_SENSOR),
        ('/local_costmap/published_footprint', 'geometry_msgs/msg/PolygonStamped', PolygonStamped, QOS_SENSOR),
        ('/global_costmap/costmap', 'nav_msgs/msg/OccupancyGrid', OccupancyGrid, QOS_SENSOR),
    ]

    # CollisionMonitorState may not be available in all containers
    try:
        from nav2_msgs.msg import CollisionMonitorState
        topics.append(('/collision_monitor_state', 'nav2_msgs/msg/CollisionMonitorState', CollisionMonitorState, QOS_SENSOR))
    except ImportError:
        pass

    # CriticsStats from patched MPPI controller (critics overlay)
    # Mounted at /workspace/critics_install/ via docker-compose.critics.yml
    try:
        import sys, ctypes
        _critics_base = '/workspace/critics_install/nav2_critics_msgs'
        _pypath = f'{_critics_base}/lib/python3.12/site-packages'
        _libpath = f'{_critics_base}/lib'
        if os.path.isdir(_pypath):
            if _pypath not in sys.path:
                sys.path.insert(0, _pypath)
            # Preload shared libraries so dlopen finds them
            for _so in sorted(os.listdir(_libpath)):
                if _so.endswith('.so'):
                    try:
                        ctypes.cdll.LoadLibrary(os.path.join(_libpath, _so))
                    except OSError:
                        pass
        from nav2_critics_msgs.msg import CriticsStats
        topics.append(('/controller_server/critics_stats', 'nav2_critics_msgs/msg/CriticsStats', CriticsStats, QOS_SENSOR))
    except (ImportError, FileNotFoundError, OSError):
        pass

    return topics


def _drive_record_topics():
    """Return topic list for drive test recording."""
    from geometry_msgs.msg import Twist, TwistStamped
    from sensor_msgs.msg import JointState
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String
    from tf2_msgs.msg import TFMessage

    topics = [
        ('/cmd_vel', 'geometry_msgs/msg/Twist', Twist, QOS_SENSOR),
        ('/tricycle_steering_controller/reference', 'geometry_msgs/msg/TwistStamped', TwistStamped, QOS_SENSOR),
        ('/joint_states', 'sensor_msgs/msg/JointState', JointState, QOS_SENSOR),
        ('/robot_joint_states', 'sensor_msgs/msg/JointState', JointState, QOS_SENSOR),
        ('/robot_joint_commands', 'sensor_msgs/msg/JointState', JointState, QOS_SENSOR),
        ('/odometry/filtered', 'nav_msgs/msg/Odometry', Odometry, QOS_SENSOR),
        ('/tf', 'tf2_msgs/msg/TFMessage', TFMessage, QOS_SENSOR),
        ('/tf_static', 'tf2_msgs/msg/TFMessage', TFMessage, QOS_STATIC_TF),
        ('/test_drive/executing', 'std_msgs/msg/Bool', Bool, QOS_SENSOR),
        ('/test_drive/error', 'std_msgs/msg/Bool', Bool, QOS_SENSOR),
        ('/test_drive/result', 'std_msgs/msg/String', String, QOS_SENSOR),
        ('/test_drive/feedback', 'std_msgs/msg/String', String, QOS_SENSOR),
        ('/test_drive/waypoints', 'std_msgs/msg/String', String, QOS_SENSOR),
    ]

    # Add simulation-specific topics
    is_simulation = os.environ.get('IS_SIMULATION', 'false').lower() == 'true'
    if is_simulation:
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Float64

        topics.extend([
            ('/gz_cmd/front_left_wheel_steering_joint/position', 'std_msgs/msg/Float64', Float64, QOS_SENSOR),
            ('/gz_cmd/front_right_wheel_steering_joint/position', 'std_msgs/msg/Float64', Float64, QOS_SENSOR),
            ('/gz_cmd/rear_left_wheel_joint/velocity', 'std_msgs/msg/Float64', Float64, QOS_SENSOR),
            ('/gz_cmd/rear_right_wheel_joint/velocity', 'std_msgs/msg/Float64', Float64, QOS_SENSOR),
            ('/jetacker/ground_truth', 'geometry_msgs/msg/PoseStamped', PoseStamped, QOS_SENSOR),
        ])

    return topics


def make_bag_path(bag_dir, cycle_num=None, prefix='nav2'):
    """Generate a timestamped bag directory path.

    Args:
        bag_dir: Parent directory for bags
        cycle_num: Optional cycle number
        prefix: Filename prefix ('nav2' or 'test')

    Returns:
        str: Full path for the bag directory
    """
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    if cycle_num is not None:
        bag_name = f'{prefix}_cycle{cycle_num:03d}_{timestamp}'
    else:
        bag_name = f'{prefix}_{timestamp}'
    return os.path.join(bag_dir, bag_name)


class BagRecorder:
    """In-process rosbag recorder using SequentialWriter.

    Attaches typed subscriptions to an existing ROS2 node.
    No subprocess, no extra DDS participant, no zombie risk.
    """

    def __init__(self, node, bag_path, topics):
        """
        Args:
            node: rclpy.Node to attach subscriptions to
            bag_path: Path for output bag directory
            topics: list of (topic_name, msg_type_str, msg_class, qos_profile)
        """
        self._node = node
        self._bag_path = bag_path
        self._closed = False

        # Ensure parent directory exists
        os.makedirs(os.path.dirname(bag_path), exist_ok=True)

        # Open writer
        storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        )

        self._writer = SequentialWriter()
        self._writer.open(storage_options, converter_options)

        # Create topic metadata and subscriptions
        self._subscriptions = []
        for topic_name, type_str, msg_class, qos in topics:
            # Register topic with writer
            topic_meta = TopicMetadata(
                id=0,
                name=topic_name,
                type=type_str,
                serialization_format='cdr',
            )
            self._writer.create_topic(topic_meta)

            # Create subscription that writes to bag
            sub = node.create_subscription(
                msg_class,
                topic_name,
                self._make_callback(topic_name),
                qos,
            )
            self._subscriptions.append(sub)

        node.get_logger().info(f'[BAG] Recording {len(topics)} topics to {bag_path}')

    def _make_callback(self, topic_name):
        """Create a subscription callback that writes messages to the bag."""
        def callback(msg):
            if self._closed:
                return
            try:
                serialized = serialize_message(msg)
                timestamp_ns = self._node.get_clock().now().nanoseconds
                self._writer.write(topic_name, serialized, timestamp_ns)
            except Exception:
                pass  # Don't crash the node if bag write fails
        return callback

    @property
    def bag_path(self):
        return self._bag_path

    def close(self):
        """Close the writer. Safe to call multiple times."""
        if self._closed:
            return
        self._closed = True
        try:
            del self._writer
        except Exception:
            pass
        self._writer = None
        self._node.get_logger().info(f'[BAG] Recording closed: {self._bag_path}')
