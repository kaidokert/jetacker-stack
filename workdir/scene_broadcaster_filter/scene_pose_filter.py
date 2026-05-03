#!/usr/bin/env python3
"""
SceneBroadcaster-based ground truth filter.

Subscribes to /world/<world>/pose/info (contains ALL model poses)
and republishes filtered poses for specific models.

This approach uses Gazebo's built-in SceneBroadcaster instead of
a custom WorldPosePublisher plugin.
"""

import rclpy
from rclpy.node import Node
from gz_msgs.msg import PoseV
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class ScenePoseFilter(Node):
    """Filter SceneBroadcaster poses for specific models."""

    def __init__(self):
        super().__init__('scene_pose_filter')

        # Declare parameters
        self.declare_parameter('world_name', '')
        self.declare_parameter('model_names', [])
        self.declare_parameter('use_sim_time', True)

        # Get parameters
        self.world_name = self.get_parameter('world_name').value
        self.model_names = self.get_parameter('model_names').value

        if not self.world_name:
            self.get_logger().error('world_name parameter is required')
            raise ValueError('world_name parameter is required')

        if not self.model_names:
            self.get_logger().error('model_names parameter is required')
            raise ValueError('model_names parameter is required')

        self.get_logger().info(f'Filtering poses for models: {self.model_names}')
        self.get_logger().info(f'World: {self.world_name}')

        # QoS settings for Gazebo bridge compatibility
        # Use RELIABLE to match Gazebo's SceneBroadcaster
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to SceneBroadcaster's unified pose topic (bridged to ROS2)
        ros_topic = f'/world/{self.world_name}/pose/info'
        self.subscription = self.create_subscription(
            PoseV,
            ros_topic,
            self.pose_callback,
            qos_profile
        )

        # Create publishers for each model
        self.publishers = {}
        for model_name in self.model_names:
            ros_topic = f'/{model_name}/ground_truth_scene'
            pub = self.create_publisher(PoseStamped, ros_topic, qos_profile)
            self.publishers[model_name] = pub
            self.get_logger().info(f'Publishing filtered poses to: {ros_topic}')

        # Statistics
        self.total_messages = 0
        self.filtered_counts = {name: 0 for name in self.model_names}

        # Log stats every 5 seconds
        self.stats_timer = self.create_timer(5.0, self.log_stats)

    def pose_callback(self, msg: PoseV):
        """Process unified pose message from SceneBroadcaster (bridged to ROS2)."""
        self.total_messages += 1

        # Iterate through all poses in the message
        for i, pose in enumerate(msg.pose):
            # Check if this pose has a name that matches our filter
            if i < len(msg.header):
                header = msg.header[i]
                # Extract model name from header data
                # Header contains key-value pairs, need to find 'name' key
                model_name = None
                for data_item in header.data:
                    if data_item.key == 'name' and len(data_item.value) > 0:
                        model_name = data_item.value[0]
                        break

                # If model name matches, publish filtered pose
                if model_name in self.model_names:
                    pose_stamped = PoseStamped()

                    # Convert header timestamp to ROS2 Time
                    pose_stamped.header.stamp.sec = header.stamp.sec
                    pose_stamped.header.stamp.nanosec = header.stamp.nsec
                    pose_stamped.header.frame_id = 'world'

                    # Convert pose to ROS2 PoseStamped
                    pose_stamped.pose.position.x = pose.position.x
                    pose_stamped.pose.position.y = pose.position.y
                    pose_stamped.pose.position.z = pose.position.z
                    pose_stamped.pose.orientation.x = pose.orientation.x
                    pose_stamped.pose.orientation.y = pose.orientation.y
                    pose_stamped.pose.orientation.z = pose.orientation.z
                    pose_stamped.pose.orientation.w = pose.orientation.w

                    # Publish
                    self.publishers[model_name].publish(pose_stamped)
                    self.filtered_counts[model_name] += 1

    def log_stats(self):
        """Log statistics about filtering."""
        if self.total_messages > 0:
            self.get_logger().info(
                f'Stats: total_msgs={self.total_messages}, '
                f'filtered={self.filtered_counts}'
            )


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ScenePoseFilter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
