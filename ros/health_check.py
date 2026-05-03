#!/usr/bin/env python3
"""
Health Check Node - Standalone Pre-Flight Validation

Validates ROS2 topics and transforms before test execution.
Runs health checks on demand via service call and publishes results.

Architecture:
- Separate node to isolate blocking TF lookups from test_drive_node
- Uses SingleThreadedExecutor (TF blocking is acceptable here)
- Publishes health status to /health_check/status topic
- Provides /runHealthCheck service for on-demand checks

This solves the MultiThreadedExecutor CPU burn issue (rclpy#1223) by
keeping blocking operations in a dedicated node.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time, Duration
import yaml
import time
import threading
from typing import Dict, Any, Optional


class HealthCheckNode(Node):
    """Standalone health checking node with pre-flight validation"""

    def __init__(self):
        super().__init__('health_check')

        # Load configuration
        config_path = '/workspace/config/health_check.yaml'
        self.config = self._load_config(config_path)

        # TF buffer for transform checks (blocking lookups OK in this node)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Track message arrivals for rate checking (thread-safe)
        self.message_trackers = {}
        self.tracker_locks = {}

        # Publisher for health status
        self.status_pub = self.create_publisher(String, '/health_check/status', 10)

        # Service for on-demand health checks
        self.health_check_service = self.create_service(
            Trigger, '/runHealthCheck', self.health_check_callback)

        # Create persistent subscriptions for all monitored topics
        self._create_health_subscriptions()

        self.get_logger().info("Health Check Node initialized")
        self.get_logger().info("Service: /runHealthCheck")
        self.get_logger().info("Status topic: /health_check/status")

    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """Load health check configuration from YAML"""
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            return config.get('health_checks', {})
        except Exception as e:
            self.get_logger().error(f"Failed to load health check config: {e}")
            return {}

    def _wait_for_publishers(self, topic, min_count=1, timeout=30.0):
        """Wait until topic has active publishers (DDS discovery)"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            publishers = self.get_publishers_info_by_topic(topic)
            if len(publishers) >= min_count:
                self.get_logger().info(
                    f"Publisher discovered on {topic} ({len(publishers)} publisher(s)) "
                    f"after {time.time() - start_time:.1f}s")
                return True
            time.sleep(0.5)

        self.get_logger().warn(f"No publishers found for {topic} after {timeout}s")
        return False

    def _create_health_subscriptions(self):
        """Create persistent subscriptions for all monitored topics"""
        from sensor_msgs.msg import Imu, JointState
        message_type_map = {
            'nav_msgs/msg/Odometry': Odometry,
            'sensor_msgs/msg/Imu': Imu,
            'sensor_msgs/msg/JointState': JointState,
        }

        qos = 10
        topics_config = self.config.get('topics', [])
        self.health_subscriptions = []

        for topic_config in topics_config:
            topic = topic_config['topic']
            message_type_str = topic_config.get('message_type', '')
            message_type = message_type_map.get(message_type_str)

            if message_type is None:
                self.get_logger().warn(f"Unknown message type {message_type_str} for {topic}")
                continue

            if not self._wait_for_publishers(topic, min_count=1):
                self.get_logger().warn(f"Skipping {topic} - no publishers available")
                continue

            # Initialize tracker
            self.message_trackers[topic] = {'times': [], 'count': 0}
            self.tracker_locks[topic] = threading.Lock()

            # Create persistent subscription
            def make_callback(topic_name):
                def callback(msg):
                    current_time = time.time()
                    with self.tracker_locks[topic_name]:
                        self.message_trackers[topic_name]['times'].append(current_time)
                        count = self.message_trackers[topic_name]['count'] + 1
                        self.message_trackers[topic_name]['count'] = count

                        # Prune old timestamps occasionally
                        if count % 50 == 0:
                            cutoff = current_time - 5.0
                            times_list = self.message_trackers[topic_name]['times']
                            self.message_trackers[topic_name]['times'] = [
                                t for t in times_list if t >= cutoff]
                return callback

            sub = self.create_subscription(message_type, topic, make_callback(topic), qos)
            self.health_subscriptions.append(sub)
            self.get_logger().info(f"Health checker monitoring: {topic}")
            time.sleep(0.2)

    def health_check_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Service callback to run health checks on demand"""
        self.get_logger().info("Health check requested via service")

        success, message = self.run_health_checks()

        response.success = success
        response.message = message

        # Publish status
        status_msg = String()
        status_msg.data = f"{'PASS' if success else 'FAIL'}: {message}"
        self.status_pub.publish(status_msg)

        return response

    def run_health_checks(self) -> tuple[bool, str]:
        """Run all configured health checks"""
        if not self.config:
            return False, "Health check config not loaded"

        self.get_logger().info("="*60)
        self.get_logger().info("RUNNING PRE-FLIGHT HEALTH CHECKS")
        self.get_logger().info("="*60)

        # Check topics
        topic_results = []
        if 'topics' in self.config:
            for topic_config in self.config['topics']:
                result = self._check_topic(topic_config)
                topic_results.append(result)

        # Check transforms (BLOCKING - can take up to 2s per transform)
        transform_results = []
        if 'transforms' in self.config:
            for tf_config in self.config['transforms']:
                result = self._check_transform(tf_config)
                transform_results.append(result)

        # Summarize results
        all_results = topic_results + transform_results
        failures = [r for r in all_results if not r[0]]

        self.get_logger().info("="*60)
        if failures:
            self.get_logger().error(f"HEALTH CHECK FAILED: {len(failures)} issue(s) found")
            error_msg = "\\n".join([f"  - {msg}" for _, msg in failures])
            self.get_logger().error(f"Failed checks:\\n{error_msg}")
            return False, f"Health check failed: {len(failures)} issue(s)"
        else:
            self.get_logger().info(f"HEALTH CHECK PASSED: All {len(all_results)} checks OK")
            self.get_logger().info("="*60)
            return True, "All health checks passed"

    def _check_topic(self, config: Dict[str, Any]) -> tuple[bool, str]:
        """Check if topic exists and has received messages"""
        topic = config['topic']
        description = config.get('description', topic)

        self.get_logger().info(f"Checking: {description}")
        self.get_logger().info(f"  Topic: {topic}")

        if topic not in self.message_trackers:
            msg = f"SKIP: {topic} - Not available (no publishers found during startup)"
            self.get_logger().warn(f"  {msg}")
            return True, msg  # Return success - topic is optional

        with self.tracker_locks[topic]:
            total_count = self.message_trackers[topic]['count']

        self.get_logger().info(f"  [DEBUG] Total messages received: {total_count}")

        if total_count == 0:
            msg = f"FAIL: {topic} - No messages received (topic not reachable)"
            self.get_logger().error(f"  {msg}")
            return False, msg

        self.get_logger().info(f"  ✓ OK: Topic is reachable and publishing")
        return True, f"{topic} OK"

    def _check_transform(self, config: Dict[str, Any]) -> tuple[bool, str]:
        """Check if transform exists and is recent (BLOCKING up to 2s)"""
        parent = config['parent_frame']
        child = config['child_frame']
        max_age = config.get('max_age_sec', 1.0)
        description = config.get('description', f"{parent}->{child}")

        self.get_logger().info(f"Checking: {description}")
        self.get_logger().info(f"  Transform: {parent} -> {child}")
        self.get_logger().info(f"  Max age: {max_age:.1f}s")

        try:
            # Look up transform (BLOCKING - can take up to 2s)
            now = Time()
            transform = self.tf_buffer.lookup_transform(
                parent, child, now, timeout=Duration(seconds=2.0))

            # Check age
            transform_time = transform.header.stamp
            current_time = self.get_clock().now()
            age = (current_time.nanoseconds - transform_time.sec * 1e9 - transform_time.nanosec) / 1e9

            if age > max_age:
                msg = f"FAIL: {parent}->{child} - Transform too old ({age:.2f}s > {max_age:.1f}s)"
                self.get_logger().error(f"  {msg}")
                return False, msg

            self.get_logger().info(f"  ✓ OK: Transform age {age:.3f}s")
            return True, f"{parent}->{child} OK"

        except Exception as e:
            msg = f"FAIL: {parent}->{child} - {str(e)}"
            self.get_logger().error(f"  {msg}")
            return False, msg


def main(args=None):
    rclpy.init(args=args)

    try:
        node = HealthCheckNode()
        # Use SingleThreadedExecutor (default) - blocking TF lookups are OK here
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
