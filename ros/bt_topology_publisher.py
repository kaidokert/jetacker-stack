#!/usr/bin/env python3
"""
Publish Behavior Tree XML topology on a latched ROS topic.

This node is intentionally simple:
- publishes raw XML string on /bt_topology_xml
- publishes cached node state snapshot on /bt_node_state_snapshot
- uses TRANSIENT_LOCAL durability so late subscribers get the latest XML
- supports explicit bt_xml_path parameter
- can optionally discover bt_xml_path from bt_navigator parameters
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Optional

from nav2_msgs.msg import BehaviorTreeLog
import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String


class BTTopologyPublisher(Node):
    def __init__(self) -> None:
        super().__init__("bt_topology_publisher")

        self.declare_parameter("bt_xml_path", "")
        self.declare_parameter("topic_name", "/bt_topology_xml")
        self.declare_parameter("publish_period_sec", 5.0)
        self.declare_parameter("discover_from_bt_navigator", True)
        self.declare_parameter("bt_navigator_node", "/bt_navigator")
        self.declare_parameter("bt_navigator_param", "default_nav_to_pose_bt_xml")
        self.declare_parameter("state_topic_name", "/bt_node_state_snapshot")
        self.declare_parameter("bt_log_topic_primary", "/behavior_tree_log")
        self.declare_parameter("bt_log_topic_secondary", "/bt_navigator/behavior_tree_log")

        topic_name = str(self.get_parameter("topic_name").value)
        state_topic_name = str(self.get_parameter("state_topic_name").value)
        self.bt_xml_path = str(self.get_parameter("bt_xml_path").value).strip()
        self.publish_period_sec = float(self.get_parameter("publish_period_sec").value)
        self.discover_from_bt_navigator = bool(
            self.get_parameter("discover_from_bt_navigator").value
        )
        self.bt_navigator_node = str(self.get_parameter("bt_navigator_node").value)
        self.bt_navigator_param = str(self.get_parameter("bt_navigator_param").value)
        self.bt_log_topic_primary = str(self.get_parameter("bt_log_topic_primary").value)
        self.bt_log_topic_secondary = str(self.get_parameter("bt_log_topic_secondary").value)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.topology_publisher = self.create_publisher(String, topic_name, qos)
        self.state_publisher = self.create_publisher(String, state_topic_name, qos)

        self.create_subscription(
            BehaviorTreeLog, self.bt_log_topic_primary, self._on_bt_log, 50
        )
        self.create_subscription(
            BehaviorTreeLog, self.bt_log_topic_secondary, self._on_bt_log, 50
        )

        self.cached_xml: Optional[str] = None
        self.cached_mtime_ns: Optional[int] = None
        self.cached_path: Optional[Path] = None
        self.node_states: dict[str, str] = {}
        self.last_state_update_unix: float = 0.0

        if not self.bt_xml_path and self.discover_from_bt_navigator:
            discovered_path = self._discover_bt_xml_from_bt_navigator()
            if discovered_path:
                self.bt_xml_path = discovered_path

        if not self.bt_xml_path:
            self.get_logger().error(
                "No BT XML path configured. Set parameter 'bt_xml_path' or enable discovery."
            )
        else:
            self.get_logger().info(f"Using BT XML path: {self.bt_xml_path}")

        self._refresh_xml_cache()
        self._publish_xml()
        self._publish_state_snapshot()
        self.timer = self.create_timer(self.publish_period_sec, self._tick)

    def _discover_bt_xml_from_bt_navigator(self) -> Optional[str]:
        client = AsyncParameterClient(self, self.bt_navigator_node)
        if not client.wait_for_services(timeout_sec=2.0):
            self.get_logger().warn(
                f"Parameter services unavailable for {self.bt_navigator_node}; "
                "falling back to bt_xml_path parameter"
            )
            return None

        future = client.get_parameters([self.bt_navigator_param])
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done():
            self.get_logger().warn(
                f"Timed out reading '{self.bt_navigator_param}' from {self.bt_navigator_node}"
            )
            return None

        try:
            values = future.result()
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(f"Failed to read bt_navigator parameter: {exc}")
            return None

        if not values:
            return None

        candidate = values[0].string_value.strip()
        if candidate:
            self.get_logger().info(
                f"Discovered BT XML from {self.bt_navigator_node}:{self.bt_navigator_param}"
            )
            return candidate
        return None

    def _refresh_xml_cache(self) -> None:
        if not self.bt_xml_path:
            return

        path = Path(self.bt_xml_path)
        if not path.exists():
            self.get_logger().error(f"BT XML file does not exist: {self.bt_xml_path}")
            return

        mtime_ns = path.stat().st_mtime_ns
        if self.cached_path == path and self.cached_mtime_ns == mtime_ns and self.cached_xml:
            return

        xml_text = path.read_text(encoding="utf-8")
        if not xml_text.strip():
            self.get_logger().warn(f"BT XML file is empty: {self.bt_xml_path}")
            return

        self.cached_path = path
        self.cached_mtime_ns = mtime_ns
        self.cached_xml = xml_text
        self.get_logger().info(
            f"Loaded BT XML ({len(xml_text)} bytes) from {self.bt_xml_path}"
        )

    def _publish_xml(self) -> None:
        if not self.cached_xml:
            return
        msg = String()
        msg.data = self.cached_xml
        self.topology_publisher.publish(msg)

    def _publish_state_snapshot(self) -> None:
        payload = {
            "updated_at_unix": self.last_state_update_unix,
            "node_states": self.node_states,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.state_publisher.publish(msg)

    def _on_bt_log(self, msg: BehaviorTreeLog) -> None:
        changed = False
        for event in msg.event_log:
            node_name = event.node_name.strip()
            if not node_name:
                continue
            status = event.current_status.strip() or "UNKNOWN"
            if self.node_states.get(node_name) != status:
                self.node_states[node_name] = status
                changed = True

        if changed:
            self.last_state_update_unix = time.time()
            self._publish_state_snapshot()

    def _tick(self) -> None:
        self._refresh_xml_cache()
        self._publish_xml()
        self._publish_state_snapshot()


def main() -> None:
    rclpy.init()
    node = BTTopologyPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
