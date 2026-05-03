#!/usr/bin/env python3
"""
Joint State Merger - Combines joint states from multiple sources

Subscribes to joint_state_broadcaster (4 controlled joints) and
passive_joint_publisher (4 passive joints) and merges them into
a single /joint_states topic for robot_state_publisher.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState

class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')

        # QoS for subscribing (match publishers)
        qos_sub = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # QoS for publishing (TRANSIENT_LOCAL for robot_state_publisher)
        qos_pub = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Subscribe to both sources
        # joint_state_broadcaster /joint_states is remapped to /joint_states_controlled in docker-compose
        self.controlled_sub = self.create_subscription(
            JointState, '/joint_states_controlled', self.controlled_callback, 10)
        self.passive_sub = self.create_subscription(
            JointState, '/passive_joint_states', self.passive_callback, 10)

        # Publish merged states
        self.pub = self.create_publisher(JointState, '/joint_states', qos_pub)

        self.controlled_states = None
        self.passive_states = None
        self.timer = self.create_timer(0.1, self.publish_merged)  # 10Hz (matches hardware_bridge input rate)

        self.get_logger().info('Joint State Merger started')

    def controlled_callback(self, msg):
        self.controlled_states = msg

    def passive_callback(self, msg):
        self.passive_states = msg

    def publish_merged(self):
        if self.controlled_states is None:
            return

        msg = JointState()
        msg.header.stamp = self.controlled_states.header.stamp
        msg.header.frame_id = 'base_link'

        # Start with controlled joints
        msg.name = list(self.controlled_states.name)
        msg.position = list(self.controlled_states.position)
        msg.velocity = list(self.controlled_states.velocity)
        msg.effort = list(self.controlled_states.effort) if self.controlled_states.effort else []

        # Add passive joints if available
        if self.passive_states:
            msg.name.extend(self.passive_states.name)
            msg.position.extend(self.passive_states.position)
            msg.velocity.extend(self.passive_states.velocity)
            if self.passive_states.effort:
                msg.effort.extend(self.passive_states.effort)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
