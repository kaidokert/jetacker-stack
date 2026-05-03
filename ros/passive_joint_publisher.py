#!/usr/bin/env python3
"""
Passive Joint State Publisher - Publishes states for uncontrolled joints

Reads front wheel joint states from Gazebo and publishes to /joint_states
so robot_state_publisher can compute their transforms for Foxglove.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState

class PassiveJointPublisher(Node):
    """Publishes states for passive/uncontrolled joints"""

    def __init__(self):
        super().__init__('passive_joint_publisher')

        # Match joint_state_broadcaster QoS: RELIABLE + TRANSIENT_LOCAL
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.create_publisher(JointState, '/passive_joint_states', qos)
        self.timer = self.create_timer(0.1, self.publish_states)  # 10Hz (constant zeros, no need for high freq)

        # Passive joints (not in ros2_control)
        self.passive_joints = [
            'front_left_wheel_steering_joint',
            'front_right_wheel_steering_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint'
        ]

        self.get_logger().info('Passive Joint Publisher started')
        self.get_logger().info(f'  Publishing: {self.passive_joints}')

    def publish_states(self):
        """Publish zero states for passive joints"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.passive_joints
        # Publish zeros - robot_state_publisher will use URDF defaults
        msg.position = [0.0] * len(self.passive_joints)
        msg.velocity = [0.0] * len(self.passive_joints)
        msg.effort = []

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PassiveJointPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
