#!/usr/bin/env python3
"""
Twist to TwistStamped relay for ros2_control controllers.

Subscribes to /cmd_vel (Twist) and publishes to /cmd_vel_stamped (TwistStamped).
The output topic is remapped per-robot in docker-compose.yml:
  - jetacker:  /cmd_vel_stamped -> /tricycle_steering_controller/reference
  - slam_bot:  /cmd_vel_stamped -> /diff_drive_controller/cmd_vel
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')

        # BEST_EFFORT for real-time control (Nav2, teleop)
        rt_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, rt_qos)

        self.publisher = self.create_publisher(
            TwistStamped, '/cmd_vel_stamped', rt_qos)

        self.get_logger().info('Twist->TwistStamped relay started')
        self._msg_count = 0
        self._last_log_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        self._msg_count += 1
        now = self.get_clock().now()

        if self._msg_count % 10 == 1 or (now - self._last_log_time).nanoseconds > 2e9:
            self.get_logger().info(
                f'Relaying cmd_vel: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f} '
                f'(count: {self._msg_count})')
            self._last_log_time = now

        stamped = TwistStamped()
        stamped.header.stamp = now.to_msg()
        stamped.header.frame_id = 'base_link'
        stamped.twist = msg
        self.publisher.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
