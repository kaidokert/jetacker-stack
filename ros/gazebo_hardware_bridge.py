#!/usr/bin/env python3
"""
Gazebo Hardware Bridge - Simulation equivalent of STM32 serial bridge

Interfaces between ros2_control's JointStateTopicSystem and Gazebo simulation.
This mimics the real robot's hardware bridge that talks to STM32 over serial.

Architecture:
  /robot_joint_commands (ROS2) → This bridge → ROS topics → ros_gz_bridge → Gazebo
  Gazebo → ros_gz_bridge → ROS topics → This bridge → /robot_joint_states (ROS2)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from steering_filter import SteeringFilter


class GazeboHardwareBridge(Node):
    """Bridge between ROS2 topic-based hardware interface and Gazebo"""

    def __init__(self):
        super().__init__('gazebo_hardware_bridge')

        # Declare steering angle limit parameter (matches URDF joint limit)
        # Real robot: 33° (0.576 rad) due to mechanical constraints
        # Simulation: 28° (0.4887 rad) from URDF definition
        self.declare_parameter('max_steering_angle_rad', 0.4887)
        self.declare_parameter('vx_steer_threshold', 0.05)
        self.declare_parameter('wheel_radius', 0.050)
        # Clutch: True = engaged (power flows, default). False = disengaged (robot stops).
        self.declare_parameter('clutch', True)

        # Speed-scaled steering rate limiter parameters
        # max_rate = min(rate_max, rate_base + rate_gain * |vx|)
        # Damps atan(wz/vx) noise in the low-speed regime where the alpha-blend
        # guard is transparent but atan sensitivity is still high.
        self.declare_parameter('steering_rate_base', 0.5)           # rad/s at vx=0
        self.declare_parameter('steering_rate_gain_per_mps', 10.0)  # (rad/s)/(m/s)
        self.declare_parameter('steering_rate_max', 6.0)            # rad/s absolute cap

        # Shared steering conditioning filter (same class used by real robot hardware node)
        self._steering_filter = SteeringFilter(
            vx_threshold=self.get_parameter('vx_steer_threshold').value,
            max_angle=self.get_parameter('max_steering_angle_rad').value,
            rate_base=self.get_parameter('steering_rate_base').value,
            rate_gain=self.get_parameter('steering_rate_gain_per_mps').value,
            rate_max=self.get_parameter('steering_rate_max').value,
        )
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f'Steering angle limit: ±{self.get_parameter("max_steering_angle_rad").value:.4f} rad '
            f'(±{self.get_parameter("max_steering_angle_rad").value * 57.3:.1f}°)'
        )

        # ROS2 interface - matches real robot's hardware driver
        self.command_sub = self.create_subscription(
            JointState,
            '/robot_joint_commands',
            self.command_callback,
            10)

        # Use BEST_EFFORT QoS to match JointStateTopicSystem subscriber
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state_pub = self.create_publisher(
            JointState,
            '/robot_joint_states',
            qos_profile)

        # Create persistent publishers for Gazebo joint commands (via ros_gz_bridge)
        # Position command publishers
        self.pos_pubs = {}
        for joint in ['turret_joint', 'front_left_wheel_steering_joint', 'front_right_wheel_steering_joint']:
            self.pos_pubs[joint] = self.create_publisher(
                Float64,
                f'/gz_cmd/{joint}/position',
                10
            )

        # Velocity command publishers
        self.vel_pubs = {}
        for joint in ['rear_left_wheel_joint', 'rear_right_wheel_joint']:
            self.vel_pubs[joint] = self.create_publisher(
                Float64,
                f'/gz_cmd/{joint}/velocity',
                10
            )

        # Subscribe to actual Gazebo joint states (NEW - replaces open-loop integration)
        self.gz_joint_state_sub = self.create_subscription(
            JointState,
            '/gz_joint_states',
            self.gz_joint_state_callback,
            10)
        self.actual_gz_joint_states = None

        # Publish joint states at 10Hz (reduced from 100Hz for Gazebo stability)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Joint name mapping - ALL non-fixed joints for Foxglove/robot_state_publisher
        self.joints = [
            'front_steering_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
            'turret_joint',
            'front_left_wheel_steering_joint',
            'front_right_wheel_steering_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint'
        ]

        # DEPRECATED: Simulated joint positions (kept for backward compatibility during transition)
        # Now using actual_gz_joint_states instead
        self.joint_positions = {joint: 0.0 for joint in self.joints}
        self.joint_velocities = {joint: 0.0 for joint in self.joints}
        self.last_update_time = self.get_clock().now()
        self.publish_count = 0

        self.get_logger().info('Gazebo Hardware Bridge started')
        self.get_logger().info('  Commands: /robot_joint_commands → /gz_cmd/* → ros_gz_bridge → Gazebo')
        self.get_logger().info('  Feedback: Gazebo → /gz_joint_states → /robot_joint_states (ACTUAL state)')
        self.get_logger().info(f'  Position publishers: {len(self.pos_pubs)} joints')
        self.get_logger().info(f'  Velocity publishers: {len(self.vel_pubs)} joints')

    def _on_param_change(self, params):
        """Sync ROS parameter changes to the steering filter."""
        from rcl_interfaces.msg import SetParametersResult
        _param_map = {
            'vx_steer_threshold': 'vx_threshold',
            'max_steering_angle_rad': 'max_angle',
            'steering_rate_base': 'rate_base',
            'steering_rate_gain_per_mps': 'rate_gain',
            'steering_rate_max': 'rate_max',
        }
        for p in params:
            attr = _param_map.get(p.name)
            if attr is not None:
                setattr(self._steering_filter, attr, p.value)
                self.get_logger().info(f'Steering filter: {p.name} -> {p.value}')
        return SetParametersResult(successful=True)

    def command_callback(self, msg: JointState):
        """
        Receive commands from JointStateTopicSystem and apply to Gazebo.

        JointState message has sparse position/velocity arrays - they only contain
        values for joints that use that interface, in the order they appear in the URDF.
        """
        # Clutch disengaged — send zero velocities to stop wheels, skip real commands
        if not self.get_parameter('clutch').value:
            for pub in self.vel_pubs.values():
                zero = Float64()
                zero.data = 0.0
                pub.publish(zero)
            return

        self.get_logger().info(f'Received command: names={msg.name}, pos={msg.position}, vel={msg.velocity}', throttle_duration_sec=1.0)

        # Position-controlled joints (must match controller config order)
        position_joints = ['front_steering_joint', 'turret_joint']
        # Velocity-controlled joints (must match controller config order!)
        # Message order: ["rear_left_wheel_joint", "rear_right_wheel_joint"] (matches JointStateBroadcaster)
        velocity_joints = ['rear_left_wheel_joint', 'rear_right_wheel_joint']

        # Apply position commands
        for i, joint_name in enumerate(position_joints):
            if i < len(msg.position):
                pos_cmd = msg.position[i]

                # CLOSED-LOOP FIX: Apply steering angle limits BEFORE storing/commanding
                # This ensures odometry uses actual achievable values, not saturated commands
                # (Matches real robot's hardware bridge implementation)
                if joint_name == 'front_steering_joint':
                    original_cmd = pos_cmd

                    # Estimate forward speed from rear wheel velocities
                    wheel_r = self.get_parameter('wheel_radius').value
                    vx_approx = 0.0
                    if self.actual_gz_joint_states is not None:
                        gz_vel = self.actual_gz_joint_states.velocity
                        gz_names = self.actual_gz_joint_states.name
                        rl_idx = gz_names.index('rear_left_wheel_joint') if 'rear_left_wheel_joint' in gz_names else None
                        rr_idx = gz_names.index('rear_right_wheel_joint') if 'rear_right_wheel_joint' in gz_names else None
                        if rl_idx is not None and rr_idx is not None:
                            vx_approx = abs(gz_vel[rl_idx] + gz_vel[rr_idx]) / 2.0 * wheel_r

                    pos_cmd = self._steering_filter.update(pos_cmd, vx_approx)

                    # Log saturation events
                    if abs(original_cmd - pos_cmd) > 0.001:
                        max_angle = self._steering_filter.max_angle
                        self.get_logger().warn(
                            f'Steering conditioning: commanded {original_cmd * 57.3:.1f}° '
                            f'-> output {pos_cmd * 57.3:.1f}° (limit +-{max_angle * 57.3:.1f}°)',
                            throttle_duration_sec=1.0
                        )

                # Track position for feedback (always, even for ignored joints)
                self.joint_positions[joint_name] = pos_cmd
                self.joint_velocities[joint_name] = 0.0

                # ALWAYS send position commands (even 0.0 - it's a command, not noise!)
                # This ensures steering resets to 0 when controller commands straight drive
                self.get_logger().info(f'Setting position {joint_name}={pos_cmd}', throttle_duration_sec=1.0)

                # Special case: front_steering_joint has broken mimic in Gazebo
                # Command both wheel steering joints in parallel to minimize desync
                if joint_name == 'front_steering_joint':
                    self._set_joint_position_parallel([
                        ('front_left_wheel_steering_joint', pos_cmd),
                        ('front_right_wheel_steering_joint', pos_cmd)
                    ])
                    self.joint_positions['front_left_wheel_steering_joint'] = pos_cmd
                    self.joint_positions['front_right_wheel_steering_joint'] = pos_cmd
                else:
                    # For other joints, command directly
                    self._set_joint_position(joint_name, pos_cmd)

        # Apply velocity commands
        for i, joint_name in enumerate(velocity_joints):
            if i < len(msg.velocity):
                vel_cmd = msg.velocity[i]
                # Always send velocity commands (even 0.0 to stop wheels)
                self.get_logger().info(f'Setting velocity {joint_name}={vel_cmd}')
                self._set_joint_velocity(joint_name, vel_cmd)

                # Track velocity for integration (DEPRECATED - now use actual Gazebo state)
                self.joint_velocities[joint_name] = vel_cmd

    def gz_joint_state_callback(self, msg: JointState):
        """
        Receive actual joint states from Gazebo via WorldStatePublisher.

        This replaces the open-loop velocity integration with true closed-loop feedback.
        Stores the latest Gazebo state which will be republished to /robot_joint_states.
        """
        self.actual_gz_joint_states = msg

        # Build debug message safely
        debug_info = f'Received Gazebo joint states: {len(msg.name)} joints'
        if len(msg.position) > 5:
            debug_info += f', steering=[{msg.position[4]:.4f}, {msg.position[5]:.4f}]'

        self.get_logger().info(debug_info, throttle_duration_sec=2.0)

    def publish_joint_states(self):
        """
        Publish joint states using ACTUAL Gazebo data (closed-loop feedback).

        Replaces open-loop velocity integration with true Gazebo state from
        WorldStatePublisher plugin. This fixes the "Split Brain" bug where
        ROS and Gazebo had divergent joint states.
        """
        current_time = self.get_clock().now()

        # Check if we have actual Gazebo state
        if self.actual_gz_joint_states is None:
            self.get_logger().warn('No Gazebo joint states received yet', throttle_duration_sec=5.0)
            return

        # Republish actual Gazebo state to /robot_joint_states
        msg = JointState()
        msg.header.stamp = current_time.to_msg()
        msg.name = self.actual_gz_joint_states.name
        msg.position = self.actual_gz_joint_states.position
        msg.velocity = self.actual_gz_joint_states.velocity

        self.state_pub.publish(msg)
        self.publish_count += 1

        # Build debug message with steering joint info (joints 4&5 typically)
        steering_info = ""
        if len(msg.position) > 5:
            steering_info = f', steering=[{msg.position[4]:.4f}, {msg.position[5]:.4f}]'

        self.get_logger().info(
            f'Published #{self.publish_count} (ACTUAL Gazebo state): {len(msg.name)} joints{steering_info}',
            throttle_duration_sec=1.0
        )

    def _set_joint_position_parallel(self, joint_positions: list):
        """
        Set multiple joint positions simultaneously via ROS publishers.
        Used for steering joints that must move in perfect sync.

        Args:
            joint_positions: List of (joint_name, position) tuples
        """
        try:
            # Publish to all joints - ros_gz_bridge forwards to Gazebo
            # Since publishers are persistent and non-blocking, this is effectively simultaneous
            for joint_name, position in joint_positions:
                self._set_joint_position(joint_name, position)

            # Log first call only
            if not hasattr(self, '_gz_sync_counter'):
                self._gz_sync_counter = 0
            self._gz_sync_counter += 1
            if self._gz_sync_counter % 50 == 1:
                joint_names = [j[0] for j in joint_positions]
                self.get_logger().info(f'[ROS] sync pub #{self._gz_sync_counter}: {joint_names} = {joint_positions[0][1]:.4f}')
        except Exception as e:
            self.get_logger().error(f'Failed to set positions in parallel: {e}', throttle_duration_sec=5.0)

    def _set_joint_position(self, joint_name: str, position: float):
        """Set joint position via ROS topic (bridged to Gazebo)"""
        try:
            msg = Float64()
            msg.data = position
            self.pos_pubs[joint_name].publish(msg)

            # Log the command being sent (every 50 calls to avoid spam)
            if not hasattr(self, '_gz_call_counter'):
                self._gz_call_counter = {}
            self._gz_call_counter[joint_name] = self._gz_call_counter.get(joint_name, 0) + 1
            if self._gz_call_counter[joint_name] % 50 == 1:
                self.get_logger().info(f'[ROS] position pub #{self._gz_call_counter[joint_name]}: {joint_name} = {position:.4f}')
        except Exception as e:
            self.get_logger().error(f'Failed to publish position for {joint_name}: {e}', throttle_duration_sec=5.0)

    def _set_joint_velocity(self, joint_name: str, velocity: float):
        """Set joint velocity via ROS topic (bridged to Gazebo)"""
        try:
            msg = Float64()
            msg.data = velocity
            self.vel_pubs[joint_name].publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish velocity for {joint_name}: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    bridge = GazeboHardwareBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
