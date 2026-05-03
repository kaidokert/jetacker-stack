#!/usr/bin/env python3
"""
Nav2 Waypoint Follower - Sequential goal sender for Nav2 navigation

Sends waypoints sequentially to Nav2's /goal_pose topic and monitors
progress via /amcl_pose. Simple state machine without action client complexity.

State Machine:
    IDLE → SEND_GOAL → NAVIGATING → GOAL_REACHED → (next waypoint) → COMPLETE

Architecture:
- Publishes to /goal_pose (PoseStamped)
- Subscribes to /amcl_pose (PoseWithCovarianceStamped)
- Timer-based state machine (10 Hz)
- Timeout handling per waypoint
- Thread-safe using threading.Event

Usage:
    python3 nav2_waypoint_follower.py --waypoints gym_loop_nav2.yaml
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import JointState, LaserScan
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from rclpy.qos import qos_profile_sensor_data
from episode_metrics import EpisodeMetrics
import math
import os
import threading
import time


def _import_critics_stats():
    """Import CriticsStats message from patched MPPI overlay.

    Returns CriticsStats class or None if overlay not available.
    Uses same ctypes preload pattern as bag_recorder.py.
    """
    import sys
    import ctypes
    _critics_base = '/workspace/critics_install/nav2_critics_msgs'
    _pypath = f'{_critics_base}/lib/python3.12/site-packages'
    _libpath = f'{_critics_base}/lib'
    if os.path.isdir(_pypath):
        if _pypath not in sys.path:
            sys.path.insert(0, _pypath)
        for _so in sorted(os.listdir(_libpath)):
            if _so.endswith('.so'):
                try:
                    ctypes.cdll.LoadLibrary(os.path.join(_libpath, _so))
                except OSError:
                    pass
    try:
        from nav2_critics_msgs.msg import CriticsStats
        return CriticsStats
    except (ImportError, FileNotFoundError, OSError):
        return None


class Nav2WaypointFollower(Node):
    """
    ROS2 node to send sequential Nav2 goals and monitor completion.

    Thread-safety: Uses threading.Event for cross-thread communication
    between timer callback and subscription callback.
    """

    def __init__(self, waypoints, skip_init=False, use_amcl=True,
                 record=False, bag_dir=None, cycle_num=None, trust_nav2=False,
                 obstacles=None):
        super().__init__('nav2_waypoint_follower',
                         parameter_overrides=[
                             rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True),
                         ])

        self.waypoints = waypoints
        self.current_waypoint_idx = 0
        self.current_pose = None
        self.use_amcl = use_amcl  # Use AMCL localization or odometry-only
        self.trust_nav2 = trust_nav2  # Trust Nav2 STATUS_SUCCEEDED for goal detection

        # State machine states
        self.STATE_INITIALIZING = 'initializing'  # Waiting for localization to be ready
        self.STATE_IDLE = 'idle'
        self.STATE_CANCEL_STALE = 'cancel_stale'  # Cancel leftover goals from previous run
        self.STATE_SEND_GOAL = 'send_goal'
        self.STATE_NAVIGATING = 'navigating'
        self.STATE_GOAL_REACHED = 'goal_reached'
        self.STATE_COMPLETE = 'complete'
        self.STATE_FAILED = 'failed'

        self.state = self.STATE_INITIALIZING  # Start in INITIALIZING, not IDLE
        self._cancel_client = None  # ActionClient for canceling stale goals
        self._cancel_future = None  # Pending cancel future

        # Thread-safe flags (CRITICAL: use threading.Event for cross-thread visibility)
        self.goal_reached_event = threading.Event()
        self.nav2_succeeded = threading.Event()  # Set when bt_navigator reports STATUS_SUCCEEDED
        self.timeout_event = threading.Event()

        # Timing
        self.goal_start_time = None
        self.test_start_time = None

        # Obstacle scheduling — list of dicts with name/spawn_at/pose/shape/size
        self.obstacle_specs = list(obstacles) if obstacles else []
        self._obstacle_mgr = None
        self._pending_obstacles = []  # specs not yet spawned
        self._spawned_obstacle_names = []  # for results metadata

        # Results tracking
        self.waypoint_results = []
        self.success = False
        self.failure_reason = None
        self.collision_detected = False

        # QoS profiles
        # Goal topic: Must match bt_navigator's VOLATILE + BEST_EFFORT subscription
        goal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # RELIABLE publisher can serve BEST_EFFORT subscriber
            durability=DurabilityPolicy.VOLATILE,     # Must match subscriber's VOLATILE
            depth=10
        )

        # Initial pose: TRANSIENT_LOCAL so AMCL receives it even after late startup
        initial_pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Survives AMCL restarts
            depth=10
        )

        # AMCL pose: Must match AMCL's TRANSIENT_LOCAL publisher
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Changed from VOLATILE
            depth=10
        )

        # Odometry: Must match EKF's VOLATILE publisher
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,  # Match odometry/filtered publisher
            depth=10
        )

        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            goal_qos
        )

        # Initial pose publisher (for AMCL initialization)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            initial_pose_qos  # Use TRANSIENT_LOCAL for restart resilience
        )

        # Subscriptions (CRITICAL: Create in __init__, NOT ad-hoc)
        if self.use_amcl:
            # AMCL mode: Subscribe to /amcl_pose
            self.amcl_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                '/amcl_pose',
                self.amcl_callback,
                amcl_qos
            )
        else:
            # Odometry-only mode: Subscribe to /odometry/filtered
            from nav_msgs.msg import Odometry
            self.odom_sub = self.create_subscription(
                Odometry,
                '/odometry/filtered',
                self.odom_callback,
                odom_qos  # Use VOLATILE to match EKF publisher
            )

        # TF listener for checking map->odom->base_link chain
        # CRITICAL: Must create in __init__, NOT ad-hoc, needs time to accumulate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Episode metrics — captures MPPI cmd_vel jitter and steering oscillation
        self.episode_metrics = EpisodeMetrics()
        self._steering_joint_idx = None  # cached index into JointState.name
        self._rear_left_vel_idx = None
        self._rear_right_vel_idx = None

        self.joint_state_sub = self.create_subscription(
            JointState, '/robot_joint_states', self.joint_state_callback,
            qos_profile_sensor_data)
        self._steering_cmd_joint_idx = None  # cached index for /robot_joint_commands
        self.joint_command_sub = self.create_subscription(
            JointState, '/robot_joint_commands', self.joint_command_callback,
            qos_profile_sensor_data)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Nav2 action status — detect when bt_navigator aborts (planner failure, recovery exhausted)
        self.nav2_aborted = threading.Event()
        nav2_status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Must match publisher (bt_navigator)
            depth=10
        )
        self.nav2_status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.nav2_status_callback,
            nav2_status_qos
        )

        # Collision detection via lidar scan — if any point < threshold, flag collision
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self._collision_range_threshold = 0.12  # meters (lidar range_min = 0.12m, saturates at contact)

        # PathFollowCritic live accumulator (from patched MPPI critics overlay)
        self._pfc_sum = 0.0
        self._pfc_max = 0.0
        self._pfc_count = 0
        self._pfc_critic_idx = None  # cached index into critics[] array
        self._pfc_available = False
        CriticsStats = _import_critics_stats()
        if CriticsStats is not None:
            self._critics_stats_sub = self.create_subscription(
                CriticsStats,
                '/controller_server/critics_stats',
                self.critics_stats_callback,
                QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    depth=10,
                ),
            )
            self._pfc_available = True
            self.get_logger().info('[PFC] CriticsStats subscription created')
        else:
            self.get_logger().warn('[PFC] nav2_critics_msgs not available — PFC metrics disabled')

        # In-process rosbag recording (no subprocess, no zombie risk)
        self.bag_recorder = None
        if record and bag_dir:
            from bag_recorder import BagRecorder, _nav2_record_topics, make_bag_path
            bag_path = make_bag_path(bag_dir, cycle_num, prefix='nav2')
            self.bag_recorder = BagRecorder(self, bag_path, _nav2_record_topics())

        # Obstacle manager — only initialize if test specifies obstacles
        if self.obstacle_specs:
            try:
                from gazebo_obstacles import ObstacleManager
                self._obstacle_mgr = ObstacleManager(self)
                if self._obstacle_mgr.wait_ready(timeout_sec=5.0):
                    self.get_logger().info(
                        f'[obstacles] Manager ready, {len(self.obstacle_specs)} obstacle(s) scheduled')
                    # Pre-clean any leftover obstacles by name from a previous run
                    for spec in self.obstacle_specs:
                        self._obstacle_mgr.delete(spec['name'])
                    self._pending_obstacles = list(self.obstacle_specs)
                else:
                    self.get_logger().error('[obstacles] Bridge services unavailable, disabling obstacle drops')
                    self._obstacle_mgr = None
            except Exception as e:
                self.get_logger().error(f'[obstacles] Failed to init manager: {e}')
                self._obstacle_mgr = None

        # State machine timer (10 Hz)
        self.timer = self.create_timer(0.1, self.state_machine_callback)

        self.get_logger().info(f'Nav2 Waypoint Follower initialized')
        self.get_logger().info(f'  Waypoints: {len(self.waypoints)}')
        self.get_logger().info(f'  Publishing goals to: /goal_pose')
        pose_topic = '/amcl_pose' if self.use_amcl else '/odometry/filtered'
        self.get_logger().info(f'  Monitoring pose: {pose_topic}')

        # Set initial pose for AMCL (robot starts at origin facing south)
        if not skip_init:
            self.get_logger().info('[INIT] Setting initial pose...')
            self.set_initial_pose()
            # AMCL is ready - start state machine
            self.state = self.STATE_IDLE
            self.get_logger().info('[INIT] Initialization complete, starting navigation')
        else:
            self.get_logger().info('[INIT] Skipping initial pose (--skip-init)')
            # Still wait a bit for AMCL to be ready
            time.sleep(1.0)
            self.state = self.STATE_IDLE

    def amcl_callback(self, msg):
        """Update current pose from AMCL."""
        self.current_pose = msg.pose.pose

        # Log first AMCL pose received
        if not hasattr(self, '_first_amcl_received'):
            self._first_amcl_received = True
            self.get_logger().info(
                f'[AMCL] First pose received: '
                f'({msg.pose.pose.position.x:.3f}, {msg.pose.pose.position.y:.3f})'
            )

        # Check if goal reached (only when navigating)
        if self.state == self.STATE_NAVIGATING:
            self.check_goal_reached()

    def odom_callback(self, msg):
        """Update current pose from odometry (when not using AMCL)."""
        self.current_pose = msg.pose.pose

        # Log first odometry pose received
        if not hasattr(self, '_first_odom_received'):
            self._first_odom_received = True
            self.get_logger().info(
                f'[ODOM] First pose received: '
                f'({msg.pose.pose.position.x:.3f}, {msg.pose.pose.position.y:.3f})'
            )

        # Check if goal reached (only when navigating)
        if self.state == self.STATE_NAVIGATING:
            self.check_goal_reached()

    def joint_state_callback(self, msg):
        """Record steering angle and rear wheel velocities for jitter metrics (from hardware bridge)."""
        if self.state != self.STATE_NAVIGATING:
            return
        if self._steering_joint_idx is None:
            names = list(msg.name)
            try:
                self._steering_joint_idx = names.index('front_left_wheel_steering_joint')
            except ValueError:
                return
            try:
                self._rear_left_vel_idx = names.index('rear_left_wheel_joint')
                self._rear_right_vel_idx = names.index('rear_right_wheel_joint')
            except ValueError:
                pass  # wheel vel tracking optional
        if self._steering_joint_idx < len(msg.position):
            left_vel = msg.velocity[self._rear_left_vel_idx] if (
                self._rear_left_vel_idx is not None and self._rear_left_vel_idx < len(msg.velocity)) else 0.0
            right_vel = msg.velocity[self._rear_right_vel_idx] if (
                self._rear_right_vel_idx is not None and self._rear_right_vel_idx < len(msg.velocity)) else 0.0
            self.episode_metrics.record_joint_state(
                time.time(), msg.position[self._steering_joint_idx],
                left_vel, right_vel)

    def joint_command_callback(self, msg):
        """Record commanded steering angle from hardware bridge (raw singularity signal)."""
        if self.state != self.STATE_NAVIGATING:
            return
        if self._steering_cmd_joint_idx is None:
            names = list(msg.name)
            try:
                self._steering_cmd_joint_idx = names.index('front_steering_joint')
            except ValueError:
                return
        if self._steering_cmd_joint_idx < len(msg.position):
            self.episode_metrics.record_steering_command(
                time.time(), msg.position[self._steering_cmd_joint_idx])

    def cmd_vel_callback(self, msg):
        """Record cmd_vel for smoothness metrics (captures MPPI output)."""
        if self.state != self.STATE_NAVIGATING:
            return
        self.episode_metrics.record_cmd_vel(
            time.time(), msg.linear.x, msg.angular.z)

    def nav2_status_callback(self, msg):
        """Detect when bt_navigator reports goal completion or abort.

        Only reacts to the most recent goal whose timestamp is >= when we sent
        our goal.  Previous entries (stale ABORTs from earlier trials) are ignored.
        """
        if not msg.status_list:
            return
        if not hasattr(self, '_goal_send_stamp') or self._goal_send_stamp is None:
            return
        # Only check the last (most recent) status entry
        status = msg.status_list[-1]
        # Ignore stale goals from before our current navigation
        goal_ns = status.goal_info.stamp.sec * 1_000_000_000 + status.goal_info.stamp.nanosec
        send_ns = self._goal_send_stamp.nanoseconds
        if goal_ns < send_ns:
            return
        if status.status == 4:  # STATUS_SUCCEEDED
            self.nav2_succeeded.set()
            elapsed = time.time() - self.goal_start_time if self.goal_start_time else 0
            self.get_logger().info(
                f'[NAV2] Goal SUCCEEDED by bt_navigator at t={elapsed:.1f}s '
                f'(state={self.state})')
        elif status.status == 6:  # STATUS_ABORTED
            if self.state == self.STATE_NAVIGATING:
                self.nav2_aborted.set()
            self.get_logger().warn('[NAV2] Goal ABORTED by bt_navigator')
        elif status.status == 5:  # STATUS_CANCELED
            if self.state == self.STATE_NAVIGATING:
                self.nav2_aborted.set()
            self.get_logger().warn('[NAV2] Goal CANCELED by bt_navigator')

    def scan_callback(self, msg):
        """Check for collision via minimum scan range.

        Skipped entirely when the test scenario defines obstacle drops —
        in those cases, lidar saturation at range_min is expected behavior
        (bot is correctly stopped near the dropped obstacle), not a real
        collision. Use ground truth pose checks instead for those tests.
        """
        if self.state != self.STATE_NAVIGATING or self.collision_detected:
            return
        if self.obstacle_specs:
            return  # Disabled for obstacle-drop tests — see docstring
        # Filter out inf/nan, check if any valid range is below threshold
        min_range = float('inf')
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max and r < min_range:
                min_range = r
        if min_range < self._collision_range_threshold:
            self.collision_detected = True
            self.get_logger().error(
                f'[COLLISION] Detected! min_range={min_range:.3f}m < {self._collision_range_threshold}m')

    def critics_stats_callback(self, msg):
        """Accumulate PathFollowCritic cost from live MPPI critic stats."""
        if self.state != self.STATE_NAVIGATING:
            return
        # Find PFC index on first message (critic list is stable per run)
        if self._pfc_critic_idx is None:
            try:
                self._pfc_critic_idx = list(msg.critics).index('PathFollowCritic')
            except ValueError:
                self.get_logger().warn(
                    f'[PFC] PathFollowCritic not in critics list: {list(msg.critics)}')
                return
        idx = self._pfc_critic_idx
        if idx >= len(msg.costs_best):
            return
        cost = msg.costs_best[idx]
        self._pfc_sum += cost
        self._pfc_count += 1
        if cost > self._pfc_max:
            self._pfc_max = cost

    def _reset_pfc(self):
        """Reset PFC accumulator for a new waypoint."""
        self._pfc_sum = 0.0
        self._pfc_max = 0.0
        self._pfc_count = 0

    def _get_pfc_metrics(self):
        """Return PFC metrics dict. Returns empty dict if no data."""
        if self._pfc_count == 0:
            return {}
        return {
            'pfc_mean': self._pfc_sum / self._pfc_count,
            'pfc_max': self._pfc_max,
            'pfc_integral': self._pfc_sum,
            'pfc_count': self._pfc_count,
        }

    def check_goal_reached(self):
        """
        Check if current waypoint reached within tolerance.

        When trust_nav2=False (default): sets goal_reached_event if within tolerance.
        When trust_nav2=True: computes errors for logging/scoring only — goal detection
        is delegated to Nav2's STATUS_SUCCEEDED via nav2_succeeded event.
        """
        if self.current_pose is None:
            return

        wp = self.waypoints[self.current_waypoint_idx]

        # Position error
        dx = self.current_pose.position.x - wp['x']
        dy = self.current_pose.position.y - wp['y']
        position_error = math.sqrt(dx**2 + dy**2)

        # Heading error
        current_heading = self.quaternion_to_yaw(self.current_pose.orientation)
        target_heading = math.radians(wp['heading_deg'])
        heading_error = abs(self.normalize_angle(current_heading - target_heading))

        if self.trust_nav2:
            # Measurement only — Nav2 decides when goal is reached
            return

        # Check tolerances
        if (position_error <= wp['tolerance_xy'] and
            heading_error <= wp['tolerance_yaw']):

            # Goal reached!
            self.goal_reached_event.set()

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle (radians)."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def set_initial_pose(self):
        """
        Set initial pose for AMCL localization.

        Robot starts at origin (0, 0) facing east (0 radians) according to SDF.
        """
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.frame_id = 'map'
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()

        # Position: origin
        initial_pose_msg.pose.pose.position.x = 0.0
        initial_pose_msg.pose.pose.position.y = 0.0
        initial_pose_msg.pose.pose.position.z = 0.0

        # Orientation: facing east (0 radians)
        # Quaternion for yaw=0: (0, 0, 0, 1)
        initial_pose_msg.pose.pose.orientation.x = 0.0
        initial_pose_msg.pose.pose.orientation.y = 0.0
        initial_pose_msg.pose.pose.orientation.z = 0.0
        initial_pose_msg.pose.pose.orientation.w = 1.0

        # Covariance: small uncertainty (0.1m position, 0.1rad orientation)
        initial_pose_msg.pose.covariance = [0.0] * 36
        initial_pose_msg.pose.covariance[0] = 0.01  # x variance
        initial_pose_msg.pose.covariance[7] = 0.01  # y variance
        initial_pose_msg.pose.covariance[35] = 0.01  # yaw variance

        # Publish initial pose (multiple times for reliability)
        for i in range(5):
            self.initial_pose_pub.publish(initial_pose_msg)
            time.sleep(0.1)

        self.get_logger().info('[INIT] Initial pose set: (0.0, 0.0, 0.0°)')

        # Give AMCL time to process initial pose and start publishing TF
        # AMCL needs time to: receive initial pose → initialize particle filter → start publishing map->odom TF
        time.sleep(2.0)

        if self.use_amcl:
            self.get_logger().info('[INIT] Waiting for AMCL to initialize...')
            # Wait for AMCL readiness with active polling
            if not self.wait_for_amcl_ready(timeout=10.0):
                self.get_logger().error('[INIT] AMCL readiness check FAILED')
                raise RuntimeError('AMCL failed to initialize after initial pose')
        else:
            self.get_logger().info('[INIT] Waiting for odometry to be ready...')
            # Wait for odometry readiness
            if not self.wait_for_odom_ready(timeout=10.0):
                self.get_logger().error('[INIT] Odometry readiness check FAILED')
                raise RuntimeError('Odometry failed to initialize')

        # Wait for Nav2 action server to be ready (replaces hardcoded 3s delay)
        # This ensures bt_navigator has completed activation and is ready to accept goals
        self.get_logger().info('[INIT] Waiting for Nav2 action server to be ready...')
        if not self.wait_for_nav2_ready(timeout=120.0):
            self.get_logger().error('[INIT] Nav2 action server never became available')
            raise RuntimeError('Nav2 failed to become ready after stack restart')

        self.get_logger().info('[INIT] Nav2 stack ready, settling for 2s...')
        time.sleep(2.0)  # Brief settling time after Nav2 confirms ready

    def wait_for_amcl_ready(self, timeout=10.0):
        """
        Wait for AMCL to be ready after initialization.

        Checks:
        1. /amcl_pose is being published

        TF chain check removed - TF buffer accumulation is unreliable immediately after node
        startup. Nav2 will handle TF availability internally.

        Args:
            timeout: Maximum wait time in seconds

        Returns:
            True if ready, False if timeout
        """
        start_time = time.time()
        check_interval = 0.5  # Check every 500ms

        amcl_ready = False

        while (time.time() - start_time) < timeout:
            # Spin once to process callbacks (CRITICAL: allows AMCL subscription to update self.current_pose)
            rclpy.spin_once(self, timeout_sec=check_interval)

            # Check: AMCL pose received
            if self.current_pose is not None:
                if not amcl_ready:  # Only log once
                    self.get_logger().info('[INIT] AMCL pose received ✓')
                    elapsed = time.time() - start_time
                    self.get_logger().info(f'[INIT] AMCL ready after {elapsed:.1f}s')
                amcl_ready = True
                return True

        # Timeout
        self.get_logger().error('[INIT] AMCL pose never received')
        return False

    def wait_for_odom_ready(self, timeout=10.0):
        """
        Wait for odometry to be ready (when not using AMCL).

        Checks:
        1. /odometry/filtered is being published

        Args:
            timeout: Maximum wait time in seconds

        Returns:
            True if ready, False if timeout
        """
        start_time = time.time()
        check_interval = 0.5  # Check every 500ms

        odom_ready = False

        while (time.time() - start_time) < timeout:
            # Spin once to process callbacks
            rclpy.spin_once(self, timeout_sec=check_interval)

            # Check: Odometry pose received
            if self.current_pose is not None:
                if not odom_ready:  # Only log once
                    self.get_logger().info('[INIT] Odometry pose received ✓')
                    elapsed = time.time() - start_time
                    self.get_logger().info(f'[INIT] Odometry ready after {elapsed:.1f}s')
                odom_ready = True
                return True

        # Timeout
        self.get_logger().error('[INIT] Odometry pose never received')
        return False

    def wait_for_nav2_ready(self, timeout=120.0):
        """
        Wait for Nav2 bt_navigator action server to be ready.

        Creates a temporary action client to check if /navigate_to_pose action server
        is available. This ensures bt_navigator has completed lifecycle activation
        and is ready to accept goals via /goal_pose topic.

        Args:
            timeout: Maximum wait time in seconds (default 120s for stack restart)

        Returns:
            True if Nav2 ready, False if timeout
        """
        # Create temporary action client to check server availability
        temp_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('[INIT] Checking for /navigate_to_pose action server...')
        start_time = time.time()

        # Wait for action server to become available
        # This blocks but spins the node internally
        server_available = temp_action_client.wait_for_server(timeout_sec=timeout)

        if server_available:
            elapsed = time.time() - start_time
            self.get_logger().info(f'[INIT] Nav2 action server ready after {elapsed:.1f}s ✓')
            temp_action_client.destroy()

            # Create cancel service client for STATE_CANCEL_STALE
            # (can't spin futures in __init__). Uses action cancel service directly.
            self._cancel_client = self.create_client(
                CancelGoal, '/navigate_to_pose/_action/cancel_goal')
            return True
        else:
            self.get_logger().error('[INIT] Nav2 action server not available after timeout')
            temp_action_client.destroy()
            return False

    def send_goal(self):
        """Publish current waypoint as Nav2 goal."""
        wp = self.waypoints[self.current_waypoint_idx]

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'  # CRITICAL: Must be 'map', not 'odom'
        goal_msg.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.position.x = wp['x']
        goal_msg.pose.position.y = wp['y']
        goal_msg.pose.position.z = 0.0

        goal_msg.pose.orientation.x = wp['qx']
        goal_msg.pose.orientation.y = wp['qy']
        goal_msg.pose.orientation.z = wp['qz']
        goal_msg.pose.orientation.w = wp['qw']

        self.goal_pub.publish(goal_msg)
        self._reset_pfc()

        self.get_logger().info(
            f'[WP {self.current_waypoint_idx + 1}/{len(self.waypoints)}] '
            f'Goal sent: {wp["name"]} '
            f'({wp["x"]:.3f}, {wp["y"]:.3f}, {wp["heading_deg"]:.1f}°)'
        )

        self.goal_start_time = time.time()
        self._goal_send_stamp = self.get_clock().now()  # ROS sim time for status filtering
        # Re-arm obstacle schedule for this new goal — spawn_at is relative to goal_start_time
        if self._obstacle_mgr is not None:
            self._pending_obstacles = list(self.obstacle_specs)
        self.get_logger().info(
            f'[WP] goal_send_stamp={self._goal_send_stamp.nanoseconds / 1e9:.3f}s (sim time)')
        self.goal_reached_event.clear()
        self.nav2_succeeded.clear()
        self.nav2_aborted.clear()
        self.timeout_event.clear()

    def _tick_obstacle_schedule(self):
        """Spawn any obstacles whose spawn_at time has elapsed since goal start."""
        if self._obstacle_mgr is None or not self._pending_obstacles:
            return
        if self.goal_start_time is None:
            return
        elapsed = time.time() - self.goal_start_time
        # Spawn all due obstacles in order
        still_pending = []
        for spec in self._pending_obstacles:
            if elapsed >= float(spec.get('spawn_at', 0.0)):
                self.get_logger().warn(
                    f'[obstacles] Spawning "{spec["name"]}" at t={elapsed:.2f}s')
                if self._obstacle_mgr.spawn(spec):
                    self._spawned_obstacle_names.append(spec['name'])
            else:
                still_pending.append(spec)
        self._pending_obstacles = still_pending

    def cleanup_obstacles(self):
        """Remove all obstacles spawned during this run."""
        if self._obstacle_mgr is not None:
            self._obstacle_mgr.cleanup_all()

    def state_machine_callback(self):
        """
        State machine timer callback (10 Hz).

        States: INITIALIZING (wait) → IDLE → SEND_GOAL → NAVIGATING → GOAL_REACHED → (next) → COMPLETE
        """

        if self.state == self.STATE_INITIALIZING:
            # Waiting for AMCL initialization to complete
            # State will transition to IDLE when set_initial_pose() finishes
            return

        elif self.state == self.STATE_IDLE:
            # Initialize and start test
            self.test_start_time = time.time()
            self.episode_metrics.reset()
            self.episode_metrics.begin(self.test_start_time)
            # Cancel stale goals before sending new ones
            if self._cancel_client is not None:
                req = CancelGoal.Request()  # zero UUID + zero stamp = cancel all
                self._cancel_future = self._cancel_client.call_async(req)
                self.state = self.STATE_CANCEL_STALE
            else:
                self.state = self.STATE_SEND_GOAL

        elif self.state == self.STATE_CANCEL_STALE:
            # Wait for cancel to complete (future resolves via spin)
            if self._cancel_future is not None and self._cancel_future.done():
                result = self._cancel_future.result()
                n = len(result.goals_canceling) if result else 0
                if n > 0:
                    self.get_logger().info(f'[INIT] Canceled {n} stale goal(s), settling...')
                    # Defer to next tick to let bt_navigator clean up
                    self._cancel_settle_until = time.time() + 1.0
                    self._cancel_future = None
                    return
                self.get_logger().info('[INIT] No stale goals to cancel')
                self._cancel_future = None
                self.state = self.STATE_SEND_GOAL
            elif hasattr(self, '_cancel_settle_until'):
                # Settling after cancel
                if time.time() >= self._cancel_settle_until:
                    del self._cancel_settle_until
                    self.state = self.STATE_SEND_GOAL

        elif self.state == self.STATE_SEND_GOAL:
            # Send current waypoint goal
            self.send_goal()
            self.state = self.STATE_NAVIGATING

        elif self.state == self.STATE_NAVIGATING:
            # Drop scheduled obstacles whose spawn_at time has elapsed
            self._tick_obstacle_schedule()
            # Monitor for collision, goal reached, or timeout
            if self.collision_detected:
                wp = self.waypoints[self.current_waypoint_idx]
                elapsed = time.time() - self.goal_start_time
                error_xy = None
                error_yaw = None
                if self.current_pose:
                    dx = self.current_pose.position.x - wp['x']
                    dy = self.current_pose.position.y - wp['y']
                    error_xy = math.sqrt(dx**2 + dy**2)
                    current_heading = self.quaternion_to_yaw(self.current_pose.orientation)
                    target_heading = math.radians(wp['heading_deg'])
                    error_yaw = abs(self.normalize_angle(current_heading - target_heading))
                self.get_logger().error(
                    f'[WP {self.current_waypoint_idx + 1}] COLLISION at t={elapsed:.1f}s')
                self.waypoint_results.append({
                    'name': wp['name'],
                    'success': False,
                    'duration': elapsed,
                    'error_xy': error_xy,
                    'error_yaw': error_yaw,
                    'reason': 'collision'
                })
                self.state = self.STATE_FAILED
                self.failure_reason = f'Collision detected at waypoint {self.current_waypoint_idx + 1}'

            elif self.goal_reached_event.is_set() or (self.trust_nav2 and self.nav2_succeeded.is_set()):
                # Goal reached!
                self.state = self.STATE_GOAL_REACHED

            elif self.nav2_aborted.is_set():
                # bt_navigator gave up (planner failure, recovery exhausted)
                wp = self.waypoints[self.current_waypoint_idx]
                elapsed = time.time() - self.goal_start_time
                error_xy = None
                error_yaw = None
                if self.current_pose:
                    dx = self.current_pose.position.x - wp['x']
                    dy = self.current_pose.position.y - wp['y']
                    error_xy = math.sqrt(dx**2 + dy**2)
                    current_heading = self.quaternion_to_yaw(self.current_pose.orientation)
                    target_heading = math.radians(wp['heading_deg'])
                    error_yaw = abs(self.normalize_angle(current_heading - target_heading))
                self.get_logger().error(
                    f'[WP {self.current_waypoint_idx + 1}] Nav2 ABORTED at t={elapsed:.1f}s'
                    f' - dist={error_xy:.2f}m' if error_xy else '')
                self.waypoint_results.append({
                    'name': wp['name'],
                    'success': False,
                    'duration': elapsed,
                    'error_xy': error_xy,
                    'error_yaw': error_yaw,
                    'reason': 'nav2_aborted'
                })
                self.state = self.STATE_FAILED
                self.failure_reason = f'Nav2 aborted at waypoint {self.current_waypoint_idx + 1}'

            else:
                # Check timeout
                wp = self.waypoints[self.current_waypoint_idx]
                elapsed = time.time() - self.goal_start_time

                # Periodic status logging (every 5 seconds)
                if not hasattr(self, '_last_status_log'):
                    self._last_status_log = 0.0

                if elapsed - self._last_status_log >= 5.0:
                    self._last_status_log = elapsed
                    if self.current_pose:
                        dx = self.current_pose.position.x - wp['x']
                        dy = self.current_pose.position.y - wp['y']
                        dist = math.sqrt(dx**2 + dy**2)

                        # Calculate heading error for debugging
                        current_heading = self.quaternion_to_yaw(self.current_pose.orientation)
                        target_heading = math.radians(wp['heading_deg'])
                        heading_error = abs(self.normalize_angle(current_heading - target_heading))
                        heading_error_deg = math.degrees(heading_error)

                        self.get_logger().info(
                            f'[WP {self.current_waypoint_idx + 1}] '
                            f't={elapsed:.1f}s, current=({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}), '
                            f'target=({wp["x"]:.2f}, {wp["y"]:.2f}), dist={dist:.2f}m, '
                            f'heading_err={heading_error_deg:.1f}° (tol={math.degrees(wp["tolerance_yaw"]):.1f}°)'
                        )
                    else:
                        self.get_logger().warn(
                            f'[WP {self.current_waypoint_idx + 1}] t={elapsed:.1f}s, NO AMCL POSE YET!'
                        )

                if elapsed > wp['timeout']:
                    # Timeout — compute final errors for graduated scoring
                    error_xy = None
                    error_yaw = None
                    timeout_msg = f'[WP {self.current_waypoint_idx + 1}] TIMEOUT after {elapsed:.1f}s (limit: {wp["timeout"]:.1f}s)'
                    if self.current_pose:
                        dx = self.current_pose.position.x - wp['x']
                        dy = self.current_pose.position.y - wp['y']
                        error_xy = math.sqrt(dx**2 + dy**2)
                        current_heading = self.quaternion_to_yaw(self.current_pose.orientation)
                        target_heading = math.radians(wp['heading_deg'])
                        error_yaw = abs(self.normalize_angle(current_heading - target_heading))
                        timeout_msg += f' - final distance: {error_xy:.2f}m, yaw error: {math.degrees(error_yaw):.1f}°'
                    else:
                        timeout_msg += ' - NO AMCL POSE RECEIVED'

                    self.get_logger().error(timeout_msg)

                    self.waypoint_results.append({
                        'name': wp['name'],
                        'success': False,
                        'duration': elapsed,
                        'error_xy': error_xy,
                        'error_yaw': error_yaw,
                        'reason': 'timeout'
                    })

                    self.state = self.STATE_FAILED
                    self.failure_reason = f'Waypoint {self.current_waypoint_idx + 1} timeout'

        elif self.state == self.STATE_GOAL_REACHED:
            # Record waypoint completion
            wp = self.waypoints[self.current_waypoint_idx]
            duration = time.time() - self.goal_start_time

            # Calculate final errors
            if self.current_pose:
                dx = self.current_pose.position.x - wp['x']
                dy = self.current_pose.position.y - wp['y']
                error_xy = math.sqrt(dx**2 + dy**2)

                current_heading = self.quaternion_to_yaw(self.current_pose.orientation)
                target_heading = math.radians(wp['heading_deg'])
                error_yaw = abs(self.normalize_angle(current_heading - target_heading))
            else:
                error_xy = None
                error_yaw = None

            self.waypoint_results.append({
                'name': wp['name'],
                'success': True,
                'duration': duration,
                'error_xy': error_xy,
                'error_yaw': error_yaw,
                'reason': None
            })

            self.get_logger().info(
                f'[WP {self.current_waypoint_idx + 1}] REACHED in {duration:.1f}s '
                f'(error: {error_xy*1000:.1f}mm, {math.degrees(error_yaw):.2f}°)'
            )

            # Move to next waypoint
            self.current_waypoint_idx += 1

            if self.current_waypoint_idx < len(self.waypoints):
                # More waypoints to go
                self.state = self.STATE_SEND_GOAL
            else:
                # All waypoints complete!
                self.state = self.STATE_COMPLETE
                self.success = True

        elif self.state in [self.STATE_COMPLETE, self.STATE_FAILED]:
            # Done - stop timer
            self.timer.cancel()

    def get_results(self):
        """
        Get test results dictionary.

        Returns:
            dict: Results with keys: success, waypoints_completed, waypoints_total,
                  duration, waypoint_results, failure_reason
        """
        total_duration = time.time() - self.test_start_time if self.test_start_time else 0.0

        result = {
            'success': self.success,
            'collision': self.collision_detected,
            'waypoints_completed': len(self.waypoint_results),
            'waypoints_total': len(self.waypoints),
            'duration': total_duration,
            'waypoint_results': self.waypoint_results,
            'failure_reason': self.failure_reason,
            'metrics': self.episode_metrics.finalize(time.time()),
        }
        pfc = self._get_pfc_metrics()
        if pfc:
            result['pfc'] = pfc
            self.get_logger().info(
                f'[PFC] mean={pfc["pfc_mean"]:.3f} max={pfc["pfc_max"]:.3f} '
                f'integral={pfc["pfc_integral"]:.1f} ticks={pfc["pfc_count"]}')
        if self._spawned_obstacle_names:
            result['obstacles_spawned'] = list(self._spawned_obstacle_names)
        return result

    def is_complete(self):
        """Check if test is complete (success or failure)."""
        return self.state in [self.STATE_COMPLETE, self.STATE_FAILED]


def execute_waypoint_test(waypoints, timeout=120.0, quiet=False, skip_init=False, use_amcl=True,
                          record=False, bag_dir=None, cycle_num=None, trust_nav2=False,
                          obstacles=None):
    """
    Execute waypoint test with timeout.

    Args:
        waypoints: List of waypoint dicts
        timeout: Overall timeout in seconds
        quiet: Suppress info logging
        skip_init: Skip initial pose setting (for manual stack restart testing)
        use_amcl: Use AMCL localization (True) or odometry-only (False)
        record: Enable in-process rosbag recording
        bag_dir: Output directory for bag files
        cycle_num: Cycle number for bag naming
        trust_nav2: Use Nav2 STATUS_SUCCEEDED for goal detection instead of tolerance check

    Returns:
        dict: Test results
    """
    rclpy.init()

    node = Nav2WaypointFollower(waypoints, skip_init=skip_init, use_amcl=use_amcl,
                                record=record, bag_dir=bag_dir, cycle_num=cycle_num,
                                trust_nav2=trust_nav2, obstacles=obstacles)

    if quiet:
        node.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)

    start_time = time.time()

    try:
        # Spin until complete or timeout
        while not node.is_complete():
            rclpy.spin_once(node, timeout_sec=0.1)

            # Check overall timeout
            elapsed = time.time() - start_time
            if elapsed > timeout:
                node.get_logger().error(f'Overall test TIMEOUT after {elapsed:.1f}s')
                node.success = False
                node.failure_reason = 'Overall test timeout'
                # Record current waypoint's final errors for graduated scoring
                if node.current_waypoint_idx < len(node.waypoints):
                    wp = node.waypoints[node.current_waypoint_idx]
                    error_xy = None
                    error_yaw = None
                    if node.current_pose:
                        dx = node.current_pose.position.x - wp['x']
                        dy = node.current_pose.position.y - wp['y']
                        error_xy = math.sqrt(dx**2 + dy**2)
                        current_heading = node.quaternion_to_yaw(node.current_pose.orientation)
                        target_heading = math.radians(wp['heading_deg'])
                        error_yaw = abs(node.normalize_angle(current_heading - target_heading))
                    node.waypoint_results.append({
                        'name': wp['name'],
                        'success': False,
                        'duration': elapsed,
                        'error_xy': error_xy,
                        'error_yaw': error_yaw,
                        'reason': 'overall_timeout'
                    })
                break

        # Get results
        results = node.get_results()

    finally:
        # Linger so user can inspect spawned obstacles after a fail/pass.
        # Configurable via OBSTACLE_LINGER_SEC env var (default 8s if obstacles spawned, else 0).
        if node._spawned_obstacle_names:
            import os as _os
            linger = float(_os.environ.get('OBSTACLE_LINGER_SEC', '8.0'))
            if linger > 0:
                node.get_logger().info(
                    f'[obstacles] Lingering {linger}s before cleanup '
                    f'(set OBSTACLE_LINGER_SEC=0 to disable)')
                end = time.time() + linger
                while time.time() < end:
                    rclpy.spin_once(node, timeout_sec=0.1)
        # Cleanup spawned obstacles BEFORE destroying the node (uses node's service clients)
        try:
            node.cleanup_obstacles()
        except Exception as e:
            node.get_logger().error(f'[obstacles] cleanup failed: {e}')
        if node.bag_recorder:
            node.bag_recorder.close()
        node.destroy_node()
        rclpy.shutdown()

    return results


if __name__ == '__main__':
    # Simple standalone test (not for production use - use nav2_test_runner.py instead)
    import sys
    import argparse as _argparse
    import yaml

    _parser = _argparse.ArgumentParser(description='Nav2 Waypoint Follower (standalone)')
    _parser.add_argument('waypoints_file', help='Waypoints YAML file')
    _parser.add_argument('--trust-nav2', action='store_true',
                         help='Trust Nav2 STATUS_SUCCEEDED for goal detection')
    _args = _parser.parse_args()

    with open(_args.waypoints_file) as f:
        data = yaml.safe_load(f)
        waypoints = data['waypoints']
        obstacles = data.get('obstacles', []) or []

    print(f'Loaded {len(waypoints)} waypoints from {_args.waypoints_file}')
    if obstacles:
        print(f'Obstacles: {len(obstacles)} scheduled drop(s)')

    results = execute_waypoint_test(waypoints, timeout=120.0, quiet=False,
                                    trust_nav2=_args.trust_nav2, obstacles=obstacles)

    print()
    print('=' * 80)
    print('TEST RESULTS')
    print('=' * 80)
    print(f'Success: {results["success"]}')
    print(f'Waypoints completed: {results["waypoints_completed"]}/{results["waypoints_total"]}')
    print(f'Duration: {results["duration"]:.1f}s')
    if results['failure_reason']:
        print(f'Failure reason: {results["failure_reason"]}')
    print('=' * 80)

    sys.exit(0 if results['success'] else 1)
