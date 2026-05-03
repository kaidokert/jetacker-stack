#!/usr/bin/env python3
"""
Test Drive Simple Node - Working minimal implementation

⚠️  CRITICAL: NO SERVICE CALLBACKS CAN BLOCK ⚠️
ALL service callbacks MUST return immediately (<100ms).
Long-running operations execute asynchronously in the control loop.

This is a KLUDGE because we can't figure out how to use existing action messages.
Proper architecture would use ROS2 Actions for long-running operations.

Simple service-based implementation that returns immediately and runs asynchronously.
This is a stepping stone toward proper Actions architecture.

Services:
- /driveDistance: Start proportional straight-line drive (returns immediately)
- /stackReset: Stop and clear state

Topics (Published):
- /test_drive/executing: Bool - True when test running, False when complete/idle
- /test_drive/error: Bool - False on success, True on failure
- /test_drive/feedback: String (JSON) - Driving progress updates
- /test_drive/result: String (JSON) - Final result when complete
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from scenario_execution_interfaces.srv import ExecuteScenario
import math
import json
import threading
import time
import yaml
import subprocess
import datetime
from pathlib import Path
from typing import Optional, List, Dict, Any
from episode_metrics import EpisodeMetrics
from dataclasses import dataclass
from enum import Enum

# QoS settings
DEFAULT_QOS_DEPTH = 10

# Control constants
LINEAR_COMPLETION_MARGIN = 0.02  # meters
ANGULAR_COMPLETION_MARGIN = 0.01  # radians (~0.6 degrees)
CONTROL_RATE_HZ = 10.0


class ExecutionPhase(Enum):
    """Execution phase for pause-drive-pause sequence"""
    IDLE = 0
    PAUSE_BEFORE = 1
    DRIVE = 2
    ROTATE = 3
    PAUSE_AFTER = 4
    SAVE_MAP = 5
    CMD_VEL = 6


@dataclass
class DriveGoal:
    """Drive distance goal with pause phases"""
    distance_meters: float
    max_speed: float
    pause_before_seconds: float
    pause_after_seconds: float


@dataclass
class RotationGoal:
    """Rotation goal with pause phases and optional forward motion"""
    degrees: float
    max_angular_speed: float
    kP_angular: float
    min_angular_velocity: float
    forward_velocity: float  # 0 for turn-in-place, >0 for arc turns
    kP_linear: float
    min_linear_velocity: float
    pause_before_seconds: float
    pause_after_seconds: float


class ProportionalDriveController:
    """Proportional controller for straight-line driving"""

    def __init__(self, kP: float = 1.5, min_velocity: float = 0.05):
        self.kP = kP
        self.min_velocity = min_velocity
        self.start_position: Optional[tuple[float, float]] = None
        self.target_distance: float = 0.0
        self.direction: float = 1.0
        self.max_velocity: float = 0.5

    def start(self, target_distance: float, max_velocity: float, current_x: float, current_y: float):
        """Initialize controller"""
        self.target_distance = abs(target_distance)
        self.direction = 1.0 if target_distance >= 0 else -1.0
        self.start_position = (current_x, current_y)
        self.max_velocity = max_velocity

    def compute_velocity(self, current_x: float, current_y: float) -> float:
        """Compute velocity command"""
        if self.start_position is None:
            return 0.0

        dx = current_x - self.start_position[0]
        dy = current_y - self.start_position[1]
        distance_traveled = math.sqrt(dx * dx + dy * dy)
        remaining = self.target_distance - distance_traveled

        # Proportional control
        velocity_magnitude = self.kP * remaining
        velocity_magnitude = max(self.min_velocity,
                                min(self.max_velocity, velocity_magnitude))

        return self.direction * velocity_magnitude

    def is_complete(self, current_x: float, current_y: float) -> bool:
        """Check if target reached"""
        if self.start_position is None:
            return False

        dx = current_x - self.start_position[0]
        dy = current_y - self.start_position[1]
        distance_traveled = math.sqrt(dx * dx + dy * dy)
        remaining = abs(self.target_distance - distance_traveled)

        return remaining < LINEAR_COMPLETION_MARGIN

    def get_distance_traveled(self, current_x: float, current_y: float) -> float:
        """Get distance traveled"""
        if self.start_position is None:
            return 0.0

        dx = current_x - self.start_position[0]
        dy = current_y - self.start_position[1]
        return math.sqrt(dx * dx + dy * dy) * self.direction


class RotationController:
    """Proportional controller for rotation with optional forward motion"""

    def __init__(self, kP_angular: float = 2.0, min_angular_velocity: float = 0.1,
                 kP_linear: float = 0.5, min_linear_velocity: float = 0.03):
        self.kP_angular = kP_angular
        self.min_angular_velocity = min_angular_velocity
        self.kP_linear = kP_linear
        self.min_linear_velocity = min_linear_velocity
        self.start_yaw: Optional[float] = None
        self.target_rotation: float = 0.0
        self.direction: float = 1.0
        self.max_angular_speed: float = 1.0
        self.max_forward_velocity: float = 0.0
        self.forward_sign: float = 1.0

    def start(self, degrees: float, max_angular_speed: float, max_forward_velocity: float,
              current_quaternion):
        """Initialize controller"""
        self.target_rotation = math.radians(abs(degrees))
        self.direction = 1.0 if degrees >= 0 else -1.0
        self.max_angular_speed = abs(max_angular_speed)
        self.forward_sign = 1.0 if max_forward_velocity >= 0 else -1.0
        self.max_forward_velocity = abs(max_forward_velocity)
        self.start_yaw = self._get_yaw_from_quaternion(current_quaternion)

    def compute_velocities(self, current_quaternion) -> tuple[float, float]:
        """Compute angular and linear velocity commands"""
        if self.start_yaw is None:
            return 0.0, 0.0

        # Calculate angle remaining
        current_yaw = self._get_yaw_from_quaternion(current_quaternion)
        rotation_done = abs(self._angle_diff(current_yaw, self.start_yaw))
        angle_remaining = self.target_rotation - rotation_done

        # Proportional control for angular velocity
        angular_velocity = self.kP_angular * angle_remaining
        angular_velocity = min(angular_velocity, self.max_angular_speed)

        # Apply minimum angular velocity to maintain steering
        if angular_velocity < self.min_angular_velocity:
            angular_velocity = self.min_angular_velocity

        # Apply direction
        angular_velocity = angular_velocity * self.direction

        # Proportional control for forward velocity (for Ackermann arc turns)
        linear_velocity = 0.0
        if self.max_forward_velocity > 0:
            linear_velocity = self.kP_linear * angle_remaining
            linear_velocity = min(linear_velocity, self.max_forward_velocity)

            # Apply minimum linear velocity to maintain steering authority
            if linear_velocity < self.min_linear_velocity:
                linear_velocity = self.min_linear_velocity

            # Apply forward/reverse sign
            linear_velocity *= self.forward_sign

        return angular_velocity, linear_velocity

    def is_complete(self, current_quaternion) -> bool:
        """Check if target rotation reached"""
        if self.start_yaw is None:
            return False

        current_yaw = self._get_yaw_from_quaternion(current_quaternion)
        rotation_done = abs(self._angle_diff(current_yaw, self.start_yaw))

        return rotation_done >= self.target_rotation

    def get_rotation_done(self, current_quaternion) -> float:
        """Get rotation completed in radians"""
        if self.start_yaw is None:
            return 0.0

        current_yaw = self._get_yaw_from_quaternion(current_quaternion)
        return abs(self._angle_diff(current_yaw, self.start_yaw)) * self.direction

    @staticmethod
    def _get_yaw_from_quaternion(q) -> float:
        """Extract yaw from quaternion"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _angle_diff(angle1: float, angle2: float) -> float:
        """Calculate shortest angular difference"""
        diff = angle1 - angle2
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff


class TestDriveSimpleNode(Node):
    """
    Minimal test drive node with async execution.

    Services return immediately, execution happens in control loop.
    """

    def __init__(self):
        super().__init__('test_drive')

        # Parameters (for manual testing without YAML)
        self.declare_parameter('kP', 1.5)
        self.declare_parameter('min_velocity', 0.05)

        # Instance context for isolated output directories (maps, rosbags, etc.)
        # Set by test runner to ensure each test run has unique output directory
        # Format: test_cycle{num}_{timestamp} (e.g., test_cycle001_20260201_200200)
        self.declare_parameter('instance_id', '')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', DEFAULT_QOS_DEPTH)
        self.feedback_pub = self.create_publisher(String, '/test_drive/feedback', DEFAULT_QOS_DEPTH)
        self.result_pub = self.create_publisher(String, '/test_drive/result', DEFAULT_QOS_DEPTH)
        self.executing_pub = self.create_publisher(Bool, '/test_drive/executing', DEFAULT_QOS_DEPTH)
        self.error_pub = self.create_publisher(Bool, '/test_drive/error', DEFAULT_QOS_DEPTH)
        self.waypoint_pub = self.create_publisher(String, '/test_drive/waypoints', DEFAULT_QOS_DEPTH)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, DEFAULT_QOS_DEPTH)
        self.joint_state_sub = self.create_subscription(
            JointState, '/robot_joint_states', self.joint_state_callback,
            qos_profile_sensor_data)
        self._steering_cmd_joint_idx = None  # cached index for /robot_joint_commands
        self.joint_command_sub = self.create_subscription(
            JointState, '/robot_joint_commands', self.joint_command_callback,
            qos_profile_sensor_data)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, DEFAULT_QOS_DEPTH)

        # Services
        self.testdrive_service = self.create_service(
            ExecuteScenario, '/testDrive', self.testdrive_callback)
        self.reset_service = self.create_service(
            Trigger, '/stackReset', self.reset_callback)

        # State
        self.latest_odom: Optional[Odometry] = None
        self.executing = threading.Event()
        self.current_drive_goal: Optional[DriveGoal] = None
        self.current_rotation_goal: Optional[RotationGoal] = None
        self.drive_controller: Optional[ProportionalDriveController] = None
        self.rotation_controller: Optional[RotationController] = None
        self.current_phase: Optional[ExecutionPhase] = None
        self.phase_start_time: float = 0.0
        self.current_pause_duration: float = 0.0  # For standalone pause instructions

        # Multi-instruction execution state
        self.instruction_queue: List[Dict] = []  # Queue of instructions to execute
        self.current_instruction_index: int = 0   # Index into queue
        self.total_instructions: int = 0          # Total count for progress tracking
        self.checkpoints: List[Dict] = []         # Pose checkpoints after each instruction

        # Save map state
        self.current_map_save_request: Optional[str] = None  # Full path for map being saved
        self.map_save_process: Optional[Any] = None           # Subprocess for map save service call

        # Episode metrics
        self.episode_metrics = EpisodeMetrics()
        self._steering_joint_idx = None  # cached index into JointState.name
        self._rear_left_vel_idx = None
        self._rear_right_vel_idx = None

        # Control loop timer — created on demand to avoid idle CPU burn
        self.control_timer = None

        self.get_logger().info("Test Drive Simple Node initialized (idle — timer off)")
        self.get_logger().info("Services: /testDrive, /stackReset")

    def _start_control_timer(self):
        """Start 10Hz control timer when execution begins."""
        if self.control_timer is None:
            self.control_timer = self.create_timer(1.0 / CONTROL_RATE_HZ, self.control_loop)
            self.get_logger().info("Control timer started (10Hz)")

    def _stop_control_timer(self):
        """Stop control timer when idle to avoid CPU burn."""
        if self.control_timer is not None:
            self.control_timer.cancel()
            self.destroy_timer(self.control_timer)
            self.control_timer = None
            self.get_logger().info("Control timer stopped (idle)")

    def odom_callback(self, msg: Odometry):
        """Cache latest odometry"""
        self.latest_odom = msg

    def joint_state_callback(self, msg: JointState):
        """Record steering angle and rear wheel velocities for jitter metrics (from hardware bridge)."""
        if not self.executing.is_set():
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

    def joint_command_callback(self, msg: JointState):
        """Record commanded steering angle from hardware bridge (raw singularity signal)."""
        if not self.executing.is_set():
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

    def cmd_vel_callback(self, msg: Twist):
        """Record cmd_vel for smoothness metrics."""
        if not self.executing.is_set():
            return
        self.episode_metrics.record_cmd_vel(
            time.time(), msg.linear.x, msg.angular.z)

    def parse_instruction_sequence(self, instructions: List[Dict], kP: float, min_velocity: float) -> List[Dict]:
        """
        Parse YAML instructions into executable queue.

        Returns: List of instructions with resolved parameters.
        Each instruction is a dict with standardized fields for its type.
        """
        queue = []

        for i, instr in enumerate(instructions):
            instr_type = instr.get('type')

            if instr_type == 'pause':
                queue.append({
                    'type': 'pause',
                    'duration': instr.get('seconds', 0.0),
                    'description': instr.get('description', f'Pause {len(queue)+1}')
                })

            elif instr_type == 'distance':
                queue.append({
                    'type': 'distance',
                    'meters': instr.get('meters'),
                    'speed': instr.get('speed'),
                    'kP': kP,  # Use global kP from YAML
                    'min_velocity': min_velocity,  # Use global min_velocity
                    'timeout': instr.get('timeout', 30.0),
                    'description': instr.get('description', f'Drive {len(queue)+1}')
                })

            elif instr_type == 'rotation':
                queue.append({
                    'type': 'rotation',
                    'degrees': instr.get('degrees'),
                    'speed': instr.get('speed'),
                    'forward_velocity': instr.get('forward_velocity', 0.0),
                    'kP_angular': instr.get('kP_angular', 2.0),
                    'min_angular_velocity': instr.get('min_angular_velocity', 0.1),
                    'kP_linear': instr.get('kP_linear', 0.5),
                    'min_linear_velocity': instr.get('min_linear_velocity', 0.03),
                    'timeout': instr.get('timeout', 30.0),
                    'description': instr.get('description', f'Turn {len(queue)+1}')
                })

            elif instr_type == 'cmd_vel':
                queue.append({
                    'type': 'cmd_vel',
                    'vx': instr.get('vx', 0.0),
                    'wz': instr.get('wz', 0.0),
                    'seconds': instr.get('seconds', 1.0),
                    'description': instr.get('description', f'CmdVel {len(queue)+1}')
                })

            elif instr_type == 'save_map':
                queue.append({
                    'type': 'save_map',
                    'map_name': instr.get('map_name', 'map'),
                    'timeout': instr.get('timeout', 10.0),
                    'description': instr.get('description', f'Save map {len(queue)+1}')
                })

        return queue

    def calculate_total_timeout(self, queue: List[Dict]) -> float:
        """
        Calculate estimated execution time for entire instruction queue.

        Returns: Total estimated time in seconds with 20% safety margin.
        """
        total = 0.0

        for instr in queue:
            if instr['type'] == 'pause':
                # Pause: exact duration
                total += instr['duration']

            elif instr['type'] == 'distance':
                # Distance: estimated time = distance / speed + buffer
                meters = instr['meters']
                speed = instr['speed']
                estimated_time = abs(meters) / speed if speed > 0 else 30.0
                total += estimated_time + 5.0  # Add 5s buffer for acceleration/deceleration

            elif instr['type'] == 'rotation':
                # Rotation: estimated time = degrees / angular_speed + buffer
                degrees = abs(instr['degrees'])
                speed_rad_s = instr['speed']
                degrees_per_second = math.degrees(speed_rad_s)
                estimated_time = degrees / degrees_per_second if degrees_per_second > 0 else 30.0
                total += estimated_time + 5.0  # Add 5s buffer for acceleration/deceleration

            elif instr['type'] == 'cmd_vel':
                # Open-loop cmd_vel: exact duration
                total += instr['seconds']

            elif instr['type'] == 'save_map':
                # Save map: use timeout value
                total += instr.get('timeout', 10.0)

        # Add 20% safety margin
        total_with_margin = total * 1.2

        return total_with_margin

    def testdrive_callback(self, request: ExecuteScenario.Request, response: ExecuteScenario.Response):
        """
        Execute driving instructions from YAML file (returns immediately).

        Service loads YAML, parses pause-drive-pause sequence, and queues for execution.
        """
        # Check if already executing
        if self.executing.is_set():
            self.get_logger().error("Test drive already in progress")
            response.result = False
            return response

        # Check odometry available
        if self.latest_odom is None:
            self.get_logger().error("No odometry available")
            response.result = False
            return response

        # Extract filename from Scenario message
        filename = request.scenario.scenario_file
        if not filename:
            filename = request.scenario.name

        if not filename:
            self.get_logger().error("No scenario filename provided")
            response.result = False
            return response

        # Load YAML file
        yaml_path = Path('/workspace/driving_instructions') / filename
        if not yaml_path.suffix:
            yaml_path = yaml_path.with_suffix('.yaml')

        if not yaml_path.exists():
            self.get_logger().error(f"Instruction file not found: {yaml_path}")
            response.result = False
            return response

        try:
            # Parse YAML
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)

            # Extract global parameters
            kP = data.get('kP', 1.5)
            min_velocity = data.get('min_velocity', 0.05)

            self.get_logger().info(f"Loaded instruction file: {yaml_path.name}")
            self.get_logger().info(f"Controller parameters: kP={kP}, min_velocity={min_velocity}")

            # Parse instruction sequence into queue
            instructions = data.get('instructions', [])
            if not instructions:
                self.get_logger().error(f"No instructions found in {yaml_path}")
                response.result = False
                return response

            # Clear all stale state from previous test run
            self._clear_execution_state()

            # Parse all instructions into queue
            self.instruction_queue = self.parse_instruction_sequence(instructions, kP, min_velocity)
            self.current_instruction_index = 0
            self.total_instructions = len(self.instruction_queue)

            if not self.instruction_queue:
                self.get_logger().error(f"No valid instructions after parsing {yaml_path}")
                response.result = False
                return response

            # Calculate estimated execution time
            estimated_duration = self.calculate_total_timeout(self.instruction_queue)

            self.get_logger().info(f"Parsed {self.total_instructions} instruction(s) from {yaml_path.name}")
            self.get_logger().info(f"Estimated execution time: {estimated_duration:.1f}s (with 20% margin)")
            for i, instr in enumerate(self.instruction_queue):
                self.get_logger().info(f"  [{i+1}] {instr['type']}: {instr.get('description', 'N/A')}")

            # Set executing flag, start control timer, signal test started
            self.executing.set()
            self._start_control_timer()
            self.episode_metrics.begin(time.time())
            self.executing_pub.publish(Bool(data=True))

            # Start first instruction
            first_instruction = self.instruction_queue[0]
            self.get_logger().info(
                f"Starting instruction 1/{self.total_instructions}: "
                f"{first_instruction['type']} - {first_instruction.get('description', 'N/A')}"
            )
            self.start_instruction(first_instruction)

            response.result = True
            return response

        except Exception as e:
            self.get_logger().error(f"Failed to load/parse YAML: {e}")
            response.result = False
            return response

    def _clear_execution_state(self):
        """Reset all execution state between test runs.

        Called from both testdrive_callback (before new test) and
        reset_callback (manual reset).  Eliminates stale state leaking
        across cycles in a long-lived node.
        """
        self.executing.clear()
        self._stop_control_timer()
        self.current_drive_goal = None
        self.current_rotation_goal = None
        self.drive_controller = None
        self.rotation_controller = None
        self.current_phase = None
        self.phase_start_time = 0.0
        self.current_pause_duration = 0.0
        self.instruction_queue = []
        self.current_instruction_index = 0
        self.total_instructions = 0
        self.checkpoints = []
        self._drive_debug_counter = 0
        self._rotate_debug_counter = 0
        self.episode_metrics.reset()

        # Kill any lingering map save subprocess
        if self.map_save_process is not None:
            try:
                self.map_save_process.kill()
                self.map_save_process.wait(timeout=2.0)
            except Exception:
                pass
        self.current_map_save_request = None
        self.map_save_process = None

    def reset_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Reset state and stop"""
        self._clear_execution_state()
        self.stop_robot()

        # Signal test stopped (for drive_test.py polling)
        self.executing_pub.publish(Bool(data=False))

        response.success = True
        response.message = "Reset complete"

        self.get_logger().info("Stack reset")
        return response

    def advance_to_next_instruction(self):
        """Move to next instruction in queue, or complete test if queue is done."""
        self.current_instruction_index += 1

        # Check if more instructions remain
        if self.current_instruction_index >= len(self.instruction_queue):
            # All instructions complete - finish test
            self.get_logger().info(f"All {self.total_instructions} instruction(s) complete!")

            result = {
                'success': True,
                'instructions_completed': self.total_instructions,
                'message': 'All instructions complete',
                'checkpoints': self.checkpoints,
                'metrics': self.episode_metrics.finalize(time.time()),
            }
            self.result_pub.publish(String(data=json.dumps(result)))
            self.executing_pub.publish(Bool(data=False))
            self.error_pub.publish(Bool(data=False))

            self.stop_robot()
            self.executing.clear()
            self._stop_control_timer()
            self.current_phase = ExecutionPhase.IDLE
            return

        # More instructions - start next one
        next_instruction = self.instruction_queue[self.current_instruction_index]
        self.get_logger().info(
            f"Starting instruction {self.current_instruction_index + 1}/{self.total_instructions}: "
            f"{next_instruction['type']} - {next_instruction.get('description', 'N/A')}"
        )

        self.start_instruction(next_instruction)

    def start_instruction(self, instruction: Dict):
        """Start executing a single instruction from the queue."""
        instr_type = instruction['type']

        # [INSTRUMENTATION] Log full instruction details
        self.get_logger().info(f"[INSTR_START] Full instruction: {instruction}")

        if instr_type == 'pause':
            self.get_logger().info(f"[PHASE_CHANGE] → PAUSE_BEFORE (duration={instruction['duration']}s)")
            self.current_phase = ExecutionPhase.PAUSE_BEFORE
            self.phase_start_time = time.time()
            self.current_pause_duration = instruction['duration']
            self.stop_robot()

        elif instr_type == 'distance':
            # Get current state
            pos = self.latest_odom.pose.pose.position
            orientation = self.latest_odom.pose.pose.orientation
            current_heading = math.degrees(self._get_yaw_from_quaternion(orientation)) % 360

            # [INSTRUMENTATION] Log DRIVE start with current state
            self.get_logger().info(
                f"[PHASE_CHANGE] → DRIVE "
                f"(target={instruction['meters']}m, speed={instruction['speed']}, "
                f"kP={instruction['kP']}, current_pos=({pos.x:.3f},{pos.y:.3f}), "
                f"current_heading={current_heading:.1f}°)"
            )

            # Create drive goal
            self.current_drive_goal = DriveGoal(
                distance_meters=instruction['meters'],
                max_speed=instruction['speed'],
                pause_before_seconds=0.0,
                pause_after_seconds=0.0
            )

            # Initialize drive controller
            self.drive_controller = ProportionalDriveController(
                kP=instruction['kP'],
                min_velocity=instruction['min_velocity']
            )

            # Start drive
            self.current_phase = ExecutionPhase.DRIVE
            self._drive_debug_counter = 0  # Reset counter for new drive
            self.drive_controller.start(
                self.current_drive_goal.distance_meters,
                self.current_drive_goal.max_speed,
                pos.x,
                pos.y
            )
            yaw = self._get_yaw_from_quaternion(orientation)
            self.episode_metrics.begin_drive_phase(pos.x, pos.y, yaw)

        elif instr_type == 'rotation':
            # Get current state
            orientation = self.latest_odom.pose.pose.orientation
            current_heading = math.degrees(self._get_yaw_from_quaternion(orientation)) % 360

            # [INSTRUMENTATION] Log ROTATE start with current state
            self.get_logger().info(
                f"[PHASE_CHANGE] → ROTATE "
                f"(target={instruction['degrees']}°, speed={instruction['speed']}, "
                f"current_heading={current_heading:.1f}°)"
            )

            # Create rotation goal
            self.current_rotation_goal = RotationGoal(
                degrees=instruction['degrees'],
                max_angular_speed=instruction['speed'],
                kP_angular=instruction['kP_angular'],
                min_angular_velocity=instruction['min_angular_velocity'],
                forward_velocity=instruction['forward_velocity'],
                kP_linear=instruction['kP_linear'],
                min_linear_velocity=instruction['min_linear_velocity'],
                pause_before_seconds=0.0,
                pause_after_seconds=0.0
            )

            # Initialize rotation controller
            self.rotation_controller = RotationController(
                kP_angular=instruction['kP_angular'],
                min_angular_velocity=instruction['min_angular_velocity'],
                kP_linear=instruction['kP_linear'],
                min_linear_velocity=instruction['min_linear_velocity']
            )

            # Start rotation
            self.current_phase = ExecutionPhase.ROTATE
            self._rotate_debug_counter = 0  # Reset counter for new rotation
            self.rotation_controller.start(
                self.current_rotation_goal.degrees,
                self.current_rotation_goal.max_angular_speed,
                self.current_rotation_goal.forward_velocity,
                orientation
            )

        elif instr_type == 'cmd_vel':
            # Open-loop cmd_vel: publish fixed (vx, wz) for N seconds
            self.get_logger().info(
                f"[PHASE_CHANGE] → CMD_VEL "
                f"(vx={instruction['vx']}, wz={instruction['wz']}, "
                f"seconds={instruction['seconds']})"
            )
            self.current_phase = ExecutionPhase.CMD_VEL
            self.phase_start_time = time.time()

        elif instr_type == 'save_map':
            # [INSTRUMENTATION] Log SAVE_MAP start
            self.get_logger().info(
                f"[PHASE_CHANGE] → SAVE_MAP "
                f"(map_name={instruction['map_name']})"
            )

            # Get instance_id parameter (set by test runner)
            instance_id = self.get_parameter('instance_id').get_parameter_value().string_value
            if not instance_id:
                # Fallback: use timestamp if instance_id not set
                instance_id = datetime.datetime.now().strftime('manual_%Y%m%d_%H%M%S')

            # Construct map save path: logs/maps/{instance_id}/{map_name}
            # slam_toolbox expects filename without extension
            map_dir = Path('/workspace/logs/maps') / instance_id
            map_dir.mkdir(parents=True, exist_ok=True)
            map_path = map_dir / instruction['map_name']

            self.get_logger().info(f"[SAVE_MAP] Saving to: {map_path}")

            # Store for tracking
            self.current_map_save_request = str(map_path)

            # Call nav2_map_server map_saver_cli directly (bypasses broken slam_toolbox service)
            # slam_toolbox's /slam_toolbox/save_map service has a QoS bug (TRANSIENT_LOCAL vs VOLATILE mismatch)
            # that causes "Failed to spin map subscription" errors. The nav2_map_server CLI tool works reliably.
            # Produces .pgm + .yaml files (occupancy grid images for quality analysis)
            # Must source ROS environment for subprocess (Popen doesn't inherit parent's environment)
            cmd = f'bash -c "source /opt/ros/jazzy/setup.bash && ros2 run nav2_map_server map_saver_cli -f {map_path} --ros-args -p map_subscribe_transient_local:=true"'

            # Launch async subprocess
            self.map_save_process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            self.phase_start_time = time.time()
            self.current_phase = ExecutionPhase.SAVE_MAP

    @staticmethod
    def _get_yaw_from_quaternion(q) -> float:
        """Extract yaw from quaternion"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """Control loop (10Hz) - handles pause-drive-pause sequence"""
        if not self.executing.is_set():
            return

        if self.latest_odom is None:
            return

        pos = self.latest_odom.pose.pose.position
        orientation = self.latest_odom.pose.pose.orientation
        current_time = time.time()

        # Determine current goal
        current_goal = self.current_drive_goal or self.current_rotation_goal

        # Phase: PAUSE_BEFORE (handles standalone pause instructions)
        if self.current_phase == ExecutionPhase.PAUSE_BEFORE:
            elapsed = current_time - self.phase_start_time

            # Check if pause is complete
            if elapsed >= self.current_pause_duration:
                # [INSTRUMENTATION] Log pause completion
                self.get_logger().info(f"[PAUSE_COMPLETE] duration={elapsed:.1f}s")
                self.stop_robot()

                # Publish waypoint marker
                current_instruction = self.instruction_queue[self.current_instruction_index]
                self.publish_waypoint(current_instruction, elapsed)

                # Clear pause state
                self.current_pause_duration = 0.0

                # Advance to next instruction
                self.advance_to_next_instruction()
            else:
                # Still pausing - publish zero velocity
                self.stop_robot()

                # Publish feedback
                feedback = {
                    'phase': 'pause',
                    'elapsed': elapsed,
                    'remaining': self.current_pause_duration - elapsed
                }
                self.feedback_pub.publish(String(data=json.dumps(feedback)))
            return

        # Phase: DRIVE
        elif self.current_phase == ExecutionPhase.DRIVE:
            # Check drive completion
            if self.drive_controller.is_complete(pos.x, pos.y):
                distance_traveled = self.drive_controller.get_distance_traveled(pos.x, pos.y)
                error = self.current_drive_goal.distance_meters - distance_traveled
                current_heading = math.degrees(self._get_yaw_from_quaternion(orientation)) % 360

                # [INSTRUMENTATION] Log completion with heading
                self.get_logger().info(
                    f"[DRIVE_COMPLETE] distance={distance_traveled:.3f}m (error={error:.4f}m), "
                    f"final_heading={current_heading:.1f}°"
                )
                self.stop_robot()
                self.episode_metrics.end_drive_phase()

                # Record checkpoint
                self.checkpoints.append({
                    'instruction': self.current_instruction_index + 1,
                    'type': 'drive',
                    'distance': round(distance_traveled, 4),
                    'error': round(error, 4),
                    'heading': round(current_heading, 1),
                    'position': {'x': round(pos.x, 4), 'y': round(pos.y, 4)},
                })

                # Publish waypoint marker
                current_instruction = self.instruction_queue[self.current_instruction_index]
                self.publish_waypoint(current_instruction, distance_traveled)

                # Clear drive state
                self.current_drive_goal = None
                self.drive_controller = None

                # Advance to next instruction
                self.advance_to_next_instruction()
                return

            # Continue driving
            velocity = self.drive_controller.compute_velocity(pos.x, pos.y)
            current_heading = math.degrees(self._get_yaw_from_quaternion(orientation)) % 360

            # [INSTRUMENTATION] Periodic logging during drive (every 10 cycles = 1 second)
            self._drive_debug_counter += 1

            if self._drive_debug_counter % 10 == 0:
                distance_traveled = self.drive_controller.get_distance_traveled(pos.x, pos.y)
                self.get_logger().info(
                    f"[DRIVE_ACTIVE] vel={velocity:.3f}m/s, heading={current_heading:.1f}°, "
                    f"traveled={distance_traveled:.3f}m"
                )

            cmd = Twist()
            cmd.linear.x = velocity
            cmd.angular.z = 0.0  # Explicit zero for clarity

            # [INSTRUMENTATION] Log cmd_vel (every 10 cycles)
            if self._drive_debug_counter % 10 == 0:
                self.get_logger().info(f"[CMD_VEL] linear.x={cmd.linear.x:.3f}, angular.z={cmd.angular.z:.3f}")

            self.cmd_vel_pub.publish(cmd)

            # Record drive sample for cross-track / heading metrics
            yaw = self._get_yaw_from_quaternion(orientation)
            self.episode_metrics.record_drive_sample(pos.x, pos.y, yaw)

            # Publish feedback
            distance_traveled = self.drive_controller.get_distance_traveled(pos.x, pos.y)
            remaining = self.current_drive_goal.distance_meters - abs(distance_traveled)

            feedback = {
                'phase': 'drive',
                'current_distance': distance_traveled,
                'current_speed': velocity,
                'remaining_distance': remaining
            }
            self.feedback_pub.publish(String(data=json.dumps(feedback)))
            return

        # Phase: ROTATE
        elif self.current_phase == ExecutionPhase.ROTATE:
            # Check if rotation controller exists
            if self.rotation_controller is None:
                self.get_logger().error("ROTATE phase but rotation_controller is None!")
                self.advance_to_next_instruction()
                return

            # Check rotation completion
            is_done = self.rotation_controller.is_complete(orientation)
            rotation_done_rad = self.rotation_controller.get_rotation_done(orientation)
            rotation_done_deg = math.degrees(rotation_done_rad)

            # Debug logging every 10 cycles (1 second)
            self._rotate_debug_counter += 1

            # Continue rotating
            angular_velocity, linear_velocity = self.rotation_controller.compute_velocities(orientation)

            # [INSTRUMENTATION] Periodic logging during rotation
            if self._rotate_debug_counter % 10 == 0:
                current_heading = math.degrees(self._get_yaw_from_quaternion(orientation)) % 360
                self.get_logger().info(
                    f"[ROTATE_ACTIVE] target={self.current_rotation_goal.degrees:.1f}°, "
                    f"done={rotation_done_deg:.1f}°, current_heading={current_heading:.1f}°, "
                    f"ang_vel={angular_velocity:.3f}rad/s, lin_vel={linear_velocity:.3f}m/s, "
                    f"complete={is_done}"
                )

            if is_done:
                error = self.current_rotation_goal.degrees - rotation_done_deg
                current_heading = math.degrees(self._get_yaw_from_quaternion(orientation)) % 360

                # [INSTRUMENTATION] Log completion with heading
                self.get_logger().info(
                    f"[ROTATE_COMPLETE] rotation={rotation_done_deg:.1f}° (error={error:.2f}°), "
                    f"final_heading={current_heading:.1f}°"
                )
                self.stop_robot()

                # Record checkpoint
                self.checkpoints.append({
                    'instruction': self.current_instruction_index + 1,
                    'type': 'rotate',
                    'rotation': round(rotation_done_deg, 1),
                    'error': round(error, 2),
                    'heading': round(current_heading, 1),
                    'position': {'x': round(pos.x, 4), 'y': round(pos.y, 4)},
                })

                # Publish waypoint marker
                current_instruction = self.instruction_queue[self.current_instruction_index]
                self.publish_waypoint(current_instruction, rotation_done_deg)

                # Clear rotation state
                self.current_rotation_goal = None
                self.rotation_controller = None
                self._rotate_debug_counter = 0

                # Advance to next instruction
                self.advance_to_next_instruction()
                return

            # Publish rotation command (velocities already computed above for debug logging)
            cmd = Twist()
            cmd.linear.x = linear_velocity
            cmd.angular.z = angular_velocity

            # [INSTRUMENTATION] Log cmd_vel (every 10 cycles)
            if self._rotate_debug_counter % 10 == 0:
                self.get_logger().info(f"[CMD_VEL] linear.x={cmd.linear.x:.3f}, angular.z={cmd.angular.z:.3f}")

            self.cmd_vel_pub.publish(cmd)

            # Publish feedback
            rotation_done_rad = self.rotation_controller.get_rotation_done(orientation)
            rotation_done = math.degrees(rotation_done_rad)
            remaining = self.current_rotation_goal.degrees - abs(rotation_done)

            feedback = {
                'phase': 'rotate',
                'current_rotation': rotation_done,
                'angular_velocity': angular_velocity,
                'linear_velocity': linear_velocity,
                'remaining_rotation': remaining
            }
            self.feedback_pub.publish(String(data=json.dumps(feedback)))
            return

        # Phase: CMD_VEL (open-loop fixed velocity for N seconds)
        elif self.current_phase == ExecutionPhase.CMD_VEL:
            elapsed = current_time - self.phase_start_time
            current_instruction = self.instruction_queue[self.current_instruction_index]

            if elapsed >= current_instruction['seconds']:
                # Done
                self.get_logger().info(
                    f"[CMD_VEL_COMPLETE] duration={elapsed:.1f}s"
                )
                self.stop_robot()

                # Record checkpoint
                self.checkpoints.append({
                    'instruction': self.current_instruction_index + 1,
                    'type': 'cmd_vel',
                    'vx': current_instruction['vx'],
                    'wz': current_instruction['wz'],
                    'duration': round(elapsed, 2),
                    'position': {'x': round(pos.x, 4), 'y': round(pos.y, 4)},
                })

                self.advance_to_next_instruction()
                return

            # Publish fixed cmd_vel
            cmd = Twist()
            cmd.linear.x = current_instruction['vx']
            cmd.angular.z = current_instruction['wz']
            self.cmd_vel_pub.publish(cmd)
            return

        # Phase: SAVE_MAP
        elif self.current_phase == ExecutionPhase.SAVE_MAP:
            elapsed = current_time - self.phase_start_time
            current_instruction = self.instruction_queue[self.current_instruction_index]
            timeout = current_instruction.get('timeout', 10.0)

            # Check if subprocess completed
            if self.map_save_process is not None:
                poll_result = self.map_save_process.poll()

                if poll_result is not None:
                    # Process finished
                    stdout, stderr = self.map_save_process.communicate()

                    if poll_result == 0:
                        self.get_logger().info(
                            f"[SAVE_MAP_COMPLETE] Map saved to: {self.current_map_save_request} "
                            f"(elapsed={elapsed:.1f}s)"
                        )

                        # Publish waypoint marker (using elapsed time as value)
                        self.publish_waypoint(current_instruction, elapsed)

                        # Clear state
                        self.current_map_save_request = None
                        self.map_save_process = None

                        # Advance to next instruction
                        self.advance_to_next_instruction()
                        return
                    else:
                        # Service call failed
                        stderr_str = stderr.decode('utf-8', errors='replace') if stderr else ''
                        self.get_logger().error(
                            f"[SAVE_MAP_ERROR] Failed to save map (exit code {poll_result}): {stderr_str[:200]}"
                        )

                        # Clear state and advance anyway (don't fail entire test)
                        self.current_map_save_request = None
                        self.map_save_process = None
                        self.advance_to_next_instruction()
                        return

                elif elapsed > timeout:
                    # Timeout
                    self.get_logger().error(
                        f"[SAVE_MAP_TIMEOUT] Map save timed out after {timeout}s"
                    )

                    # Kill subprocess
                    try:
                        self.map_save_process.kill()
                        self.map_save_process.wait(timeout=2.0)
                    except:
                        pass

                    # Clear state and advance anyway
                    self.current_map_save_request = None
                    self.map_save_process = None
                    self.advance_to_next_instruction()
                    return

                else:
                    # Still waiting - publish feedback
                    feedback = {
                        'phase': 'save_map',
                        'map_path': self.current_map_save_request,
                        'elapsed': elapsed,
                        'remaining': timeout - elapsed
                    }
                    self.feedback_pub.publish(String(data=json.dumps(feedback)))
                    return
            else:
                # No process - shouldn't happen, advance anyway
                self.get_logger().error("[SAVE_MAP] No map save process found")
                self.advance_to_next_instruction()
                return

    def publish_waypoint(self, instruction: dict, value: float):
        """Publish waypoint marker for completed instruction."""
        # Get current pose from odometry
        pos = self.latest_odom.pose.pose.position
        orientation = self.latest_odom.pose.pose.orientation
        heading = self._get_yaw_from_quaternion(orientation)

        waypoint_data = {
            'timestamp': self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9,
            'instruction_number': self.current_instruction_index + 1,
            'instruction_type': instruction['type'],
            'description': instruction.get('description', 'N/A'),
            'value': value,  # distance in m or rotation in degrees
            'odometry_pose': {
                'position': {
                    'x': pos.x,
                    'y': pos.y,
                    'z': pos.z
                },
                'orientation': {
                    'heading_deg': math.degrees(heading) % 360
                }
            }
        }
        self.waypoint_pub.publish(String(data=json.dumps(waypoint_data)))
        self.get_logger().info(f"[WAYPOINT] Published marker for instruction {waypoint_data['instruction_number']}")

    def stop_robot(self):
        """Stop robot"""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)


def main(args=None):
    import os
    if os.environ.get('NEUTER', '').lower() in ('1', 'true', 'yes'):
        # Neuter mode: keep container alive for docker exec without rclpy overhead
        import signal
        print("[test_drive] NEUTER mode — sleeping (no ROS node)", flush=True)
        signal.signal(signal.SIGTERM, lambda *_: exit(0))
        while True:
            time.sleep(3600)

    rclpy.init(args=args)

    try:
        node = TestDriveSimpleNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
