# Robot Movement Configuration - Jetacker Tricycle Drive

This document explains the complete configuration required to achieve actual robot movement in Gazebo simulation using ros2_control with topic-based hardware interface.

## Overview

The Jetacker robot uses a **tricycle drive** configuration:
- **Rear wheels**: Powered/driven wheels (differential drive)
- **Front wheels**: Passive/freewheeling caster wheels
- **Steering**: Single virtual steering joint at front axle center

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    ROS2 Control Stack                            │
├─────────────────────────────────────────────────────────────────┤
│  tricycle_steering_controller                                    │
│         ↓                                                        │
│  JointStateTopicSystem (Hardware Interface)                      │
│         ↓                                                        │
│  /robot_joint_commands topic                                     │
└──────────────────┬──────────────────────────────────────────────┘
                   │
                   ↓
┌─────────────────────────────────────────────────────────────────┐
│            gazebo_hardware_bridge.py (Python)                    │
│  - Receives: /robot_joint_commands                               │
│  - Publishes: /robot_joint_states                                │
│  - Commands Gazebo via: gz topic pub / gz service               │
└──────────────────┬──────────────────────────────────────────────┘
                   │
                   ↓
┌─────────────────────────────────────────────────────────────────┐
│                  Gazebo Simulation                               │
│  - JointController plugins on rear wheels (velocity control)    │
│  - NO controllers on front wheels (freewheeling)                │
│  - Friction configured on all surfaces                           │
└─────────────────────────────────────────────────────────────────┘
```

## Critical Configuration Files

### 1. URDF Hardware Interface (`models/jetacker/jetacker.urdf`)

**Purpose**: Defines ros2_control hardware interface configuration

**Key Changes**:
```xml
<ros2_control name="JetackerAckermannSystem" type="system">
  <hardware>
    <plugin>joint_state_topic_hardware_interface/JointStateTopicSystem</plugin>
    <param name="sum_wrapped_joint_states">false</param>
    <param name="trigger_joint_command_threshold">-1.0</param>
  </hardware>

  <!-- Steering joint: position control -->
  <joint name="front_steering_joint">
    <command_interface name="position"/>
    <state_interface name="position">
      <param name="initial_value">0.0</param>
    </state_interface>
    <state_interface name="velocity">
      <param name="initial_value">0.0</param>
    </state_interface>
  </joint>

  <!-- Rear wheels: velocity control -->
  <joint name="rear_left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position">
      <param name="initial_value">0.0</param>
    </state_interface>
    <state_interface name="velocity">
      <param name="initial_value">0.0</param>
    </state_interface>
  </joint>

  <joint name="rear_right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position">
      <param name="initial_value">0.0</param>
    </state_interface>
    <state_interface name="velocity">
      <param name="initial_value">0.0</param>
    </state_interface>
  </joint>

  <!-- Turret joint: position control (not used by tricycle controller) -->
  <joint name="turret_joint">
    <command_interface name="position"/>
    <state_interface name="position">
      <param name="initial_value">0.0</param>
    </state_interface>
    <state_interface name="velocity">
      <param name="initial_value">0.0</param>
    </state_interface>
  </joint>
</ros2_control>
```

**Critical Details**:
- ✅ Uses `JointStateTopicSystem` instead of `GazeboSimSystem` to match real robot architecture
- ✅ Position-controlled joints need BOTH position AND velocity state interfaces for joint_state_broadcaster
- ✅ Velocity-controlled joints need BOTH position AND velocity state interfaces

### 2. URDF Joint Axes (`models/jetacker/jetacker.urdf`)

**Purpose**: Define rotation axes for wheel joints

**CRITICAL FIX - Wheel Axis Directions**:
```xml
<!-- Left wheel: standard Y-axis -->
<joint name="rear_left_wheel_joint" type="continuous">
  <parent link="base_link" />
  <child link="rear_left_wheel" />
  <origin rpy="0 0 0" xyz="-0.10599 0.090107 0.050328" />
  <axis xyz="0 1 0" />  <!-- Standard Y-axis -->
</joint>

<!-- Right wheel: INVERTED Y-axis -->
<joint name="rear_right_wheel_joint" type="continuous">
  <parent link="base_link" />
  <child link="rear_right_wheel" />
  <origin rpy="0 0 0" xyz="-0.106 -0.091444 0.050335" />
  <axis xyz="0 -1 0" />  <!-- INVERTED for opposite side mounting -->
</joint>
```

**Why This Matters**:
- Wheels on opposite sides need opposite axis directions
- Without this, same velocity commands cause opposite rotations
- This is a differential drive requirement

### 3. Gazebo Model SDF (`models/jetacker/model.sdf`)

**Purpose**: Gazebo-specific physics and controller plugins

**A. Rear Wheel Controllers (Velocity Control)**:
```xml
<!-- Rear wheels: JointController for velocity control -->
<plugin
  filename="gz-sim-joint-controller-system"
  name="gz::sim::systems::JointController">
  <joint_name>rear_left_wheel_joint</joint_name>
  <initial_velocity>0.0</initial_velocity>
</plugin>

<plugin
  filename="gz-sim-joint-controller-system"
  name="gz::sim::systems::JointController">
  <joint_name>rear_right_wheel_joint</joint_name>
  <initial_velocity>0.0</initial_velocity>
</plugin>
```

**B. Front Wheels (NO Controllers - Freewheeling)**:
```xml
<!-- Front wheels: NO CONTROLLER PLUGINS -->
<!-- They spin freely based on physics when robot moves -->
```

**CRITICAL**: Front wheels MUST NOT have JointController plugins!
- If they have controllers with `initial_velocity=0.0`, they act as **brakes**
- Remove ALL controller plugins from front wheel joints
- They will spin freely based on physics/friction

**C. Wheel Friction (ALL wheels)**:
```xml
<collision name='rear_left_wheel_collision'>
  <geometry>
    <cylinder>
      <length>0.039</length>
      <radius>0.050</radius>
    </cylinder>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>      <!-- Friction coefficient -->
        <mu2>1.0</mu2>    <!-- Secondary friction -->
        <slip1>0.0</slip1> <!-- No slip -->
        <slip2>0.0</slip2>
      </ode>
    </friction>
  </surface>
</collision>
```

**Why This Matters**:
- Without friction, wheels spin in place (no traction)
- Applies to ALL four wheels
- Same parameters for all wheels

### 4. World/Ground Configuration (`worlds/jetacker/jetacker.sdf`)

**Purpose**: Define ground plane with proper friction

**Ground Plane Friction**:
```xml
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>1.0</mu>
            <mu2>1.0</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
  </link>
</model>
```

**Why This Matters**:
- Ground must have friction for wheels to grip
- Without ground friction, wheels spin but robot doesn't move
- Friction coefficients must be compatible with wheel friction

### 5. Gazebo Hardware Bridge (`ros/gazebo_hardware_bridge.py`)

**Purpose**: Bridge between ros2_control topics and Gazebo gz transport

**Key Features**:
- **Subscribes**: `/robot_joint_commands` (from JointStateTopicSystem)
- **Publishes**: `/robot_joint_states` (to JointStateTopicSystem)
- **Commands Gazebo**: Via `gz topic pub` and `gz service` commands

**CRITICAL FIX - gz Command Path**:
```python
class GazeboHardwareBridge(Node):
    # Full path to gz command (not in PATH without sourcing ROS)
    GZ_CMD = '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz'

    def _set_joint_velocity(self, joint_name: str, velocity: float):
        subprocess.run([
            self.GZ_CMD, 'topic',  # Use full path!
            '-t', f'/model/{self.model_name}/joint/{joint_name}/cmd_vel',
            '-m', 'gz.msgs.Double',
            '-p', f'data: {velocity}'
        ], check=False, capture_output=True)
```

**Why This Matters**:
- The `gz` command is not in PATH by default
- Without full path, ALL commands silently fail
- Robot appears to work but nothing actually happens in Gazebo

**Joint Commanding Logic**:
```python
def command_callback(self, msg: JointState):
    # Position-controlled joints (must match controller config order)
    position_joints = ['front_steering_joint', 'turret_joint']

    # Velocity-controlled joints (must match controller config order!)
    velocity_joints = ['rear_right_wheel_joint', 'rear_left_wheel_joint']

    # Apply position commands
    for i, joint_name in enumerate(position_joints):
        if i < len(msg.position):
            pos_cmd = msg.position[i]
            self._set_joint_position(joint_name, pos_cmd)

            # Command mimic steering joints together
            if joint_name == 'front_steering_joint':
                self._set_joint_position('front_left_wheel_steering_joint', pos_cmd)
                self._set_joint_position('front_right_wheel_steering_joint', pos_cmd)

    # Apply velocity commands
    for i, joint_name in enumerate(velocity_joints):
        if i < len(msg.velocity):
            vel_cmd = msg.velocity[i]
            self._set_joint_velocity(joint_name, vel_cmd)
```

### 6. Controller Configuration (`config/jetacker/tricycle_controller.yaml`)

**Purpose**: Configure tricycle steering controller behavior

**Critical Settings**:
```yaml
tricycle_steering_controller:
  ros__parameters:
    steering_joints_names: ["front_steering_joint"]
    traction_joints_names: ["rear_right_wheel_joint", "rear_left_wheel_joint"]

    # Control mode
    open_loop: true  # CRITICAL: Set to true for simulation
    position_feedback: false

    # Physical parameters (matching jetacker model)
    wheelbase: 0.213  # meters
    traction_track_width: 0.182  # meters
    traction_wheels_radius: 0.050  # meters

    # Odometry
    enable_odom_tf: true
    odom_frame_id: "odom"
    base_frame_id: "base_link"

joint_state_broadcaster:
  ros__parameters:
    joints:
      - front_steering_joint
      - rear_left_wheel_joint
      - rear_right_wheel_joint
      - turret_joint
    state_publish_rate: 50.0
    interfaces:
      - position
      - velocity
```

**Why open_loop=true**:
- Closed-loop mode requires accurate velocity feedback
- Our hardware bridge returns dummy velocities (0.0, 0.0)
- Open-loop bypasses feedback requirements for sim testing

### 7. Joint State Management

**Purpose**: Merge controlled and passive joint states for robot_state_publisher

**A. Passive Joint Publisher (`ros/passive_joint_publisher.py`)**:
```python
# Publishes states for uncontrolled joints (front wheels)
self.passive_joints = [
    'front_left_wheel_steering_joint',
    'front_right_wheel_steering_joint',
    'front_left_wheel_joint',
    'front_right_wheel_joint'
]
```

**B. Joint State Merger (`ros/joint_state_merger.py`)**:
```python
# Combines:
# - /dynamic_joint_states (4 controlled joints from joint_state_broadcaster)
# - /passive_joint_states (4 passive front wheel joints)
# Into:
# - /joint_states (8 total joints for robot_state_publisher)
```

**Why This Matters**:
- robot_state_publisher needs ALL joint states to compute TF tree
- ros2_control only publishes controlled joints (4 joints)
- Passive joints need separate publisher (4 front wheel joints)
- Merger combines them for complete TF tree (8 joints total)

## Problem Solving Journey

### Problem 1: Wheels Spinning Opposite Directions
**Symptom**: Left wheel forward, right wheel backward
**Root Cause**: Joint axis mismatch - both wheels had same axis direction
**Solution**: Inverted right wheel axis to `0 -1 0` in URDF

### Problem 2: Commands Not Reaching Gazebo
**Symptom**: Logs showed commands being sent, but wheels didn't respond
**Root Cause**: `gz` command not in PATH, subprocess calls silently failing
**Solution**: Use full path `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`

### Problem 3: Robot Not Moving Despite Wheels Spinning
**Symptom**: Rear wheels spinning, robot stationary
**Root Cause**: Two issues:
1. Ground plane had no friction (wheels couldn't grip)
2. Front wheels had JointController plugins acting as brakes

**Solution**:
1. Added friction to ground plane collision
2. Removed JointController plugins from front wheels entirely

### Problem 4: Controller Activation Failure
**Symptom**: Controllers stuck in INACTIVE state
**Root Cause**: Position-controlled joints missing velocity state interfaces
**Solution**: Added velocity state interfaces to all joints in URDF ros2_control block

## Testing Movement

### Basic Forward Drive Test:
```bash
# Send forward velocity command (0.1 m/s)
ros2 topic pub --rate 2 /tricycle_steering_controller/reference \
  geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.1}, angular: {z: 0.0}}}'
```

**Expected Behavior**:
- Both rear wheels spin forward at same speed
- Front wheels roll freely (not locked)
- Robot moves forward smoothly
- No skidding or sliding

### Steering Test:
```bash
# Turn right while moving forward
ros2 topic pub --rate 2 /tricycle_steering_controller/reference \
  geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.1}, angular: {z: -0.3}}}'
```

**Expected Behavior**:
- Rear wheels maintain forward motion with differential velocities
- Front steering joint rotates
- Front wheel steering joints follow (mimic)
- Robot turns right while moving

## Key Takeaways

1. **Topic-Based Architecture**: Using JointStateTopicSystem matches real robot's hardware bridge pattern
2. **Wheel Axes Matter**: Differential drive requires opposite axes for opposite-side wheels
3. **Freewheeling is Critical**: Front wheels MUST NOT have velocity controllers
4. **Friction Everywhere**: Ground, wheels, all surfaces need proper friction coefficients
5. **Command Path Issues**: Always use full paths for tools in subprocess calls
6. **State Interfaces**: Position-controlled joints still need velocity state interfaces for joint_state_broadcaster
7. **Open Loop for Sim**: Use open_loop mode when feedback is not accurate

## Docker Compose Services

All these components are orchestrated via docker-compose.yml:

- `jetacker-gazebo`: Gazebo simulation
- `jetacker-hardware-bridge`: Python bridge (ros2_control ↔ Gazebo)
- `jetacker-controller-manager`: ros2_control controller_manager
- `jetacker-controller-spawner`: Spawns and activates controllers
- `jetacker-passive-joint-publisher`: Publishes passive joint states
- `jetacker-joint-state-merger`: Merges controlled + passive joint states
- `jetacker-robot-state-publisher`: Computes TF tree from joint states

## Next Steps

With movement working, the next phases are:

- **Phase 3**: Add sensors (lidar, IMU)
- **Phase 4**: Integrate SLAM (slam_toolbox)
- **Phase 5**: Integrate Nav2 for autonomous navigation

## References

- ROS2 Control Documentation: https://control.ros.org/
- Gazebo Harmonic Documentation: https://gazebosim.org/
- Tricycle Controller: https://control.ros.org/master/doc/ros2_controllers/tricycle_steering_controller/doc/userdoc.html
