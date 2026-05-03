# ROS2 Gazebo Simulation with Nav2

Tricycle-drive (Ackermann) robot simulation using ROS2 Jazzy, Gazebo Harmonic, and Nav2. Everything runs in Docker containers — no ROS2 installation needed on the host.

## Prerequisites

- **Docker Desktop** (WSL2 backend on Windows, or native on macOS/Linux)
- **16 GB RAM minimum** — Gazebo + Nav2 runs ~20 containers simultaneously
- **~10 GB disk space** for Docker images
- **Python 3** with `pyyaml` (`pip install pyyaml`)
- **Foxglove Studio** ([download](https://foxglove.dev/download)) — for visualization and teleop (optional but recommended)

## Build (one-time)

All images are built locally — there are no pre-built images to pull.

```bash
docker compose build
```

This builds 7 images in dependency order (~15 min first time):

```
ubuntu:24.04 ─┬─ ros2-jazzy-base ─┬─ ros2-jazzy-gazebo ─┬─ ros2-jazzy-overlay
              │                   │                     └─ ros2-jazzy-nav2
              │                   ├─ ros2-jazzy-debug
              │                   └─ ros2-jazzy-foxglove
              └─ x11-server
```

The overlay image compiles a patched `robot_localization` from source (fixes a deadlock bug with `use_sim_time`).

## Start Infrastructure

Infrastructure must be started **before** any robot stack. These services persist across stack restarts — start them once per session.

```bash
docker compose up -d debug x11-server foxglove
```

**Why these three?**
- **debug** — provides the shared network namespace for all services (ROS2 DDS multicast requires it). Also a convenience shell with all workspace mounts and ROS2 CLI tools — `docker compose exec debug bash` to introspect topics, TFs, etc.
- **x11-server** — provides display `:99` for Gazebo rendering (without it, Gazebo crashes with a Qt5 fatal error)
- **foxglove** — WebSocket bridge for Foxglove Studio visualization

## Stage 1: Teleop (verify the simulation works)

```bash
python stack.py start jetacker:base
```

This starts ~11 services: Gazebo, clock bridge, robot state publisher, hardware bridge, controllers, EKF, and more. Wait for all healthchecks to pass (~30s).

**View Gazebo GUI:** open http://localhost:6080 in a browser (noVNC).

**Connect Foxglove Studio:** open a new connection to `ws://localhost:8765`.

**Drive the robot:** use the Teleop panel in Foxglove, or manually publish a `TwistStamped` to `/tricycle_steering_controller/reference`:

```json
{"twist": {"linear": {"x": 0.2}, "angular": {"z": 0.5}}}
```

You should see the robot moving in both the Gazebo GUI and Foxglove's 3D panel.

## Stage 2: SLAM (verify driving and mapping)

```bash
python stack.py force-clean
python stack.py start jetacker:slam
```

Drive the robot around with teleop and watch the `/map` topic in Foxglove to see the occupancy grid being built in real time. You can also use the automated test drive script:

```bash
python drive_simple.py --cycles 1
```

A pre-built map for the default gym world is included in `maps/` — you don't need to save your own.

## Stage 3: Nav2 (autonomous navigation)

Nav2 requires a pre-built map. The default map is configured in `docker-compose.yml` (`jetacker-map-server` → `yaml_filename`). To use a different map, update that path.

```bash
python stack.py force-clean
python stack.py start jetacker:nav2
```

**Run a waypoint test:**

```bash
python drive_nav2.py --waypoints nav2_matrix_1_forward_straight --cycles 1 --timeout 60
```

See `driving_instructions/` for available waypoint files.

## Stopping / Switching Stacks

Stacks conflict with each other — always `force-clean` before switching:

```bash
python stack.py force-clean              # Stops all robot services (preserves infra)
python stack.py start jetacker:slam      # Start a different stack
python stack.py status                   # Show what's running
```

## Stack Reference

All stack topology is defined in `stacks.yaml` and managed by `stack.py`.

| Target | Purpose |
|--------|---------|
| `jetacker:base` | Core robot: Gazebo, controllers, EKF, joint state processing |
| `jetacker:slam` | Base + SLAM Toolbox for building maps |
| `jetacker:nav2` | Base + Nav2 with AMCL localization (requires map) |
| `jetacker:nav2_odom` | Base + Nav2 without AMCL (odometry-only testing) |
| `slam_bot:base` | Differential drive robot simulation |
| `slam_bot:slam` | Differential drive + SLAM |

## Test Drives

**Odometry-based** (proportional control, no Nav2):

```bash
python drive_simple.py --cycles 1
python drive_simple.py --instruction test_square --cycles 5 --record
```

**Nav2 waypoints** (autonomous navigation):

```bash
python drive_nav2.py --waypoints gym_loop_nav2 --cycles 1 --timeout 60
python drive_nav2.py --waypoints nav2_matrix_1_forward_straight --cycles 1 --no-amcl
```

Instruction and waypoint YAML files live in `driving_instructions/`.

## Troubleshooting

**Gazebo crashes immediately (Qt5 fatal error)**
x11-server is not running. Start it first: `docker compose up -d x11-server`

**Gazebo receives SIGINT on startup / containers exit immediately**
Race condition when starting all services at once. Use `stack.py` which handles startup ordering, or start `jetacker-gazebo` first and wait for it to be healthy before starting remaining services.

**Controller spawner fails "switch controller timed out"**
Gazebo wasn't healthy yet when the controller spawner ran. Restart the spawner after Gazebo is healthy: `docker compose restart jetacker-controller-spawner`

**"dependency failed to start: container exited (0)"**
A dependency (usually Gazebo) died before its healthcheck passed. Check x11-server is running, then check `docker compose logs jetacker-gazebo`.

**Topics visible but no data flowing**
DDS multicast issue. Verify all services use `network_mode: service:debug`.

**Debug shell**
```bash
docker compose exec debug bash
# Inside: ros2 topic list, ros2 node list, ros2 topic echo /clock, etc.
```

## Architecture

- **One `ros2 run` per container** — no launch files, explicit composition via docker-compose
- **Shared network namespace** — all services use `network_mode: service:debug` for DDS discovery
- **Hardware bridge** — topic-based `JointStateTopicSystem` mimicking the real robot's STM32 serial bridge
- **Controller** — `tricycle_steering_controller` with closed-loop velocity feedback
- **EKF sensor fusion** — `robot_localization` fuses wheel odometry + IMU → `/odom`
- **Reset orchestrator** — DAG-based declarative reset via `tools/reset_orchestrator.py`
