# Nav2 MPPI controller with per-critic cost logging + custom cusp/ghost extensions
#
# Builds from workdir/ref/navigation2/ git clone (branch pr6, based on jazzy).
# Only builds the two modified packages; everything else stays apt-installed.
#
# Source of truth: workdir/ref/navigation2/
#   - nav2_critics_msgs (our custom message package)
#   - nav2_mppi_controller (cusp extension, debug logging, critic stats)
#
# Toggle via COMPOSE_FILE in .env:
#   COMPOSE_FILE=docker-compose.core.yml;docker-compose.sim.yml;docker-compose.critics.yml

ARG ROS_DISTRO=jazzy
FROM ros2-${ROS_DISTRO}-nav2:latest

ENV ROS_DISTRO=${ROS_DISTRO}

# Install build tools (not present in nav2 runtime image)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Copy ONLY the two modified packages from the navigation2 git clone
COPY workdir/ref/navigation2/nav2_critics_msgs /build_ws/src/nav2_critics_msgs
COPY workdir/ref/navigation2/nav2_mppi_controller /build_ws/src/nav2_mppi_controller

# Build into /workspace/install/ — auto-sourced by base entrypoint
# nav2_critics_msgs must build first (nav2_mppi_controller depends on it)
RUN /bin/bash -c "\
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /build_ws && \
    colcon build \
        --install-base /workspace/install \
        --build-base /build_ws/build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --packages-select nav2_critics_msgs && \
    source /workspace/install/setup.bash && \
    colcon build \
        --install-base /workspace/install \
        --build-base /build_ws/build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --packages-select nav2_mppi_controller \
        --allow-overriding nav2_mppi_controller && \
    rm -rf /build_ws/build /build_ws/src"
