# ROS2 + Gazebo Sim
ARG ROS_DISTRO=jazzy
FROM ros2-${ROS_DISTRO}-base:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

# Gazebo Sim and ROS-Gazebo bridge
# ros2_control: Hardware abstraction and controller framework
# Mesa drivers for software rendering (OpenGL through X11)
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-gz-sim-vendor \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-tricycle-steering-controller \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-joint-state-topic-hardware-interface \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-imu-filter-madgwick \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-xacro \
    libgl1 \
    libglx-mesa0 \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

ENV DISPLAY=:99

WORKDIR /workspace

CMD ["gz", "sim", "-v", "4"]
