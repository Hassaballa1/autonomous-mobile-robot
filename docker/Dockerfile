# Development environment: ROS 2 Jazzy desktop (Gazebo, RViz) in the browser via noVNC.
# Multi-arch, so it runs natively on Apple Silicon as well as x86.
FROM tiryoh/ros2-desktop-vnc:jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz \
      ros-jazzy-slam-toolbox \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-rplidar-ros \
      ros-jazzy-v4l2-camera \
      ros-jazzy-vision-msgs \
      ros-jazzy-cv-bridge \
      ros-jazzy-xacro \
      ros-jazzy-teleop-twist-keyboard \
      python3-serial \
      python3-pytest \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

COPY ros2_ws/src/amr_perception/requirements.txt /tmp/requirements.txt
RUN pip3 install --break-system-packages -r /tmp/requirements.txt
