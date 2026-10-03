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

# YOLOv8: CPU-only PyTorch, then Ultralytics with versions that keep ROS 2 Jazzy working
COPY ros2_ws/src/amr_perception/requirements.txt /tmp/requirements.txt
RUN pip3 install --break-system-packages --no-cache-dir \
      --index-url https://download.pytorch.org/whl/cpu torch torchvision \
    && pip3 install --break-system-packages --no-cache-dir -r /tmp/requirements.txt
