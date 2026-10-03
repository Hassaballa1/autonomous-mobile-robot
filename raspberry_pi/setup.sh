#!/usr/bin/env bash
# One-time setup of a Raspberry Pi 4/5 running Ubuntu Server 24.04 (64-bit) for the robot.
#   git clone https://github.com/Eslamhabashy1/autonomous-mobile-robot.git ~/amr
#   cd ~/amr/raspberry_pi && ./setup.sh
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
WS="$REPO_DIR/ros2_ws"

echo "==> ROS 2 Jazzy apt repository"
sudo apt-get update
sudo apt-get install -y curl software-properties-common
sudo add-apt-repository -y universe
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | awk -F'"' '{print $4}')
CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
curl -fsSL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt-get update

echo "==> ROS 2 packages (headless: no Gazebo or RViz on the robot)"
sudo apt-get install -y \
  ros-jazzy-ros-base \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rplidar-ros \
  ros-jazzy-v4l2-camera \
  ros-jazzy-cv-bridge \
  ros-jazzy-vision-msgs \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-teleop-twist-keyboard \
  python3-serial \
  python3-colcon-common-extensions \
  python3-pip

echo "==> YOLOv8 (Ultralytics, CPU-only PyTorch) for the system Python that ROS uses"
pip3 install --break-system-packages --index-url https://download.pytorch.org/whl/cpu torch torchvision
pip3 install --break-system-packages -r "$WS/src/amr_perception/requirements.txt"

echo "==> Serial and camera access"
sudo usermod -aG dialout,video "$USER"
sudo cp "$REPO_DIR/raspberry_pi/99-amr.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "==> Build the workspace (amr_sim is skipped: it needs Gazebo)"
cd "$WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-skip amr_sim

echo "==> Start on boot"
sed -e "s|__USER__|$USER|" -e "s|__WS__|$WS|" "$REPO_DIR/raspberry_pi/amr.service" \
  | sudo tee /etc/systemd/system/amr.service > /dev/null
sudo systemctl daemon-reload
sudo systemctl enable amr.service

echo
echo "Done. Log out and back in (for the dialout/video groups), then:"
echo "  sudo systemctl start amr      # or run by hand:"
echo "  source $WS/install/setup.bash && ros2 launch amr_bringup slam.launch.py"
