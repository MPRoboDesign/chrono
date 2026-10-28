#!/usr/bin/env bash
# Installs ROS 2 Humble system-wide on Ubuntu 22.04 (jammy).
# Run this yourself with sudo privileges: bash install_ros2_humble.sh
set -euo pipefail

sudo apt-get update
sudo apt-get install -y software-properties-common curl
sudo add-apt-repository -y universe

ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt-get install -y /tmp/ros2-apt-source.deb

sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y ros-humble-desktop ros-dev-tools

echo "ROS 2 Humble installed. Source it with: source /opt/ros/humble/setup.bash"
