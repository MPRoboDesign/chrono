#!/usr/bin/env bash
# One-time system-wide prerequisites for building all 5 chrono_ws Chrono modules
# (Irrlicht, Vehicle, ROS, Parsers, Sensor) on Ubuntu 22.04 (jammy).
# Everything else (Irrlicht, urdfdom stack, GLM/GLEW/GLFW) is built from source
# into module_*/ folders by the scripts in each of those folders - this script
# only covers what genuinely has to live at the system level: compilers/build
# tools, X11/OpenGL headers needed to compile against the system's graphics
# stack, ROS 2 itself, and the CUDA toolkit.
#
# Run this yourself: bash install_system_prerequisites.sh
set -euo pipefail

echo "=== Build tools (cmake, ninja, compilers) ==="
sudo apt-get update
sudo apt-get install -y build-essential cmake ninja-build git wget unzip curl

echo "=== X11 / OpenGL dev headers (needed to build Irrlicht and GLFW) ==="
sudo apt-get install -y \
    libgl1-mesa-dev libxxf86vm-dev libxcursor-dev libx11-dev \
    libxrandr-dev libxinerama-dev libxi-dev libxext-dev

echo "=== CUDA Toolkit (distro package; driver already present) ==="
sudo apt-get install -y nvidia-cuda-toolkit

echo "=== ROS 2 Humble ==="
bash "$(dirname "$0")/module_ros/install_ros2_humble.sh"

echo "All system prerequisites installed."
echo "Next: run module_irrlicht/build_irrlicht.sh, module_parsers/build_urdf_deps.sh, and module_sensor/build_gl_deps.sh"
