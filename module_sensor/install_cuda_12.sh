#!/usr/bin/env bash
# Installs CUDA Toolkit 12.8 from NVIDIA's official apt repo, alongside the
# distro's existing CUDA 11.5 (nvidia-cuda-toolkit). Needed because 11.5's
# nvcc cannot target Blackwell GPUs (compute capability 12.0, e.g. RTX 50
# series) required for Chrono::Sensor's OptiX-based ray-traced camera/lidar.
# Installs to /usr/local/cuda-12.8 (versioned path), does not remove 11.5.
#
# Run this yourself: bash install_cuda_12.sh
set -euo pipefail

cd /tmp
wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb -O cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb

sudo apt-get update
sudo apt-get install -y cuda-toolkit-12-8

echo "CUDA 12.8 installed to /usr/local/cuda-12.8"
echo "nvcc: /usr/local/cuda-12.8/bin/nvcc"
/usr/local/cuda-12.8/bin/nvcc --version
