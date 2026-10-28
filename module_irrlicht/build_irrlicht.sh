#!/usr/bin/env bash
# Builds Irrlicht Engine 1.8.5 (already downloaded/extracted into ./irrlicht-1.8.5)
# Requires: g++, make, libgl1-mesa-dev, libxxf86vm-dev, libxcursor-dev, libx11-dev
# (installed by ../install_system_prerequisites.sh)
set -euo pipefail
cd "$(dirname "$0")/irrlicht-1.8.5/source/Irrlicht"
make sharedlib -j"$(nproc)"
echo "Irrlicht built. Set Irrlicht_ROOT to: $(cd "$(dirname "$0")/irrlicht-1.8.5" && pwd)"
