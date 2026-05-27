#!/usr/bin/env bash
set -euo pipefail

sudo apt-get update
sudo apt-get install -y \
  git \
  cmake \
  build-essential \
  libssl-dev \
  libusb-1.0-0-dev \
  libgtk-3-dev \
  pkg-config \
  libglfw3-dev \
  libgl1-mesa-dev \
  libglu1-mesa-dev \
  python3-dev \
  python3-pip \
  python3-numpy

WORKDIR="${HOME}/librealsense-build"
rm -rf "${WORKDIR}"
git clone https://github.com/IntelRealSense/librealsense.git "${WORKDIR}"
cd "${WORKDIR}"
mkdir build
cd build
cmake .. \
  -DBUILD_PYTHON_BINDINGS=bool:true \
  -DPYTHON_EXECUTABLE="$(command -v python3)" \
  -DBUILD_EXAMPLES=false \
  -DBUILD_GRAPHICAL_EXAMPLES=false
make -j"$(nproc)"
sudo make install
python3 -m pip install pyrealsense2
