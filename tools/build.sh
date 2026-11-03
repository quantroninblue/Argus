#!/bin/sh
# Builds Argus into build/. With ROS 2 Jazzy installed, this includes Argo's ROS 2 driver.
set -e
cd "$(dirname "$0")/.."
if [ -f /opt/ros/jazzy/setup.sh ]; then
  . /opt/ros/jazzy/setup.sh
fi
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DARGUS_BUILD_TESTS=OFF
cmake --build build -j"$(nproc)"
echo "Built. Put the argus command on your PATH with:"
echo "  mkdir -p ~/.local/bin && ln -s \"\$PWD/tools/argus\" ~/.local/bin/argus"
