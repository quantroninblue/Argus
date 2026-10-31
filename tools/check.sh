#!/bin/sh
# Builds everything and runs the C++ and Python test suites. With ROS 2 Jazzy installed, its
# environment is used for the C++ build, which adds Argo's ROS 2 driver and its tests.
set -e
cd "$(dirname "$0")/.."
(
  if [ -f /opt/ros/jazzy/setup.sh ]; then
    . /opt/ros/jazzy/setup.sh
  fi
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
  cmake --build build -j"$(nproc)"
  ctest --test-dir build --output-on-failure
)
python3 -m pytest -q
