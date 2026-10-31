#!/bin/sh
# Builds everything and runs the C++ and Python test suites. With ROS 2 Jazzy installed, its
# environment is used for the C++ build, which adds Argo's ROS 2 driver and its tests.
# --gazebo also runs the Gazebo episodes (tests/test_gazebo.py, a few minutes), which need ROS 2.
set -e
cd "$(dirname "$0")/.."
gazebo=0
for arg in "$@"; do
  case "$arg" in
    --gazebo) gazebo=1 ;;
    *) echo "usage: tools/check.sh [--gazebo]" >&2; exit 2 ;;
  esac
done
(
  if [ -f /opt/ros/jazzy/setup.sh ]; then
    . /opt/ros/jazzy/setup.sh
  fi
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
  cmake --build build -j"$(nproc)"
  ctest --test-dir build --output-on-failure
)
python3 -m pytest -q
if [ "$gazebo" = 1 ]; then
  if [ ! -f /opt/ros/jazzy/setup.sh ]; then
    echo "--gazebo needs ROS 2 Jazzy" >&2
    exit 1
  fi
  (. /opt/ros/jazzy/setup.sh && python3 -m pytest -q -rxX tests/test_gazebo.py)
fi
