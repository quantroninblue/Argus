#!/bin/sh
# Builds the argo binary natively on the robot (Vicharak Axon, or any Ubuntu with ROS 2 Jazzy),
# without tests or Python bindings. Prints the apt packages to install if any are missing.
set -e
cd "$(dirname "$0")/.."
if [ ! -f /opt/ros/jazzy/setup.sh ]; then
  echo "ROS 2 Jazzy not found in /opt/ros/jazzy" >&2
  exit 1
fi
. /opt/ros/jazzy/setup.sh
missing=""
for p in cmake g++ libeigen3-dev libyaml-cpp-dev libprotobuf-dev protobuf-compiler libgrpc++-dev \
         protobuf-compiler-grpc; do
  dpkg -s "$p" >/dev/null 2>&1 || missing="$missing $p"
done
if [ -n "$missing" ]; then
  echo "missing packages:$missing" >&2
  echo "install them with: sudo apt install$missing" >&2
  exit 1
fi
build="${BUILD_DIR:-build/argo}"
cmake -S . -B "$build" -DCMAKE_BUILD_TYPE=Release -DARGUS_BUILD_TESTS=OFF -DARGUS_BUILD_PYTHON=OFF
cmake --build "$build" -j"$(nproc)" --target argo
echo "built $build/ros/argo_ros/argo"
