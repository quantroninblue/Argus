#!/bin/sh
# Builds everything and runs the C++ and Python test suites.
set -e
cd "$(dirname "$0")/.."
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
python3 -m pytest -q
