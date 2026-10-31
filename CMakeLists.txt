cmake_minimum_required(VERSION 3.22)
project(argus VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(ARGUS_BUILD_TESTS "Build the C++ tests" ON)
option(ARGUS_BUILD_PYTHON "Build the Python bindings and generated protocol modules" ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)

# Python modules built here (bindings, generated protocol code) land in this directory.
set(ARGUS_PYTHON_OUTPUT_DIR ${CMAKE_BINARY_DIR}/python)

if(ARGUS_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  include(GoogleTest)
endif()

if(ARGUS_BUILD_PYTHON)
  find_package(Python 3.10 COMPONENTS Interpreter Development.Module REQUIRED)
  find_package(pybind11 CONFIG REQUIRED)
endif()

add_subdirectory(proto)
add_subdirectory(cpp/grid)
add_subdirectory(cpp/capabilities)
add_subdirectory(cpp/node)
add_subdirectory(cpp/sitemap)
add_subdirectory(cpp/argo)

# Argo's ROS 2 driver and the argo executable, when a ROS 2 environment is sourced.
find_package(rclcpp QUIET)
if(rclcpp_FOUND)
  add_subdirectory(ros/argo_ros)
endif()
