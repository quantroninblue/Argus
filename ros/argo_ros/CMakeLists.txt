# Argo's ROS 2 driver and the argo executable. The only code that includes ROS headers.
# Built when a ROS 2 environment is sourced (tools/check.sh sources Jazzy if it is installed).
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(argo_convert src/convert.cpp)
target_include_directories(argo_convert PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argo_convert PUBLIC argus_node)
target_compile_options(argo_convert PRIVATE -Wall -Wextra -Wpedantic)

add_library(argo_ros src/ros_driver.cpp)
target_include_directories(argo_ros PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argo_ros PUBLIC argo_convert argo_runtime rclcpp::rclcpp
  ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS})
target_compile_options(argo_ros PRIVATE -Wall -Wextra)

add_executable(argo src/main.cpp)
target_link_libraries(argo PRIVATE argo_ros ${std_msgs_TARGETS})

if(ARGUS_BUILD_TESTS)
  add_executable(argo_ros_tests tests/test_convert.cpp tests/test_ros_driver.cpp)
  target_link_libraries(argo_ros_tests PRIVATE argo_ros GTest::gtest_main)
  target_compile_definitions(argo_ros_tests
    PRIVATE ARGUS_EXAMPLES_DIR="${PROJECT_SOURCE_DIR}/schemas/examples")
  gtest_discover_tests(argo_ros_tests)
endif()
