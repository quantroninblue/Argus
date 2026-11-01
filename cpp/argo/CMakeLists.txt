# Argo's runtime on a robot: the core and its hub session on a fixed-rate control thread, a
# driver interface, and the config file. Drivers (ROS 2) live elsewhere and link this.
add_library(argo_runtime src/runtime.cpp src/config.cpp)
target_include_directories(argo_runtime PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argo_runtime PUBLIC argus_node_session argus_node_grpc yaml-cpp::yaml-cpp)
target_compile_options(argo_runtime PRIVATE -Wall -Wextra -Wpedantic)

# Timing of the core on this machine against its budgets (see src/bench.cpp).
add_executable(argo_bench src/bench.cpp)
target_link_libraries(argo_bench PRIVATE argus_node)
target_compile_options(argo_bench PRIVATE -Wall -Wextra -Wpedantic)

if(ARGUS_BUILD_TESTS)
  add_executable(argo_runtime_tests tests/test_runtime.cpp)
  target_link_libraries(argo_runtime_tests PRIVATE argo_runtime GTest::gtest_main)
  target_compile_definitions(argo_runtime_tests
    PRIVATE ARGUS_EXAMPLES_DIR="${PROJECT_SOURCE_DIR}/schemas/examples")
  gtest_discover_tests(argo_runtime_tests)
endif()
