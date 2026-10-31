add_library(argus_node
  src/types.cpp
  src/ray_pattern.cpp
  src/local_map.cpp
  src/body.cpp
  src/motion.cpp
  src/workspace.cpp
  src/mission.cpp
  src/safety.cpp
  src/planner.cpp
  src/adapter.cpp
  src/node.cpp)
target_include_directories(argus_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_node PUBLIC argus_capabilities)
target_compile_options(argus_node PRIVATE -Wall -Wextra -Wpedantic)

if(ARGUS_BUILD_TESTS)
  add_executable(argus_node_tests
    tests/test_local_map.cpp
    tests/test_motion.cpp
    tests/test_safety.cpp
    tests/test_node.cpp)
  target_link_libraries(argus_node_tests PRIVATE argus_node GTest::gtest_main)
  target_compile_definitions(argus_node_tests
    PRIVATE ARGUS_EXAMPLES_DIR="${PROJECT_SOURCE_DIR}/schemas/examples")
  gtest_discover_tests(argus_node_tests)
endif()

if(ARGUS_BUILD_PYTHON)
  pybind11_add_module(argus_node_python src/python_bindings.cpp)
  target_link_libraries(argus_node_python PRIVATE argus_node)
  set_target_properties(argus_node_python PROPERTIES
    OUTPUT_NAME node
    LIBRARY_OUTPUT_DIRECTORY ${ARGUS_PYTHON_OUTPUT_DIR}/argus)
endif()
