add_library(argus_capabilities
  src/geometry.cpp
  src/profile.cpp
  src/capabilities.cpp)
target_include_directories(argus_capabilities PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_capabilities PUBLIC Eigen3::Eigen PRIVATE yaml-cpp::yaml-cpp)
target_compile_options(argus_capabilities PRIVATE -Wall -Wextra -Wpedantic)

if(ARGUS_BUILD_TESTS)
  add_executable(argus_capabilities_tests
    tests/test_geometry.cpp
    tests/test_profile.cpp
    tests/test_capabilities.cpp)
  target_link_libraries(argus_capabilities_tests
    PRIVATE argus_capabilities yaml-cpp::yaml-cpp GTest::gtest_main)
  target_compile_definitions(argus_capabilities_tests
    PRIVATE ARGUS_EXAMPLES_DIR="${PROJECT_SOURCE_DIR}/schemas/examples")
  gtest_discover_tests(argus_capabilities_tests)
endif()

if(ARGUS_BUILD_PYTHON)
  pybind11_add_module(argus_capabilities_python src/python_bindings.cpp)
  target_link_libraries(argus_capabilities_python PRIVATE argus_capabilities)
  set_target_properties(argus_capabilities_python PROPERTIES
    OUTPUT_NAME capabilities
    LIBRARY_OUTPUT_DIRECTORY ${ARGUS_PYTHON_OUTPUT_DIR}/argus)
endif()
