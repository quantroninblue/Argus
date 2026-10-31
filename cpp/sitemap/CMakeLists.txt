add_library(argus_sitemap src/site_map.cpp src/route.cpp)
target_include_directories(argus_sitemap PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_sitemap PUBLIC argus_grid argus_capabilities)
target_compile_options(argus_sitemap PRIVATE -Wall -Wextra -Wpedantic)

if(ARGUS_BUILD_TESTS)
  add_executable(argus_sitemap_tests tests/test_site_map.cpp)
  target_link_libraries(argus_sitemap_tests PRIVATE argus_sitemap argus_node GTest::gtest_main)
  gtest_discover_tests(argus_sitemap_tests)
endif()

if(ARGUS_BUILD_PYTHON)
  pybind11_add_module(argus_sitemap_python src/python_bindings.cpp)
  target_link_libraries(argus_sitemap_python PRIVATE argus_sitemap)
  set_target_properties(argus_sitemap_python PROPERTIES
    OUTPUT_NAME sitemap
    LIBRARY_OUTPUT_DIRECTORY ${ARGUS_PYTHON_OUTPUT_DIR}/argus)
endif()
