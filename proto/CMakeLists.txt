file(GLOB ARGUS_PROTO_FILES CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/argus/protocol/v1/*.proto)

add_library(argus_protocol ${ARGUS_PROTO_FILES})
target_link_libraries(argus_protocol PUBLIC protobuf::libprotobuf)
target_include_directories(argus_protocol PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate(
  TARGET argus_protocol
  LANGUAGE cpp
  IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
  PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})

if(ARGUS_BUILD_PYTHON)
  set(py_outputs)
  foreach(proto_file ${ARGUS_PROTO_FILES})
    file(RELATIVE_PATH rel ${CMAKE_CURRENT_SOURCE_DIR} ${proto_file})
    string(REGEX REPLACE "\\.proto$" "_pb2.py" py_file ${rel})
    list(APPEND py_outputs ${ARGUS_PYTHON_OUTPUT_DIR}/${py_file})
  endforeach()
  add_custom_command(
    OUTPUT ${py_outputs}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${ARGUS_PYTHON_OUTPUT_DIR}
    COMMAND protobuf::protoc -I ${CMAKE_CURRENT_SOURCE_DIR}
            --python_out=${ARGUS_PYTHON_OUTPUT_DIR} ${ARGUS_PROTO_FILES}
    DEPENDS ${ARGUS_PROTO_FILES}
    COMMENT "Generating Python protocol modules")
  add_custom_target(argus_protocol_python ALL DEPENDS ${py_outputs})
endif()

if(ARGUS_BUILD_TESTS)
  add_executable(argus_protocol_tests tests/test_protocol.cpp)
  target_link_libraries(argus_protocol_tests PRIVATE argus_protocol GTest::gtest_main)
  gtest_discover_tests(argus_protocol_tests)
endif()
