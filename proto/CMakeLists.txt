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

# Service stubs. HubLink (link.proto) has C++ and Python sides; HubApp (app.proto) is served
# and called from Python only.
set(ARGUS_APP_PROTO ${CMAKE_CURRENT_SOURCE_DIR}/argus/protocol/v1/app.proto)
set(ARGUS_SERVICE_PROTO ${CMAKE_CURRENT_SOURCE_DIR}/argus/protocol/v1/link.proto)
add_library(argus_protocol_grpc ${ARGUS_SERVICE_PROTO})
target_link_libraries(argus_protocol_grpc PUBLIC argus_protocol gRPC::grpc++)
protobuf_generate(
  TARGET argus_protocol_grpc
  LANGUAGE grpc
  GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc
  PLUGIN protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>
  IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
  PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})

if(ARGUS_BUILD_PYTHON)
  set(py_outputs)
  foreach(proto_file ${ARGUS_PROTO_FILES})
    file(RELATIVE_PATH rel ${CMAKE_CURRENT_SOURCE_DIR} ${proto_file})
    string(REGEX REPLACE "\\.proto$" "_pb2.py" py_file ${rel})
    list(APPEND py_outputs ${ARGUS_PYTHON_OUTPUT_DIR}/${py_file})
  endforeach()
  list(APPEND py_outputs ${ARGUS_PYTHON_OUTPUT_DIR}/argus/protocol/v1/link_pb2_grpc.py
                         ${ARGUS_PYTHON_OUTPUT_DIR}/argus/protocol/v1/app_pb2_grpc.py)
  add_custom_command(
    OUTPUT ${py_outputs}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${ARGUS_PYTHON_OUTPUT_DIR}
    COMMAND protobuf::protoc -I ${CMAKE_CURRENT_SOURCE_DIR}
            --python_out=${ARGUS_PYTHON_OUTPUT_DIR} ${ARGUS_PROTO_FILES}
    COMMAND protobuf::protoc -I ${CMAKE_CURRENT_SOURCE_DIR}
            --plugin=protoc-gen-grpc_python=$<TARGET_FILE:gRPC::grpc_python_plugin>
            --grpc_python_out=${ARGUS_PYTHON_OUTPUT_DIR} ${ARGUS_SERVICE_PROTO} ${ARGUS_APP_PROTO}
    DEPENDS ${ARGUS_PROTO_FILES}
    COMMENT "Generating Python protocol modules")
  add_custom_target(argus_protocol_python ALL DEPENDS ${py_outputs})
endif()

if(ARGUS_BUILD_TESTS)
  add_executable(argus_protocol_tests tests/test_protocol.cpp)
  target_link_libraries(argus_protocol_tests PRIVATE argus_protocol_grpc GTest::gtest_main)
  gtest_discover_tests(argus_protocol_tests)

  add_executable(argus_link_grpc_tests tests/test_link_grpc.cpp)
  target_link_libraries(argus_link_grpc_tests PRIVATE argus_protocol_grpc GTest::gtest_main)
  gtest_discover_tests(argus_link_grpc_tests)
endif()
