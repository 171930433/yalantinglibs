cmake_minimum_required(VERSION 3.10)

project(protoc-gen-struct_pb)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/bin)

# find_package(Protobuf REQUIRED)
file(GLOB SRCS "*.cpp" "*.cc" "*.h")

include(cmake/struct_pb.cmake)
protobuf_generate_modified(
  PROTO_PATH_FROM_CMAKE_SOURCE
  LANGUAGE cpp
  OUT_VAR PROTO_CLASS_SRCS
  # IMPORT_DIRS ${CMAKE_SOURCE_DIR}
  PROTOS proto_to_struct.proto
  # PROTOC_OUT_DIR ${CMAKE_BINARY_DIR}
)


add_executable(protoc-gen-struct_pb ${SRCS} ${PROTO_CLASS_SRCS})
target_include_directories(protoc-gen-struct_pb PUBLIC ${CMAKE_BINARY_DIR})
target_link_libraries(protoc-gen-struct_pb PUBLIC protobuf::libprotoc protobuf::libprotobuf)
install(
    TARGETS protoc-gen-struct_pb
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)


