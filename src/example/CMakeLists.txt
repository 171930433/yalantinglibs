
set(ENV{proto_message} "proto_message")

# 设置编译器选项
add_compile_options(-Wno-attributes)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/examples)

# protobuf_generate_struct_pb 函数
set(struct_pb_plugin_DIRS ${CMAKE_SOURCE_DIR}/src/protoc-plugin)  # struct_pb_plugin_DIRS 将指定 proto_to_struct.proto,struct_pb.cmake等目录
set(Protobuf_IMPORT_DIRS ${CMAKE_SOURCE_DIR} ${struct_pb_plugin_DIRS} ${CMAKE_CURRENT_LIST_DIR})     # 设置protoc --proto_path参数
include(${struct_pb_plugin_DIRS}/cmake/struct_pb.cmake)

find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED COMPONENTS core)


file(GLOB PROTO_FILES "*.proto")

# generate .pb.cc .pb.h
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
  ${struct_pb_plugin_DIRS}/proto_to_struct.proto
  ${PROTO_FILES}
)

protobuf_generate_python(PROTO_PYTHON_SRCS
  ${struct_pb_plugin_DIRS}/proto_to_struct.proto
  ${PROTO_FILES}
)

protobuf_generate_struct_pb(PROTO_STRUCT_SRCS PROTO_STRUCT_HDRS
  ${PROTO_FILES}
OPTION "namespace=zhito.inner_struct,converter_namespace=zhito.converter"
# PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}
)


add_library(proto_common OBJECT
  ${PROTO_SRCS}
  ${PROTO_HDRS}
  ${PROTO_STRUCT_SRCS}
  ${PROTO_STRUCT_HDRS}
  ${PROTO_PYTHON_SRCS}
)

target_link_libraries(proto_common protobuf::libprotobuf )

target_include_directories(proto_common PUBLIC 
  ${CMAKE_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_LIST_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}/src/protoc-plugin
)


add_executable(struct_pb_tutorial tutorial.cpp )

target_link_libraries(struct_pb_tutorial 
  proto_common
  $ENV{proto_message}
)

target_include_directories(struct_pb_tutorial PUBLIC 
  ${CMAKE_BINARY_DIR}
  ${CMAKE_SOURCE_DIR}
)


add_subdirectory(message)
