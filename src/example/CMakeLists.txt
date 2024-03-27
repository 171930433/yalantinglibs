# 设置编译器选项
add_compile_options(-Wno-attributes)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/examples)

# protobuf_generate_struct_pb 函数
set(struct_pb_plugin_DIRS ${CMAKE_SOURCE_DIR}/src/protoc_plugin)  # struct_pb_plugin_DIRS 将指定 proto_to_struct.proto,struct_pb.cmake等目录
include(${struct_pb_plugin_DIRS}/cmake/struct_pb.cmake)

find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED COMPONENTS core)

file(GLOB_RECURSE PROTO_FILES "*.proto")

protobuf_generate_modified(
  PROTO_PATH_FROM_CMAKE_SOURCE
  LANGUAGE cpp
  OUT_VAR PROTO_CLASS_SRCS
  PROTOS ${struct_pb_plugin_DIRS}/proto_to_struct.proto  ${PROTO_FILES}
)

protobuf_generate_modified(
  PROTO_PATH_FROM_CMAKE_SOURCE
  LANGUAGE python
  OUT_VAR PROTO_PYTHON_SRCS
  PROTOS ${struct_pb_plugin_DIRS}/proto_to_struct.proto  ${PROTO_FILES}
)

list(FILTER PROTO_FILES EXCLUDE REGEX "proto_to_struct\\.proto$|image\\.proto$|pointcloud\\.proto$")


protobuf_generate_modified(
  PROTO_PATH_FROM_CMAKE_SOURCE
  LANGUAGE struct_pb
  # PLUGIN "struct_pb"  # 这么写,protoc-gen-struct_pb需要在环境变量中
  PLUGIN "${CMAKE_BINARY_DIR}/output/bin/protoc-gen-struct_pb"
  OUT_VAR PROTO_STRUCT_SRCS
  PROTOS  ${PROTO_FILES}
  PROTOC_OPTION "namespace=zhito.inner_struct,converter_namespace=zhito.converter"
)


add_library(proto_common OBJECT
  message/zpointcloud.cpp   message/zimage.cpp
  ${PROTO_CLASS_SRCS}
  ${PROTO_STRUCT_SRCS}
  ${PROTO_PYTHON_SRCS}
)

target_link_libraries(proto_common
PUBLIC
  protobuf::libprotobuf
  # 第三方库
  pcl_common pcl_io # pcl
  opencv_imgcodecs opencv_core  # opencv
)

target_include_directories(proto_common PUBLIC 
  ${CMAKE_BINARY_DIR} 
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/ # 找到iguana
)


add_executable(struct_pb_tutorial tutorial.cpp )

target_link_libraries(struct_pb_tutorial 
  proto_common
)

target_include_directories(struct_pb_tutorial PUBLIC 
  ${CMAKE_BINARY_DIR}
  ${CMAKE_SOURCE_DIR}
)
