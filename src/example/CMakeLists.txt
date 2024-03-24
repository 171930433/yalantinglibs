
set(ENV{proto_message} "proto_message")
set(ENV{struct_pb_options} "namespace=zhito.inner_struct,converter_namespace=zhito.converter")

# 设置编译器选项
add_compile_options(-Wno-attributes)

set(protobuf_MODULE_COMPATIBLE ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/examples)

# protobuf_generate_struct_pb 函数
set(struct_pb_plugin_DIRS ${CMAKE_SOURCE_DIR}/src/protoc-plugin)  # struct_pb_plugin_DIRS 将指定 proto_to_struct.proto,struct_pb.cmake等目录
# set(Protobuf_IMPORT_DIRS  ${struct_pb_plugin_DIRS} ${CMAKE_CURRENT_LIST_DIR})     # 设置protoc --proto_path参数
set(Protobuf_IMPORT_DIRS  ${CMAKE_SOURCE_DIR})     # 设置protoc --proto_path参数
include(${struct_pb_plugin_DIRS}/cmake/struct_pb.cmake)

# find_package(PCL REQUIRED COMPONENTS common io)
# find_package(OpenCV REQUIRED COMPONENTS core)


file(GLOB_RECURSE PROTO_FILES "*.proto")
# set(PROTO_FILES 
#    "/home/gsk/pro/yalantinglibs/src/example/base.proto"
#    "/home/gsk/pro/yalantinglibs/src/example/message/zframe.proto" 
#    "/home/gsk/pro/yalantinglibs/src/example/message/imu.proto" )


protobuf_generate(
  LANGUAGE cpp
  OUT_VAR PROTO_CLASS_SRCS
  IMPORT_DIRS ${CMAKE_SOURCE_DIR}
  PROTOS ${struct_pb_plugin_DIRS}/proto_to_struct.proto  ${PROTO_FILES}
  PROTOC_OUT_DIR ${CMAKE_BINARY_DIR}
)


protobuf_generate(
  LANGUAGE python
  OUT_VAR PROTO_PYTHON_SRCS
  IMPORT_DIRS ${CMAKE_SOURCE_DIR}
  PROTOS ${struct_pb_plugin_DIRS}/proto_to_struct.proto  ${PROTO_FILES}
  PROTOC_OUT_DIR ${CMAKE_BINARY_DIR}
)

list(FILTER PROTO_FILES EXCLUDE REGEX "proto_to_struct\\.proto$")


protobuf_generate_modified(
  LANGUAGE struct_pb
  OUT_VAR PROTO_STRUCT_SRCS
  IMPORT_DIRS ${CMAKE_SOURCE_DIR}
  PROTOS  ${PROTO_FILES}
  PROTOC_OUT_DIR ${CMAKE_BINARY_DIR}
)

# protobuf_generate_struct_pb(PROTO_STRUCT_SRCS PROTO_STRUCT_HDRS
#   ${PROTO_FILES}
# OPTION $ENV{struct_pb_options}
# )


add_library(proto_common OBJECT
  ${PROTO_CLASS_SRCS}
  # ${PROTO_STRUCT_SRCS}
  ${PROTO_PYTHON_SRCS}
)

target_link_libraries(proto_common protobuf::libprotobuf )

target_include_directories(proto_common PUBLIC 
  ${CMAKE_BINARY_DIR} # base.h
  ${CMAKE_CURRENT_LIST_DIR}/ # 找到iguana

)


add_executable(struct_pb_tutorial tutorial.cpp )

target_link_libraries(struct_pb_tutorial 
  proto_common
  # $ENV{proto_message}
)

target_include_directories(struct_pb_tutorial PUBLIC 
  ${CMAKE_BINARY_DIR}
  ${CMAKE_SOURCE_DIR}
)


# add_subdirectory(message)
