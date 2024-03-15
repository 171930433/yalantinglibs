
  # 设置编译器选项
add_compile_options(-Wno-attributes)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/examples)

# protobuf_generate_struct_pb 函数
include(${struct_pb_plugin_DIRS}/cmake/struct_pb.cmake)

find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED COMPONENTS core)

# file(GLOB PROTO_FILES "*.proto")

# set(PROTO_FILES "base.proto" "zframe.proto" "imu.proto")
set(PROTO_FILES "/home/gsk/pro/yalantinglibs/src/example/base.proto" "/home/gsk/pro/yalantinglibs/src/example/zframe.proto")


# 输出到指定目录
message(!!!!!!"${CMAKE_BINARY_DIR}"!!!!!!!)
protobuf_generate_cpp(COMMON_PROTO_SRCS COMMON_PROTO_HDRS
  ${struct_pb_plugin_DIRS}/proto_to_struct.proto
  PROTOC_OUT_DIR ${CMAKE_BINARY_DIR}/src/protoc-plugin
)
protobuf_generate_python(COMMON_PROTO_PYTHON_SRCS
  ${struct_pb_plugin_DIRS}/proto_to_struct.proto
  PROTOC_OUT_DIR ${CMAKE_BINARY_DIR}/src/protoc-plugin
)

# generate .pb.cc .pb.h
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
  ${PROTO_FILES}
)

protobuf_generate_python(PROTO_PYTHON_SRCS
  ${PROTO_FILES}
)

# 删除点云,图片proto记录文件
list(FILTER PROTO_FILES EXCLUDE REGEX "pointcloud\\.proto$|image\\.proto$")
# message(status "-------${PROTO_FILES}-------")


protobuf_generate_struct_pb(PROTO_STRUCT_SRCS PROTO_STRUCT_HDRS
  ${PROTO_FILES}
OPTION "namespace=zhito.inner_struct,converter_namespace=zhito.converter"
# PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}
)

add_executable(struct_pb_tutorial tutorial.cpp 
  # 
  ${COMMON_PROTO_SRCS}  ${COMMON_PROTO_HDRS} ${COMMON_PROTO_PYTHON_SRCS}
  # zpointcloud.cpp zimage.cpp
  ${PROTO_SRCS}
  ${PROTO_HDRS}
  ${PROTO_STRUCT_SRCS}
  ${PROTO_STRUCT_HDRS}
  # ${PROTO_PYTHON_SRCS}
)

target_link_libraries(struct_pb_tutorial protobuf::libprotobuf 
  pcl_common pcl_io # pcl
  opencv_imgcodecs opencv_core  # opencv
)

target_include_directories(struct_pb_tutorial PUBLIC 
  ${CMAKE_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_LIST_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}/src/protoc-plugin
)


