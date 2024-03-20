file(GLOB PROTO_FILES "*.proto")


# generate .pb.cc .pb.h
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
  ${PROTO_FILES}
)

protobuf_generate_python(PROTO_PYTHON_SRCS
  ${PROTO_FILES}
)

# 删除点云,图片proto记录文件
list(FILTER PROTO_FILES EXCLUDE REGEX "pointcloud\\.proto$|image\\.proto$")

protobuf_generate_struct_pb(PROTO_STRUCT_SRCS PROTO_STRUCT_HDRS
  ${PROTO_FILES}
OPTION "namespace=zhito.inner_struct,converter_namespace=zhito.converter"
)

add_library($ENV{proto_message} OBJECT
  # 
  zpointcloud.cpp zimage.cpp
  ${PROTO_SRCS}
  ${PROTO_HDRS}
  ${PROTO_STRUCT_SRCS}
  ${PROTO_STRUCT_HDRS}
  ${PROTO_PYTHON_SRCS}
)

target_link_libraries($ENV{proto_message} protobuf::libprotobuf 
  pcl_common pcl_io # pcl
  opencv_imgcodecs opencv_core  # opencv
)

target_include_directories($ENV{proto_message} PUBLIC 
  ${CMAKE_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/.. # 找到common
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # 找到iguana
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}/src/protoc-plugin
)


