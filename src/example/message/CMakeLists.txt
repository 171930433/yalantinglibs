file(GLOB PROTO_FILES "*.proto")


# generate .pb.cc .pb.h
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS   ${PROTO_FILES})

protobuf_generate_python(PROTO_PYTHON_SRCS   ${PROTO_FILES})

# 删除点云,图片proto记录文件
list(FILTER PROTO_FILES EXCLUDE REGEX "pointcloud\\.proto$|image\\.proto$")

protobuf_generate_struct_pb(PROTO_STRUCT_SRCS PROTO_STRUCT_HDRS
  ${PROTO_FILES}
OPTION $ENV{struct_pb_options}
)

add_library($ENV{proto_message} OBJECT
  zpointcloud.cpp zimage.cpp
  # 
  ${PROTO_SRCS}  
  ${PROTO_STRUCT_SRCS}
  ${PROTO_PYTHON_SRCS}
)

target_link_libraries($ENV{proto_message} protobuf::libprotobuf 
  pcl_common pcl_io # pcl
  opencv_imgcodecs opencv_core  # opencv
)

target_include_directories($ENV{proto_message} PUBLIC 
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_BINARY_DIR}         # 找到build目录下的zframe.struct_pb.h
  ${CMAKE_CURRENT_LIST_DIR}/.. # 找到iguana
  ${CMAKE_CURRENT_BINARY_DIR}/.. # 找到common
# 第三方库
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


