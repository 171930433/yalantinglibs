
  # 设置编译器选项
add_compile_options(-Wno-attributes)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/examples)

include(cmake/struct_pb.cmake)

cmake_policy(SET CMP0075 NEW)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED COMPONENTS core)

# generate .pb.cc .pb.h
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
proto_to_struct.proto
base.proto
imu.proto
gnss.proto
state.proto
chasis.proto
perception.proto
pointcloud.proto
image.proto
)

  
protobuf_generate_python(PROTO_PYTHON_SRCS
  proto_to_struct.proto
  base.proto
  imu.proto
  gnss.proto
  state.proto
  chasis.proto
  perception.proto
  pointcloud.proto
  image.proto  
)

protobuf_generate_struct_pb(PROTO_STRUCT_SRCS PROTO_STRUCT_HDRS
base.proto
imu.proto
gnss.proto
state.proto
chasis.proto
perception.proto
OPTION "namespace=inner_struct,converter_namespace=converter")

add_executable(struct_pb_tutorial tutorial.cpp zpointcloud.cpp zimage.cpp
  ${PROTO_SRCS}
  ${PROTO_HDRS}
  ${PROTO_STRUCT_SRCS}
  ${PROTO_STRUCT_HDRS}
  ${PROTO_PYTHON_SRCS}
)

target_link_libraries(struct_pb_tutorial protobuf::libprotobuf 
  pcl_common pcl_io # pcl
  opencv_imgcodecs opencv_core  # opencv
)

target_include_directories(struct_pb_tutorial PUBLIC 
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_LIST_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


