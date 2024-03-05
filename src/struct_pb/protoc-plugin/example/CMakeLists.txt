
  # 设置编译器选项
add_compile_options(-Wno-attributes)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/examples)

include(../cmake/struct_pb.cmake)

if (Protobuf_FOUND)

    # generate .pb.cc .pb.h
    protobuf_generate_cpp(PROTO_SRCS
    PROTO_HDRS
    # addressbook.proto
    proto_to_struct.proto
    base.proto
    # imu.proto
    )
    add_executable(struct_pb_tutorial tutorial.cpp           
      ${PROTO_SRCS}
      ${PROTO_HDRS})
      target_link_libraries(struct_pb_tutorial protobuf::libprotobuf)

    target_include_directories(struct_pb_tutorial PUBLIC 
      ${CMAKE_CURRENT_BINARY_DIR}
      ${CMAKE_CURRENT_LIST_DIR}
      )

    target_protos_struct_pb(struct_pb_tutorial PRIVATE 
    # addressbook.proto  
    base.proto
    # imu.proto
    OPTION "namespace=inner_struct")

endif()
