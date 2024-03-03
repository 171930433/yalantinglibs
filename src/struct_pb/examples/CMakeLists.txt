if("${yaLanTingLibs_SOURCE_DIR}" STREQUAL "${CMAKE_SOURCE_DIR}")
  # If this is a subproject in ylt
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/examples)
else()
  # else find installed yalantinglibs
    cmake_minimum_required(VERSION 3.15)
    project(coro_rpc_examples)
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release")
    endif()
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
  # if you have install ylt  
    find_package(yalantinglibs REQUIRED)
    link_libraries(yalantinglibs::yalantinglibs)
  # else 
  # include_directories(include)
  # include_directories(include/ylt/thirdparty)
    include(struct_pb_helper.cmake)
endif()

find_package(Protobuf QUIET)
if (Protobuf_FOUND)

    # generate .pb.cc .pb.h
    protobuf_generate_cpp(PROTO_SRCS
    PROTO_HDRS
    addressbook.proto
    proto_to_struct.proto
    )
    add_executable(struct_pb_tutorial tutorial.cpp           
      ${PROTO_SRCS}
      ${PROTO_HDRS})
      target_link_libraries(struct_pb_tutorial protobuf::libprotobuf)

    target_include_directories(struct_pb_tutorial PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

    target_protos_struct_pb(struct_pb_tutorial PRIVATE addressbook.proto 
    OPTION "namespace=MyInner")

endif()
