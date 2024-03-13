cmake_minimum_required(VERSION 3.10)
# set(CMAKE_C_COMPILER gcc-9)
# set(CMAKE_CXX_COMPILER g++-9)
# set(CMAKE_CXX_STANDARD 17)

project(protoc-gen-struct_pb)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/bin)

find_package(Protobuf QUIET)
if (Protobuf_FOUND)

    file(GLOB SRCS "*.cpp" "*.cc" "*.h")
    protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto_to_struct.proto)

    add_executable(protoc-gen-struct_pb ${SRCS} ${PROTO_SRCS} ${PROTO_HDRS})
    target_include_directories(protoc-gen-struct_pb PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
    target_link_libraries(protoc-gen-struct_pb PUBLIC protobuf::libprotoc protobuf::libprotobuf)
    install(
        TARGETS protoc-gen-struct_pb
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
    )
else ()
    message(INFO "struct_pb is skipped. To build struct_pb protoc plugin, you must install libprotoc first\n"

            "see https://alibaba.github.io/yalantinglibs/en/struct_pb/struct_pb_generating_your_struct.html"
            )
endif ()
