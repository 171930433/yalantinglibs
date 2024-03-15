cmake_minimum_required(VERSION 3.10)

# set(CMAKE_C_COMPILER gcc-9)
# set(CMAKE_CXX_COMPILER g++-9)
set(CMAKE_CXX_STANDARD 17)

project(yaLanTingLibs
        VERSION 0.2.9
        DESCRIPTION "yaLanTingLibs"
        HOMEPAGE_URL "https://github.com/alibaba/yalantinglibs"
        LANGUAGES CXX
        )


# set(SYSROOT_DIR /opt/zhito/sysroot)
set(CMAKE_PREFIX_PATH /home/gsk/pro/CyberRT/install/)

find_package(Protobuf REQUIRED)

# load pack finder
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Find/)

find_package(Threads REQUIRED)
link_libraries(Threads::Threads)

include(cmake/install.cmake)

if(CMAKE_PROJECT_NAME STREQUAL "yaLanTingLibs") # if ylt is top-level project
        # add include path
        # include_directories(include)
        # include_directories(include/ylt/thirdparty)
        # include_directories(src/include)

        # include(cmake/utils.cmake)
        # include(cmake/struct_pb.cmake)
        # include(cmake/build.cmake)
        # include(cmake/develop.cmake)
        # # add project config, such as enable_ssl.
        # include(cmake/config.cmake)
        # # add project's source such as unit test, example & benchmark
        # include(cmake/subdir.cmake)
endif()


# 指定struct_pb库目录
set(struct_pb_plugin_DIRS ${CMAKE_CURRENT_LIST_DIR}/src/protoc-plugin)  # struct_pb_plugin_DIRS 将指定 proto_to_struct.proto,struct_pb.cmake等目录
set(Protobuf_IMPORT_DIRS ${CMAKE_SOURCE_DIR})     # 设置protoc --proto_path参数


# add_subdirectory(src/protoc-plugin)
add_subdirectory(src/example)