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
set(CMAKE_PREFIX_PATH "/home/gsk/pro/CyberRT/install")

find_package(protobuf REQUIRED)

# add_subdirectory(src/protoc-plugin)

option(BUILD_EXAMPLES OFF)

if(BUILD_EXAMPLES)
endif()
add_subdirectory(src/example)
