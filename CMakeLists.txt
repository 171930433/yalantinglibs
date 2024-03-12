cmake_minimum_required(VERSION 3.10)

set(CMAKE_C_COMPILER gcc-9)
set(CMAKE_CXX_COMPILER g++-9)
set(CMAKE_CXX_STANDARD 17)

project(yaLanTingLibs
        VERSION 0.2.9
        DESCRIPTION "yaLanTingLibs"
        HOMEPAGE_URL "https://github.com/alibaba/yalantinglibs"
        LANGUAGES CXX
        )

find_package(Protobuf REQUIRED)

# 将插件目录添加到环境变量
execute_process(COMMAND export PATH=$PATH:${CMAKE_BINARY_DIR}/output/bin/)



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


# add_subdirectory(src/struct_pb/protoc-plugin)
add_subdirectory(src/example)
