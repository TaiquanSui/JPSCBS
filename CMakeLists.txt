cmake_minimum_required(VERSION 3.14)
project(CBS_Localized_Conflict_Resolution)

# 编译选项
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Google Test 配置
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.1
    FIND_PACKAGE_ARGS NAMES GTest
)
FetchContent_MakeAvailable(googletest)
enable_testing()

# 收集源文件
file(GLOB_RECURSE PROJECT_SOURCES 
    "src/algorithm/*.cpp"
    "src/algorithm/*.h"
    "src/data_loader/*.cpp"
    "src/data_loader/*.h"
)

# 创建库
add_library(project_lib ${PROJECT_SOURCES})
target_include_directories(project_lib PUBLIC ${CMAKE_SOURCE_DIR}/src)

# 添加测试目录
add_subdirectory(test)

# 基准测试程序
add_executable(cbs_benchmark src/benchmark/CBSBenchmark.cpp)
target_link_libraries(cbs_benchmark PRIVATE project_lib)

add_executable(jpscbs_benchmark src/benchmark/JPSCBSBenchmark.cpp)
target_link_libraries(jpscbs_benchmark PRIVATE project_lib)

