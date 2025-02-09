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
# FetchContent_Declare(
#     googletest
#     GIT_REPOSITORY https://github.com/google/googletest.git
#     GIT_TAG release-1.12.1
# )
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://gitee.com/mirrors/googletest.git
    GIT_TAG release-1.12.1
)
FetchContent_MakeAvailable(googletest)
enable_testing()

# 收集源文件
file(GLOB_RECURSE PROJECT_SOURCES 
    "src/*.cpp"
    "src/*.h"
    "data_loader/*.cpp"
    "data_loader/*.h"
    "benchmark/*.cpp"
    "benchmark/*.h"
)

# 只排除具体的benchmark可执行文件源文件
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*/CBSBenchmark\\.cpp$")
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*/JPSCBSBenchmark\\.cpp$")
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*/Benchmark\\.cpp$")
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*/CBSSingleTest\\.cpp$")
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*/JPSCBSSingleTest\\.cpp$")
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*/GenerateComparisonSummary\\.cpp$")

# 创建库
add_library(project_lib ${PROJECT_SOURCES})
target_include_directories(project_lib PUBLIC ${CMAKE_SOURCE_DIR})

# 添加测试目录
add_subdirectory(test)

# 基准测试程序
add_executable(cbs_benchmark benchmark/CBSBenchmark.cpp)
target_link_libraries(cbs_benchmark PRIVATE project_lib)

add_executable(jpscbs_benchmark benchmark/JPSCBSBenchmark.cpp)
target_link_libraries(jpscbs_benchmark PRIVATE project_lib)

add_executable(comparison_benchmark benchmark/Benchmark.cpp)
target_link_libraries(comparison_benchmark PRIVATE project_lib)

add_executable(jpscbs_single_test benchmark/JPSCBSSingleTest.cpp)
target_link_libraries(jpscbs_single_test PRIVATE project_lib)

add_executable(cbs_single_test benchmark/CBSSingleTest.cpp)
target_link_libraries(cbs_single_test PRIVATE project_lib)

add_executable(generate_comparison_summary benchmark/GenerateComparisonSummary.cpp)
target_link_libraries(generate_comparison_summary PRIVATE project_lib)


