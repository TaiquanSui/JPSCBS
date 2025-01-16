# 收集测试源文件
file(GLOB_RECURSE TEST_SOURCES 
    "algorithm/*_test.cpp"
)

# 创建测试可执行文件
add_executable(algorithm_tests ${TEST_SOURCES})

# 设置包含目录和链接库
target_include_directories(algorithm_tests PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(algorithm_tests
    PRIVATE
    project_lib
    GTest::gtest
    GTest::gtest_main
)

# 发现并添加测试
include(GoogleTest)
gtest_discover_tests(algorithm_tests
    PROPERTIES
    TIMEOUT 10  # 设置测试超时时间为10秒
) 