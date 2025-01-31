# Collect test source files
file(GLOB_RECURSE TEST_SOURCES 
    "algorithm/astar/*_test.cpp"
    "algorithm/cbs/*_test.cpp"
    "algorithm/jps/*_test.cpp"
    "algorithm/jpscbs/*_test.cpp"
)

# Create test executable
add_executable(algorithm_tests ${TEST_SOURCES})

# Set include directories and link libraries
target_include_directories(algorithm_tests PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(algorithm_tests
    PRIVATE
    project_lib
    GTest::gtest
    GTest::gtest_main
)

# Discover and add tests
include(GoogleTest)
gtest_discover_tests(algorithm_tests
    PROPERTIES
    TIMEOUT 60  # Set test timeout to 60 seconds
) 