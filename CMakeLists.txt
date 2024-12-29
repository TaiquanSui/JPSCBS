cmake_minimum_required(VERSION 3.30)
project(CBS_Localized_Conflict_Resolution)

set(CMAKE_CXX_STANDARD 20)

# 创建库
add_library(project_lib
        src/algorithm/Vertex.cpp
        src/algorithm/Vertex.h
        src/algorithm/astar/AStar.cpp
        src/algorithm/astar/AStar.h
        src/algorithm/cbs/CBS.cpp
        src/algorithm/cbs/CBS.h
        src/data_loader/MapLoader.cpp
        src/data_loader/MapLoader.h
        src/algorithm/Heuristic.cpp
        src/algorithm/Heuristic.h
        src/algorithm/Agent.cpp
        src/algorithm/Agent.h
        src/algorithm/jpscbs/JPSCBS.cpp
        src/algorithm/jpscbs/JPSCBS.h)

# 主程序
add_executable(CBS_Localized_Conflict_Resolution src/main.cpp)
target_link_libraries(CBS_Localized_Conflict_Resolution PRIVATE project_lib)

# 基准测试程序
add_executable(cbs_benchmark src/benchmark/CBSBenchmark.cpp)
target_link_libraries(cbs_benchmark PRIVATE project_lib)
