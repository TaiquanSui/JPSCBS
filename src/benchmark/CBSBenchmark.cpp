#include "../data_loader/MapLoader.h"
#include "../algorithm/cbs/CBS.h"
#include <chrono>
#include <iostream>
#include <iomanip>

int main() {
    // 加载地图
    auto grid = load_map("data/mapf-map/Berlin_1_256.map");
    
    // 加载场景
    auto scenarios = load_scen("data/mapf-scen-random/Berlin_1_256-random-1.scen");
    
    // 创建CBS实例
    CBS cbs;
    
    // 记录开始时间
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 运行CBS
    auto solution = cbs.solve(scenarios, grid);
    
    // 记录结束时间
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time
    ).count();
    
    // 输出结果
    std::cout << "Total Agents: " << scenarios.size() 
              << " | Success: " << (!solution.empty() ? "Yes" : "No")
              << " | Time: " << duration << "ms" << std::endl;
    
    return 0;
} 