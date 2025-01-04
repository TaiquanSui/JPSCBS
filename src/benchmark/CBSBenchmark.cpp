#include "../data_loader/MapLoader.h"
#include "../algorithm/cbs/CBS.h"
#include <chrono>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

std::string get_project_root() {
    fs::path current_path = fs::current_path();
    while (!fs::exists(current_path / "data")) {
        if (current_path.parent_path() == current_path) {
            throw std::runtime_error("Cannot find project root directory");
        }
        current_path = current_path.parent_path();
    }
    return current_path.string();
}

void run_benchmark(const std::string& map_file, const std::string& scen_file) {
    try {
        // 加载数据
        auto grid = load_map(map_file);
        auto scenarios = load_scen(scen_file);
        
        // 创建CBS实例（启用优化）
        CBS cbs(true);
        
        // 计时
        auto start_time = std::chrono::high_resolution_clock::now();
        auto solution = cbs.solve(scenarios, grid);
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time).count();
            
        // 计算总代价
        int total_cost = 0;
        if (!solution.empty()) {
            total_cost = std::accumulate(solution.begin(), solution.end(), 0,
                [](int sum, const auto& path) { 
                    return sum + path.size() - 1; 
                });
        }
        
        // 输出详细结果
        std::cout << std::fixed << std::setprecision(2)
                  << "Results:\n"
                  << "  Map: " << map_file << "\n"
                  << "  Scenario: " << scen_file << "\n"
                  << "  Total Agents: " << scenarios.size() << "\n"
                  << "  Success: " << (!solution.empty() ? "Yes" : "No") << "\n"
                  << "  Runtime: " << duration / 1000.0 << " seconds\n"
                  << "  Solution Cost: " << total_cost << "\n"
                  << "----------------------------------------" << std::endl;
                  
    } catch (const std::exception& e) {
        std::cerr << "Error testing " << map_file << " with " << scen_file << ": " 
                  << e.what() << std::endl;
    }
}

std::string get_map_name(const std::string& map_path) {
    std::string filename = fs::path(map_path).filename().string();
    return filename.substr(0, filename.find(".map"));
}

int main() {
    try {
        std::string root_dir = get_project_root();
        std::string data_dir = root_dir + "/data";
        std::string maps_dir = data_dir + "/mapf-map";
        std::string random_scen_dir = data_dir + "/mapf-scen-random";
        std::string even_scen_dir = data_dir + "/mapf-scen-even";
        
        if (!fs::exists(maps_dir)) {
            throw std::runtime_error("Maps directory not found: " + maps_dir);
        }
        
        // 遍历所有地图文件
        for (const auto& map_entry : fs::directory_iterator(maps_dir)) {
            if (map_entry.path().extension() == ".map") {
                std::string map_path = map_entry.path().string();
                std::string map_name = get_map_name(map_path);
                std::cout << "map_name: " << map_name << std::endl;
                
                // 测试random场景
                for (int i = 1; i <= 25; ++i) {
                    std::string scen_file = random_scen_dir + "/" + map_name + 
                                          "-random-" + std::to_string(i) + ".scen";
                    std::cout << "scen_file: " << scen_file << std::endl;
                    if (fs::exists(scen_file)) {
                        run_benchmark(map_path, scen_file);
                    }
                }
                
                // 测试even场景
                for (int i = 1; i <= 25; ++i) {
                    std::string scen_file = even_scen_dir + "/" + map_name + 
                                          "-even-" + std::to_string(i) + ".scen";
                    std::cout << "scen_file: " << scen_file << std::endl;
                    if (fs::exists(scen_file)) {
                        run_benchmark(map_path, scen_file);
                    }
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 