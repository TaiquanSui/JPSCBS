#include "MapLoader.h"
#include <fstream>
#include <sstream>

std::vector<std::vector<int>> load_map(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open map file: " + filename);
    }

    std::string line;
    int width = 0, height = 0;

    // 读取地图信息
    std::getline(file, line); // type octile
    std::getline(file, line); // height
    sscanf(line.c_str(), "height %d", &height);
    std::getline(file, line); // width
    sscanf(line.c_str(), "width %d", &width);
    std::getline(file, line); // map

    if (width <= 0 || height <= 0) {
        throw std::runtime_error("Invalid map dimensions");
    }

    // 读取地图数据
    std::vector<std::vector<int>> grid;
    while (std::getline(file, line)) {
        if (line.empty()) continue;  // 跳过空行
        
        std::vector<int> row;
        row.reserve(width);  // 预分配内存以提高性能
        
        for (char cell : line) {
            if (cell == '.') {
                row.push_back(0);  // 可通行区域
            } else if (cell == '@' || cell == 'T') {
                row.push_back(1);  // 障碍物
            }
        }

        if (!row.empty()) {
            if (row.size() != width) {
                throw std::runtime_error("Inconsistent map width at row " + 
                                       std::to_string(grid.size()));
            }
            grid.push_back(row);
        }
    }

    if (grid.size() != height) {
        throw std::runtime_error("Inconsistent map height");
    }

    return grid;
}

std::vector<std::pair<Vertex, Vertex>> load_scen(const std::string& filename) {
    std::vector<std::pair<Vertex, Vertex>> scenarios;
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open scenario file: " + filename);
    }

    std::string line;
    // 跳过第一行（版本信息）
    std::getline(file, line);

    // 读取每一行场景
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        int bucket;
        std::string map_name;
        int height, width;
        int start_x, start_y, goal_x, goal_y;
        float optimal_length;

        // 解析行数据
        if (!(iss >> bucket >> map_name >> height >> width 
              >> start_x >> start_y >> goal_x >> goal_y 
              >> optimal_length)) {
            continue;  // 跳过无效行
        }

        // 创建起点和终点的Vertex
        Vertex start(start_x, start_y);
        Vertex goal(goal_x, goal_y);
        
        // 添加到场景列表
        scenarios.emplace_back(start, goal);
    }

    if (scenarios.empty()) {
        throw std::runtime_error("No valid scenarios found in file: " + filename);
    }

    return scenarios;
}
