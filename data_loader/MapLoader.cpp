#include "MapLoader.h"
#include "../src/utilities/Log.h"
#include <fstream>
#include <sstream>

std::vector<std::vector<int>> load_map(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开地图文件: " + filename);
    }

    std::string line;
    int width = 0, height = 0;

    // 读取头信息
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        std::string key;
        iss >> key;
        
        if (key == "height") {
            iss >> height;
        } else if (key == "width") {
            iss >> width;
        } else if (key == "map") {
            break;  // 地图内容开始
        }
    }

    if (width <= 0 || height <= 0) {
        throw std::runtime_error("地图尺寸无效");
    }

    // 读取地图内容
    std::vector<std::vector<int>> map(height, std::vector<int>(width, 0));
    int row = 0;
    int col = 0;
    char c;
    
    // 逐字符读取地图内容
    while (file.get(c) && row < height) {
        // 跳过换行符和回车符
        if (c == '\n' || c == '\r') {
            continue;
        }
        
        // 处理当前字符
        switch(c) {
            case '.':
            case 'G':
                map[row][col] = 0;  // 可通行
                break;
            case '@':
            case 'O':
            case 'T':
                map[row][col] = 1;  // 不可通行
                break;
            case 'S':
            case 'W':
                map[row][col] = 0;  // 暂时视为可通行
                break;
            default:
                throw std::runtime_error("地图包含未知字符: " + std::string(1, c));
        }
        
        // 更新位置
        ++col;
        if (col >= width) {
            col = 0;
            ++row;
        }
    }

    if (row != height) {
        throw std::runtime_error("地图数据不足");
    }

    return map;
}

std::vector<Agent> load_scen(const std::string& filename, const std::vector<std::vector<int>>& grid) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开场景文件: " + filename);
    }

    std::vector<Agent> agents;
    std::string line;
    int agent_id = 0;  // 为每个agent分配递增的ID

    // 跳过版本行
    std::getline(file, line);

    // 读取每个场景
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::istringstream iss(line);
        std::string bucket, map_name;
        int width, height, start_x, start_y, goal_x, goal_y;
        double optimal_length;

        // 解析场景行
        if (!(iss >> bucket >> map_name >> width >> height 
              >> start_x >> start_y >> goal_x >> goal_y >> optimal_length)) {
            continue;  // 跳过无效行
        }

        agents.emplace_back(
            agent_id++,
            Vertex(start_x, start_y),
            Vertex(goal_x, goal_y)
        );
    }

    return agents;
}
