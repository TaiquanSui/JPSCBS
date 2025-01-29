#include "MapLoader.h"
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
    while (std::getline(file, line) && row < height) {
        if (line.empty()) continue;
        if (line.length() < width) {
            throw std::runtime_error("地图行长度不足");
        }
        
        for (int col = 0; col < width; ++col) {
            // 0表示可通行(@或.), 1表示障碍物(T)
            map[row][col] = (line[col] == 'T') ? 1 : 0;
        }
        ++row;
    }

    if (row != height) {
        throw std::runtime_error("地图行数不足");
    }

    return map;
}

std::vector<Agent> load_scen(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开场景文件: " + filename);
    }

    std::vector<Agent> agents;
    std::string line;
    int agent_id = 0;

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

        // 创建新的Agent
        agents.emplace_back(
            agent_id++,
            Vertex(start_x, start_y),
            Vertex(goal_x, goal_y)
        );
    }

    return agents;
}
