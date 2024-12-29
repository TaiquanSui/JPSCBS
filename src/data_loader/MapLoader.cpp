#include "MapLoader.h"
#include <fstream>
#include <sstream>

std::vector<std::vector<int>> load_map(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;

    for (int i = 0; i < 4; ++i) {
        std::getline(file, line);
    }

    std::vector<std::vector<int>> grid;
    while (std::getline(file, line)) {
        std::vector<int> row;
        for (char cell : line) {
            if (cell == '.') {
                row.push_back(0);
            } else if (cell == '@' || cell == 'T') {
                row.push_back(1);
            }
        }
        grid.push_back(row);
    }
    return grid;
}

std::vector<std::pair<Vertex, Vertex>> load_scen(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;
    std::getline(file, line); // 跳过版本行
    
    std::vector<std::pair<Vertex, Vertex>> scenarios;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        int x_start, y_start, x_goal, y_goal;
        
        // 跳过前4个token
        for (int i = 0; i < 4; ++i) {
            iss >> token;
        }
        
        // 读取坐标
        iss >> x_start >> y_start >> x_goal >> y_goal;
        scenarios.emplace_back(Vertex(x_start, y_start), Vertex(x_goal, y_goal));
    }
    
    return scenarios;
}
