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

std::pair<std::vector<Vertex>, std::vector<Vertex>> load_scen(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;
    std::getline(file, line);

    std::vector<Vertex> starts, goals;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        int x_start, y_start, x_goal, y_goal;

        for (int i = 0; i < 8; ++i) {
            iss >> token;
            if (i == 4) x_start = std::stoi(token);
            if (i == 5) y_start = std::stoi(token);
            if (i == 6) x_goal = std::stoi(token);
            if (i == 7) y_goal = std::stoi(token);
        }

        starts.emplace_back(x_start, y_start);
        goals.emplace_back(x_goal, y_goal);
    }

    return {starts, goals};
}
