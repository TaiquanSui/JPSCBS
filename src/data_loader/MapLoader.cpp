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

    // Read map information
    std::getline(file, line); // type octile
    std::getline(file, line); // height
    sscanf(line.c_str(), "height %d", &height);
    std::getline(file, line); // width
    sscanf(line.c_str(), "width %d", &width);
    std::getline(file, line); // map

    if (width <= 0 || height <= 0) {
        throw std::runtime_error("Invalid map dimensions");
    }

    // Read map data
    std::vector<std::vector<int>> grid;
    while (std::getline(file, line)) {
        if (line.empty()) continue;  // Skip empty lines
        
        std::vector<int> row;
        row.reserve(width);  // Preallocate memory for performance
        
        for (char cell : line) {
            if (cell == '.') {
                row.push_back(0);  // Passable area
            } else if (cell == '@' || cell == 'T') {
                row.push_back(1);  // Obstacle
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

std::vector<Agent> load_scen(const std::string& filename) {
    std::vector<Agent> agents;
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open scenario file: " + filename);
    }

    std::string line;
    // Skip first line (version information)
    std::getline(file, line);

    int agent_id = 0;  // Assign a unique agent ID to each scenario
    // Read each line of scenario
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        int bucket;
        std::string map_name;
        int height, width;
        int start_x, start_y, goal_x, goal_y;
        float optimal_length;

        // Parse line data
        if (!(iss >> bucket >> map_name >> height >> width 
              >> start_x >> start_y >> goal_x >> goal_y 
              >> optimal_length)) {
            continue;  // Skip invalid lines
        }

        // Create new agent
        agents.emplace_back(agent_id++, Vertex(start_x, start_y), Vertex(goal_x, goal_y));
    }

    if (agents.empty()) {
        throw std::runtime_error("No valid scenarios found in file: " + filename);
    }

    return agents;
}
