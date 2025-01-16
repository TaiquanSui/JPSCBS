#pragma once
#include "../algorithm/Vertex.h"
#include "../algorithm/Agent.h"
#include <vector>
#include <string>

// Load map, return 2D grid
std::vector<std::vector<int>> load_map(const std::string& filename);

// Load scenario, return agent list
std::vector<Agent> load_scen(const std::string& filename);
