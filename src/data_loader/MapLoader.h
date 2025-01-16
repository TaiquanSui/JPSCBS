#pragma once
#include "../algorithm/Vertex.h"
#include "../algorithm/Agent.h"
#include <vector>
#include <string>

// 加载地图，返回二维网格
std::vector<std::vector<int>> load_map(const std::string& filename);

// 加载场景，返回智能体列表
std::vector<Agent> load_scen(const std::string& filename);
