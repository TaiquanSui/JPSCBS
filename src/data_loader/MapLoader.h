#pragma once
#include "../algorithm/Vertex.h"
#include <vector>
#include <string>

// 加载地图，返回二维网格
std::vector<std::vector<int>> load_map(const std::string& filename);

// 加载场景，返回起点和终点对的列表
std::vector<std::pair<Vertex, Vertex>> load_scen(const std::string& filename);
