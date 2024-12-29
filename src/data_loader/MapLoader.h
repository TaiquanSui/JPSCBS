#ifndef MAPLOADER_H
#define MAPLOADER_H

#include "../algorithm/Vertex.h"
#include "../algorithm/Agent.h"
#include <vector>
#include <string>
#include <utility>

// 加载地图
std::vector<std::vector<int>> load_map(const std::string& filename);

// 加载场景文件，返回所有起点和终点对
std::vector<std::pair<Vertex, Vertex>> load_scen(const std::string& filename);

// 创建智能体列表
std::vector<Agent> create_agents(const std::vector<std::pair<Vertex, Vertex>>& scenarios, 
                               int num_agents);

#endif // MAPLOADER_H
