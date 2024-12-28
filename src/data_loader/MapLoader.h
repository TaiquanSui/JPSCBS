#ifndef MAPLOADER_H
#define MAPLOADER_H

#include "../algorithm/Vertex.h"
#include <vector>
#include <string>
#include <utility>

std::vector<std::vector<int>> load_map(const std::string& filename);
std::pair<std::vector<Vertex>, std::vector<Vertex>> load_scen(const std::string& filename);

#endif // MAPLOADER_H
