#ifndef ASTAR_H
#define ASTAR_H

#include "../Vertex.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cmath>

struct AStarNode {
    Vertex pos;
    int g, h;
    std::shared_ptr<AStarNode> parent;

    AStarNode(Vertex p, int g, int h, std::shared_ptr<AStarNode> parent = nullptr)
        : pos(p), g(g), h(h), parent(std::move(parent)) {}

    int f() const { return g + h; }
};

struct AStarPathComparator {
    bool operator()(const AStarNode& a, const AStarNode& b) const {
        return a.f() > b.f(); // 基于f值的优先级
    }
};

std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal, const std::vector<std::vector<int>>& grid);

#endif // ASTAR_H