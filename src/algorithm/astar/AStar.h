#ifndef ASTAR_H
#define ASTAR_H

#include "../Vertex.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>

struct AStarNode {
    Vertex pos;
    int g, h;
    int time;  // 添加时间维度
    std::shared_ptr<AStarNode> parent;

    AStarNode(Vertex p, int g, int h, int t = 0, std::shared_ptr<AStarNode> parent = nullptr)
        : pos(p), g(g), h(h), time(t), parent(std::move(parent)) {}

    int f() const { return g + h; }
};

struct AStarNodeComparator {
    bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
        return a->f() > b->f();
    }
};

// 基本的A*搜索
std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal, 
                          const std::vector<std::vector<int>>& grid);

// 带约束的A*搜索
std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal, 
                          const std::vector<std::vector<int>>& grid,
                          const std::function<bool(const Vertex&, int)>& is_valid);

#endif // ASTAR_H