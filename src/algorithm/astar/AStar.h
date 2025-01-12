#ifndef ASTAR_H
#define ASTAR_H

#include "../Vertex.h"
#include "../action/Action.h"
#include "../Heuristic.h"
#include "../Utility.h"
#include "../cbs/CBS.h"
#include "../Agent.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>

struct AStarNode {
    Vertex pos;
    double g;
    int h;
    std::shared_ptr<AStarNode> parent;
    int time;

    AStarNode(Vertex p, double g, int h, std::shared_ptr<AStarNode> parent = nullptr, int t = 0)
        : pos(p), g(g), h(h), parent(std::move(parent)), time(t) {}

    double f() const { return g + h; }
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
std::vector<Vertex> a_star(const Agent& agent,
                          const std::vector<std::vector<int>>& grid,
                          const std::vector<Constraint>& constraints,
                          int start_time = 0);

// 私有辅助函数
namespace {
    std::vector<Vertex> reconstruct_path(const std::shared_ptr<AStarNode>& goal_node);
    bool check_constraints(const std::vector<Constraint>& constraints, 
                         int agent_id, const Vertex& pos, int time);
}

#endif // ASTAR_H