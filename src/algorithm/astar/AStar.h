#ifndef ASTAR_H
#define ASTAR_H

#include "../Vertex.h"
#include "../action/Action.h"
#include "../Heuristic.h"
#include "../Utility.h"
#include "../cbs/CBS.h"
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

// Basic A* search
std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal, 
                          const std::vector<std::vector<int>>& grid);

// Constrained A* search
std::vector<Vertex> a_star(int agent_id,
                          const Vertex& start,
                          const Vertex& goal,
                          const std::vector<std::vector<int>>& grid,
                          const std::vector<Constraint>& constraints,
                          int start_time = 0);

#endif // ASTAR_H