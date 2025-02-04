#ifndef ASTAR_H
#define ASTAR_H

#include "../Vertex.h"
#include "../action/Action.h"
#include "../Heuristic.h"
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
    double h;
    std::shared_ptr<AStarNode> parent;
    int time;

    AStarNode(Vertex p, double g, double h, std::shared_ptr<AStarNode> parent = nullptr, int t = 0)
        : pos(p), g(g), h(h), parent(std::move(parent)), time(t) {}

    double f() const { return g + h; }
};

struct AStarNodeComparator {
    bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
        return a->f() > b->f();
    }
};

struct StateKey {
    Vertex pos;
    int time;
    
    bool operator==(const StateKey& other) const {
        return pos == other.pos && time == other.time;
    }
};

struct StateKeyHash {
    std::size_t operator()(const StateKey& key) const {
        return std::hash<int>()(key.pos.x) ^ 
               std::hash<int>()(key.pos.y) ^ 
               std::hash<int>()(key.time);
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