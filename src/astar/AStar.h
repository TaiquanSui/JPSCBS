#ifndef ASTAR_H
#define ASTAR_H

#include "../Vertex.h"
#include "../action/Action.h"
#include "../Heuristic.h"
#include "../cbs/CBS.h"
#include "../cbs/ConflictAvoidanceTable.h"
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
    int conflicts;

    AStarNode(Vertex p, double g, double h, std::shared_ptr<AStarNode> parent = nullptr, int t = 0, int c = 0)
        : pos(p), g(g), h(h), parent(std::move(parent)), time(t), conflicts(c) {}

    double f() const { return g + h; }
};

struct AStarNodeComparator {
    bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
        // 1. f(n) 低的优先
        if (std::abs(a->f() - b->f()) > 1e-6) {
            return a->f() > b->f();
        }

        // 2. h=0 的节点优先（说明到达目标）
        if (std::abs(a->h) < 1e-6) {  // a 到达目标
            return false;  // a 优先
        }

        if (std::abs(b->h) < 1e-6) {  // b 到达目标
            return true;   // b 优先
        }
        
        // 3. 冲突次数低的优先
        if (a->conflicts != b->conflicts) {
            return a->conflicts > b->conflicts;
        }
        
        // 4. g(n) 高的优先
        if (std::abs(a->g - b->g) > 1e-6) {
            return a->g < b->g;  // 注意这里是 < 因为我们要高的优先
        }

        // // 5. 对角线移动优先
        // if (a->parent && b->parent && a->parent->pos == b->parent->pos) {
        //     if (utils::isDiagonal(a->pos - a->parent->pos)) {
        //         return false;
        //     }

        //     if (utils::isDiagonal(b->pos - b->parent->pos)) {
        //         return true;
        //     }
        // }
        
        // 6. 时间戳早的优先
        return a->time > b->time;
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
                          const int start_time = 0,
                          const ConflictAvoidanceTable& cat = ConflictAvoidanceTable());

#endif // ASTAR_H