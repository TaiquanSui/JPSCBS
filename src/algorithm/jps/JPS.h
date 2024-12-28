#ifndef JPS_H
#define JPS_H

#include "../Vertex.h"
#include "../astar/AStar.h"
#include "../Heuristic.h"
#include "../action/Action.h"
#include "JPSState.h"
#include <vector>
#include <memory>
#include <queue>

struct Interval {
    Vertex jump_point1;
    Vertex jump_point2;

    Interval(Vertex jp1, Vertex jp2) : jump_point1(jp1), jump_point2(jp2) {}
};

struct JPSPath {
    std::vector<Vertex> path;
    std::vector<Vertex> jump_points;
    std::vector<Interval> possible_intervals;

    JPSPath(const std::vector<Vertex>& path, const std::vector<Vertex>& jump_points, const std::vector<Interval>& possible_intervals)
        : path(path), jump_points(jump_points), possible_intervals(possible_intervals) {}
};

struct JPSPathComparator {
    bool operator()(const JPSPath& a, const JPSPath& b) const {
        return a.path.size() > b.path.size(); // 基于路径长度的优先级
    }
};

JPSPath jump_point_search(const Vertex& start, const Vertex& goal, const std::vector<std::vector<int>>& grid, JPSState& state);

#endif // JPS_H 