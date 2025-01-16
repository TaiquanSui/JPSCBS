#ifndef JPS_H
#define JPS_H

#include "../Vertex.h"
#include "../astar/AStar.h"
#include "../Heuristic.h"
#include "../action/Action.h"
#include <vector>
#include <memory>
#include <queue>

struct JPSState {
    std::priority_queue<std::shared_ptr<AStarNode>, 
                       std::vector<std::shared_ptr<AStarNode>>, 
                       AStarNodeComparator> open_list;
    bool is_complete = false;

    JPSState() = default;
};

struct Interval {
    std::vector<Vertex> jump_points;  // 按顺序存储interval中的所有跳点

    Interval(const std::vector<Vertex>& points) : jump_points(points) {}
    
    // 获取interval的起点
    Vertex get_start() const {
        return jump_points.front();
    }
    
    // 获取interval的终点
    Vertex get_end() const {
        return jump_points.back();
    }
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