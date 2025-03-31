#ifndef JPS_H
#define JPS_H

#include "../basic/Vertex.h"
#include "../astar/AStar.h"
#include <vector>
#include <memory>
#include <queue>

struct Interval {
    std::vector<Vertex> jump_points;  // Store all jump points in the interval in order

    Interval(const std::vector<Vertex>& points) : jump_points(points) {}
    
    // Get interval start point
    Vertex get_start() const {
        return jump_points.front();
    }
    
    // Get interval end point
    Vertex get_end() const {
        return jump_points.back();
    }
};

struct JumpPointWithIndex {
    Vertex pos;
    size_t path_index;  // 在path中的索引位置

    JumpPointWithIndex(const Vertex& p, size_t idx) 
        : pos(p), path_index(idx) {}
};

struct JPSPath {
    std::vector<Vertex> path;
    std::vector<JumpPointWithIndex> jump_points;
    std::vector<Interval> possible_intervals;

    JPSPath(const std::vector<Vertex>& path, 
            const std::vector<JumpPointWithIndex>& jump_points, 
            const std::vector<Interval>& possible_intervals)
        : path(path), jump_points(jump_points), possible_intervals(possible_intervals) {}
};

struct JPSPathComparator {
    bool operator()(const JPSPath& a, const JPSPath& b) const;
    static void set_solution_context(const std::unordered_map<int, std::priority_queue<JPSPath, std::vector<JPSPath>, JPSPathComparator>>* ctx);

private:
    static const std::unordered_map<int, std::priority_queue<JPSPath, std::vector<JPSPath>, JPSPathComparator>>* solution_context;
    int evaluate_conflicts_with_all_combinations(const JPSPath& test_path) const;
};

struct JPSState {
    std::priority_queue<std::shared_ptr<AStarNode>, 
                       std::vector<std::shared_ptr<AStarNode>>, 
                       AStarNodeComparator> open_list;
    std::vector<std::shared_ptr<AStarNode>> closed_list;
    
    void clear() {
        while (!open_list.empty()) {
            open_list.pop();
        }
        closed_list.clear();
    }
};

JPSPath jump_point_search(const Vertex& start, 
                         const Vertex& goal, 
                         const std::vector<std::vector<int>>& grid,
                         JPSState& state);

#endif // JPS_H 