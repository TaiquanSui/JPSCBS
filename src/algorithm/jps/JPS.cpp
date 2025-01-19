#include "../utilities/Utility.h"
#include "JPS.h"
#include <algorithm>

namespace {
    inline bool is_in_parent_chain(const std::shared_ptr<AStarNode>& current, const Vertex& point) {
        auto temp = current;
        while (temp) {
            if (temp->pos == point) {
                return true;
            }
            temp = temp->parent;
        }
        return false;
    }

    bool check_diagonal_forced(const std::vector<std::vector<int>>& grid, int x, int y, int dx, int dy) {
        // Check two possible forced neighbors
        bool forced1 = !utils::isWalkable(grid, x - dx, y) && 
                       utils::isWalkable(grid, x - dx, y + dy);
        bool forced2 = !utils::isWalkable(grid, x, y - dy) && 
                       utils::isWalkable(grid, x + dx, y - dy);
        return forced1 || forced2;
    }

    bool check_straight_forced(const std::vector<std::vector<int>>& grid, int x, int y, int dx, int dy) {
        if (dx != 0) {
            return (!utils::isWalkable(grid, x, y - 1) && utils::isWalkable(grid, x + dx, y - 1)) ||
                   (!utils::isWalkable(grid, x, y + 1) && utils::isWalkable(grid, x + dx, y + 1));
        } else {
            return (!utils::isWalkable(grid, x - 1, y) && utils::isWalkable(grid, x - 1, y + dy)) ||
                   (!utils::isWalkable(grid, x + 1, y) && utils::isWalkable(grid, x + 1, y + dy));
        }
    }

    Vertex jump(int x, int y, int dx, int dy, const std::shared_ptr<AStarNode>& current,
                const std::vector<std::vector<int>>& grid, const Vertex& goal) {
                
        // 1: n ← step(x, d~)
        Vertex next(x + dx, y + dy);
    
        // 2: if n is an obstacle or is outside the grid then
        // 3: return null
        if (!utils::isWalkable(grid, next.x, next.y)) 
            return Vertex(-1, -1);
    
        // 4: if n = g then
        // 5: return n
        if (next == goal) 
            return next;
    
        // 6: if ∃ n′ ∈ neighbours(n) s.t. n′ is forced then
        // 7: return n
        if (dx != 0 && dy != 0) {  // Diagonal move
            if(check_diagonal_forced(grid, next.x, next.y, dx, dy) && !is_in_parent_chain(current, next))
                return next;
            // 8: if d~ is diagonal then
            // 9: for all i ∈ {1, 2} do
            // 10: if jump(n, d~i, s, g) is not null then
            // 11: return n
            if (jump(next.x, next.y, dx, 0, current, grid, goal).x != -1 ||
                jump(next.x, next.y, 0, dy, current, grid, goal).x != -1) {
                if (!is_in_parent_chain(current, next)) {
                    return next;
                }
            }
        }else{
            if(check_straight_forced(grid, next.x, next.y, dx, dy) && !is_in_parent_chain(current, next))
                return next;
        }
    
        // 12: return jump(n, d~, s, g)
        return jump(next.x, next.y, dx, dy, current, grid, goal);
    }
    
    
    JPSPath reconstruct_path(const std::shared_ptr<AStarNode>& goal_node) {
        // 1. Collect jump points and build basic path
        std::vector<Vertex> all_jump_points;
        std::vector<Vertex> path;
        {
            auto node = goal_node;
            while (node) {
                all_jump_points.push_back(node->pos);
                node = node->parent;
            }
            std::reverse(all_jump_points.begin(), all_jump_points.end());
        }
    
        // 2. Build full path (insert intermediate points between jump points)
        if (!all_jump_points.empty()) {
            for (size_t i = 0; i < all_jump_points.size() - 1; ++i) {
                const Vertex& curr_point = all_jump_points[i];
                const Vertex& next_point = all_jump_points[i + 1];
                Vertex delta = utils::calculateDirection(curr_point, next_point);
    
                // Add all points between current jump point and next jump point
                Vertex curr = curr_point;
                while (curr != next_point) {
                    path.push_back(curr);
                    curr = Vertex(curr.x + delta.x, curr.y + delta.y);
                }
            }
            path.push_back(all_jump_points.back());  // Add goal
        }
    
        // 3. Identify symmetric intervals and key jump points
        std::vector<Vertex> jump_points;
        std::vector<Interval> possible_intervals;
        
        for (size_t i = 0; i < all_jump_points.size() - 2; ++i) {
            const auto& current = all_jump_points[i];
            const auto& next = all_jump_points[i + 1];
            const auto& next2 = all_jump_points[i + 2];
    
            // Get key action direction
            Vertex dir1 = utils::calculateDirection(current, next);
            Vertex dir2 = utils::calculateDirection(next, next2);
    
            // If not diagonal-straight pattern, add current jump point directly
            if (utils::isStraight(dir1) || utils::isDiagonal(dir2)) {
                jump_points.push_back(current);
                continue;
            }
    
            // Find symmetric interval
            for (size_t j = i + 2; j + 1 < all_jump_points.size(); ++j) {
                const auto& curr_point = all_jump_points[j];
                const auto& next_point = all_jump_points[j + 1];
                Vertex curr_dir = utils::calculateDirection(curr_point, next_point);
    
                // Check if reached interval end
                if (curr_dir != dir1 && curr_dir != dir2) {
                    // Determine interval end position
                    bool is_diagonal_end = utils::isDiagonal(all_jump_points[j-1], curr_point);
                    const auto& interval_end = is_diagonal_end ? all_jump_points[j - 1] : curr_point;
                    
                    // Collect all jump points in interval
                    std::vector<Vertex> interval_points;
                    for (size_t k = i; k <= (is_diagonal_end ? j - 1 : j); ++k) {
                        interval_points.push_back(all_jump_points[k]);
                    }
                    
                    // Record interval and update index
                    possible_intervals.emplace_back(interval_points);
                    i = j - (is_diagonal_end ? 1 : 0);
                    break;
                }
            }
    
            // Add current jump point regardless of whether an interval is found
            jump_points.push_back(current);
        }
    
        // Add last two jump points
        if (all_jump_points.size() >= 2) {
            jump_points.push_back(all_jump_points[all_jump_points.size() - 2]);
        }
        if (!all_jump_points.empty()) {
            jump_points.push_back(all_jump_points.back());
        }
    
        return JPSPath(path, jump_points, possible_intervals);
    }
    
    // Get directions to explore
    std::vector<Vertex> get_pruned_neighbors(const std::shared_ptr<AStarNode>& current,
                             const std::vector<std::vector<int>>& grid) {
        std::vector<Vertex> neighbors;
        const Vertex& pos = current->pos;
        
        // If starting node, explore all 8 directions
        if (!current->parent) {
            for (const auto& move : Action::DIRECTIONS_8) {
                if (utils::isWalkable(grid, pos.x + move.x, pos.y + move.y)) {
                    neighbors.push_back(move);
                }
            }
            return neighbors;
        }
    
        // Calculate incoming direction
        Vertex dir = utils::calculateDirection(current->parent->pos, current->pos);
        
        if (utils::isDiagonal(dir)) {  // Diagonal move
            // Natural neighbours for diagonal moves:
            // 1. Current diagonal direction
            // 2. Horizontal and vertical components
            if (utils::isWalkable(grid, pos.x + dir.x, pos.y + dir.y)) {
                neighbors.push_back(dir);
            }
            // Horizontal and vertical components are only added if they are walkable
            // Forced neighbours
            // When the path length through a non-natural neighbor is shorter than through current
            if (utils::isWalkable(grid, pos.x + dir.x, pos.y)) {
                neighbors.push_back(Vertex(dir.x, 0));
                if (!utils::isWalkable(grid, pos.x, pos.y - dir.y) && 
                    utils::isWalkable(grid, pos.x + dir.x, pos.y - dir.y)) {
                    neighbors.push_back(Vertex(dir.x, -dir.y));
                }
            }
            if (utils::isWalkable(grid, pos.x, pos.y + dir.y)) {
                neighbors.push_back(Vertex(0, dir.y));
                if (!utils::isWalkable(grid, pos.x - dir.x, pos.y) && 
                    utils::isWalkable(grid, pos.x - dir.x, pos.y + dir.y)) {
                    neighbors.push_back(Vertex(-dir.x, dir.y));
                }
            }
            
        } else {  // Straight move
            // Natural neighbours for straight moves:
            // Only current direction
            if (dir.x != 0) {  // Horizontal move
                if (utils::isWalkable(grid, pos.x + dir.x, pos.y)) {
                    neighbors.push_back(dir);
                }
                
                // Forced neighbours
                if (!utils::isWalkable(grid, pos.x, pos.y + 1) && 
                    utils::isWalkable(grid, pos.x + dir.x, pos.y + 1)) {
                    neighbors.push_back(Vertex(dir.x, 1));
                }
                if (!utils::isWalkable(grid, pos.x, pos.y - 1) && 
                    utils::isWalkable(grid, pos.x + dir.x, pos.y - 1)) {
                    neighbors.push_back(Vertex(dir.x, -1));
                }
            } else {  // vertical move
                if (utils::isWalkable(grid, pos.x, pos.y + dir.y)) {
                    neighbors.push_back(dir);
                }
                
                // Forced neighbours
                if (!utils::isWalkable(grid, pos.x + 1, pos.y) && 
                    utils::isWalkable(grid, pos.x + 1, pos.y + dir.y)) {
                    neighbors.push_back(Vertex(1, dir.y));
                }
                if (!utils::isWalkable(grid, pos.x - 1, pos.y) && 
                    utils::isWalkable(grid, pos.x - 1, pos.y + dir.y)) {
                    neighbors.push_back(Vertex(-1, dir.y));
                }
            }
        }
        
        return neighbors;
    }

}

JPSPath jump_point_search(const Vertex& start, const Vertex& goal,
                         const std::vector<std::vector<int>>& grid,
                         std::priority_queue<std::shared_ptr<AStarNode>, 
                                          std::vector<std::shared_ptr<AStarNode>>, 
                                          AStarNodeComparator>& open_list) {
    // Initialize starting node on first search
    if (open_list.empty()) {
        auto start_node = std::make_shared<AStarNode>(
            start, 0, heuristic(start, goal));
        open_list.push(start_node);
    }

    while (!open_list.empty()) {
        auto current = open_list.top();
        open_list.pop();

        if (current->pos == goal) {
            return reconstruct_path(current);
        }

        // Get pruned neighbor directions
        std::vector<Vertex> pruned_dirs = get_pruned_neighbors(current, grid);
        
        // Expand current node
        for (const auto& dir : pruned_dirs) {
            Vertex jump_point = jump(current->pos.x, current->pos.y, 
                                   dir.x, dir.y, current, grid, goal);
            if (jump_point.x == -1) continue;

            double tentative_g = current->g + utils::getMoveCost(current->pos, jump_point);

            auto next_node = std::make_shared<AStarNode>(
                jump_point, 
                tentative_g, 
                heuristic(jump_point, goal),
                current
            );
            open_list.push(next_node);
        }
    }

    return JPSPath({}, {}, {});
}




