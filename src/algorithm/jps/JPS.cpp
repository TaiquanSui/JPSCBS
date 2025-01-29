#include "../utilities/Utility.h"
#include "../utilities/Log.h"
#include "JPS.h"
#include <algorithm>
#include <sstream>

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
        bool forced = false;
        
        if (!utils::isWalkable(grid, x + dx, y) && utils::isWalkable(grid, x + 2*dx, y))
            forced = true;
        if (!utils::isWalkable(grid, x, y + dy) && utils::isWalkable(grid, x , y + 2*dy))
            forced = true;
        
        return forced;
    }

    bool check_straight_forced(const std::vector<std::vector<int>>& grid, int x, int y, int dx, int dy) {
        if (dx != 0) {  // 垂直移动（南/北）
            return (!utils::isWalkable(grid, x, y - 1) && utils::isWalkable(grid, x + dx, y - 1)) ||  // 检查西边
                   (!utils::isWalkable(grid, x, y + 1) && utils::isWalkable(grid, x + dx, y + 1));    // 检查东边
        } else {        // 水平移动（东/西）
            return (!utils::isWalkable(grid, x - 1, y) && utils::isWalkable(grid, x - 1, y + dy)) ||  // 检查北边
                   (!utils::isWalkable(grid, x + 1, y) && utils::isWalkable(grid, x + 1, y + dy));    // 检查南边
        }
    }
    
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

    Vertex jump(int x, int y, int dx, int dy, const std::shared_ptr<AStarNode>& current,
                const std::vector<std::vector<int>>& grid, const Vertex& goal) {
                
        // 1: n ← step(x, d~)
        Vertex next(x + dx, y + dy);
    
        // 构建基础日志信息
        std::stringstream base_info;
        base_info << "From (" << x << "," << y << ") with direction (" << dx << "," << dy << ")";

        // 2: if n is an obstacle or is outside the grid then
        // 3: return null
        if (!utils::isWalkable(grid, next.x, next.y)) {
            // logger::log_info(base_info.str() + " - Jump point rejected: (" + 
            //                std::to_string(next.x) + "," + std::to_string(next.y) + 
            //                ") - obstacle or outside grid");
            return Vertex(-1, -1);
        }
    
        // 4: if n = g then
        // 5: return n
        if (next == goal) {
            // logger::log_info(base_info.str() + " - Jump point found: (" + 
            //                std::to_string(next.x) + "," + std::to_string(next.y) + 
            //                ") - goal reached");
            return next;
        }
    
        // 6: if ∃ n′ ∈ neighbours(n) s.t. n′ is forced then
        // 7: return n
        if (dx != 0 && dy != 0) {  // Diagonal move
            if(check_diagonal_forced(grid, next.x, next.y, dx, dy) && !is_in_parent_chain(current, next)) {
                // logger::log_info(base_info.str() + " - Jump point found: (" + 
                //                std::to_string(next.x) + "," + std::to_string(next.y) + 
                //                ") - diagonal forced neighbor");
                return next;
            }
            // 8: if d~ is diagonal then
            // 9: for all i ∈ {1, 2} do
            // 10: if jump(n, d~i, s, g) is not null then
            // 11: return n
            if (jump(next.x, next.y, dx, 0, current, grid, goal).x != -1 ||
                jump(next.x, next.y, 0, dy, current, grid, goal).x != -1) {
                if (!is_in_parent_chain(current, next)) {
                    // logger::log_info(base_info.str() + " - Jump point found: (" + 
                    //                std::to_string(next.x) + "," + std::to_string(next.y) + 
                    //                ") - diagonal recursive check");
                    return next;
                }
            }
        } else {
            if(check_straight_forced(grid, next.x, next.y, dx, dy) && !is_in_parent_chain(current, next)) {
                // logger::log_info(base_info.str() + " - Jump point found: (" + 
                //                std::to_string(next.x) + "," + std::to_string(next.y) + 
                //                ") - straight forced neighbor");
                return next;
            }
        }
    
        // 12: return jump(n, d~, s, g)
        return jump(next.x, next.y, dx, dy, current, grid, goal);
    }

    std::vector<Vertex> identify_successors(const std::shared_ptr<AStarNode>& current,
                                      const std::vector<std::vector<int>>& grid,
                                      const Vertex& goal) {
        // 1: successors(x) ← ∅
        std::vector<Vertex> successors;

        // 2: neighbours(x) ← prune(x, neighbours(x))
        std::vector<Vertex> pruned_dirs = get_pruned_neighbors(current, grid);
        // logger::log_info("Pruned directions: " + logger::vectorToString(pruned_dirs));

        // 3: for all n ∈ neighbours(x) do
        for (const auto& dir : pruned_dirs) {
            // 4: n ← jump(x, direction(x, n), s, g)
            Vertex jump_point = jump(current->pos.x, current->pos.y, 
                                   dir.x, dir.y, current, grid, goal);

            // 5: add n to successors(x)
            if (jump_point.x != -1) {
                successors.push_back(jump_point);
                // logger::log_info("Jump point found: (" + std::to_string(jump_point.x) + "," + 
                //                std::to_string(jump_point.y) + ") - added to successors");
            }
        }

        // 6: return successors(x)
        return successors;
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

        // 移除方向相同的中间跳点
        if (all_jump_points.size() > 2) {
            std::vector<Vertex> filtered_jump_points;
            filtered_jump_points.reserve(all_jump_points.size());  // 预分配内存
            filtered_jump_points.push_back(all_jump_points[0]);  // 始终保留起点

            for (size_t i = 1; i < all_jump_points.size() - 1; ++i) {
                Vertex prev_dir = utils::calculateDirection(all_jump_points[i-1], all_jump_points[i]);
                Vertex next_dir = utils::calculateDirection(all_jump_points[i], all_jump_points[i+1]);
                
                if (prev_dir.x != next_dir.x || prev_dir.y != next_dir.y) {
                    filtered_jump_points.push_back(all_jump_points[i]);
                }
            }

            filtered_jump_points.push_back(all_jump_points.back());  // 始终保留终点
            all_jump_points = std::move(filtered_jump_points);  // 使用移动语义
        }

        // 2. Build full path (insert intermediate points between jump points)
        if (!all_jump_points.empty()) {
            // 预计算路径长度并预分配内存
            size_t path_length = 1;  // 起点
            for (size_t i = 0; i < all_jump_points.size() - 1; ++i) {
                path_length += 2*utils::octileDistance(all_jump_points[i], all_jump_points[i + 1]);
            }
            path.reserve(path_length);

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
        jump_points.reserve(all_jump_points.size());
        possible_intervals.reserve(all_jump_points.size() / 2);

        for (size_t i = 0; i < all_jump_points.size() - 1; ++i) {
            // 先检查单个（对角线-直线）模式
            if (i + 2 < all_jump_points.size()) {
                const auto& current = all_jump_points[i];
                const auto& next = all_jump_points[i + 1];
                const auto& next2 = all_jump_points[i + 2];
                
                Vertex dir1 = utils::calculateDirection(current, next);
                Vertex dir2 = utils::calculateDirection(next, next2);
                
                if (utils::isDiagonal(dir1) && utils::isStraight(dir2)) {
                    // 对于对称区间，添加起点
                    jump_points.push_back(current);
                    
                    // 找到一个（对角线-直线）模式
                    std::vector<Vertex> interval_points;
                    interval_points.reserve(3);
                    interval_points.push_back(current);
                    interval_points.push_back(next);
                    interval_points.push_back(next2);
                    
                    // 继续检查后续是否有相同的模式
                    size_t j = i + 2;
                    while (j + 2 < all_jump_points.size()) {
                        const auto& pattern_start = all_jump_points[j];
                        const auto& pattern_mid = all_jump_points[j + 1];
                        const auto& pattern_end = all_jump_points[j + 2];
                        
                        Vertex pattern_dir1 = utils::calculateDirection(pattern_start, pattern_mid);
                        Vertex pattern_dir2 = utils::calculateDirection(pattern_mid, pattern_end);
                        
                        // 检查是否是相同的模式
                        if (utils::isDiagonal(pattern_dir1) && utils::isStraight(pattern_dir2) &&
                            pattern_dir1 == dir1 && pattern_dir2 == dir2) {
                            // 添加到当前区间
                            interval_points.push_back(pattern_mid);
                            interval_points.push_back(pattern_end);
                            j += 2;
                        } else {
                            break;
                        }
                    }
                    
                    // 记录找到的区间
                    possible_intervals.emplace_back(interval_points);
                    
                    // 更新索引到区间终点，下一轮从这里开始寻找新的对称区间
                    i = j;
                    continue;
                }
            }
            
            // 如果不是对称区间的起点，直接添加当前跳点
            jump_points.push_back(all_jump_points[i]);
        }

        // 添加最后一个跳点
        if (!all_jump_points.empty()) {
            jump_points.push_back(all_jump_points.back());
        }

        
        return JPSPath(std::move(path), std::move(jump_points), std::move(possible_intervals));
    }
    

}


JPSPath jump_point_search(const Vertex& start, const Vertex& goal,
                         const std::vector<std::vector<int>>& grid,
                         JPSState& state) {

    while (!state.open_list.empty()) {
        // std::stringstream ss;
        // ss << "当前open_list中的节点: ";
        
        // // 创建临时队列用于遍历
        // auto temp_queue = state.open_list;
        // while(!temp_queue.empty()) {
        //     auto node = temp_queue.top();
        //     ss << "(" << node->pos.x << "," << node->pos.y << ")[g=" 
        //        << node->g << ",h=" << node->h << "] ";
        //     temp_queue.pop();
        // }
        // logger::log_info(ss.str());

        auto current = state.open_list.top();
        state.open_list.pop();
        state.closed_list.push_back(current);

        if (current->pos == goal) {
            auto path = reconstruct_path(current);
            
            // 添加日志信息
            std::stringstream ss;
            ss << "JPS found path from (" << start.x << "," << start.y 
               << ") to (" << goal.x << "," << goal.y << "):" << std::endl;
            
            // 打印完整路径
            ss << "Full path: ";
            for (size_t i = 0; i < path.path.size(); ++i) {
                ss << "(" << path.path[i].x << "," << path.path[i].y << ")";
                if (i < path.path.size() - 1) ss << " -> ";
            }
            ss << std::endl;
            
            // 打印跳点
            ss << "Jump points: ";
            for (size_t i = 0; i < path.jump_points.size(); ++i) {
                ss << "(" << path.jump_points[i].x << "," << path.jump_points[i].y << ")";
                if (i < path.jump_points.size() - 1) ss << " -> ";
            }
            ss << std::endl;
            
            //打印possible_intervals
            logger::print_intervals(path.possible_intervals, "Possible intervals: ");
            
            logger::log_info(ss.str());
            
            return path;
        }

        // logger::log_info("Current node: (" + std::to_string(current->pos.x) + "," + 
        //                std::to_string(current->pos.y) + "), g值: " + 
        //                std::to_string(current->g) + ", h值: " + 
        //                std::to_string(current->h));
        
        // identify_successors
        std::vector<Vertex> successors = identify_successors(current, grid, goal);
        
        // Expand current node
        for (const auto& successor : successors) {
            double move_cost = utils::getMoveCost(current->pos, successor);
            double tentative_g = current->g + move_cost;
            double h_value = heuristic(successor, goal);

            auto next_node = std::make_shared<AStarNode>(
                successor, 
                tentative_g,  // 使用计算出的 tentative_g
                h_value,
                current
            );
            // logger::log_info("生成后继节点: (" + std::to_string(successor.x) + "," + 
            //                std::to_string(successor.y) + "), g值: " + 
            //                std::to_string(tentative_g) + ", 移动代价: " + 
            //                std::to_string(move_cost) + ", h值: " + 
            //                std::to_string(h_value));
            state.open_list.push(next_node);
        }
    }

    logger::log_warning("JPS found no path from (" + 
                      std::to_string(start.x) + "," + std::to_string(start.y) + 
                      ") to (" + std::to_string(goal.x) + "," + std::to_string(goal.y) + ")");
    return JPSPath({}, {}, {});
}




