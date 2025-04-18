#include "JPS.h"
#include "../utilities/GridUtility.h"
#include "../utilities/Log.h"
#include "../heuristic/Heuristic.h"
#include "../action/Action.h"
#include "../utilities/ConstraintUtility.h"
#include <algorithm>
#include <unordered_map>
#include <sstream>
#include <chrono>

namespace {
    inline bool is_cycle_free(const std::shared_ptr<AStarNode>& current, const Vertex& point) {
        // 检查从parent到current的完整路径
        if (!current->parent) return true;
        
        // 获取从parent到current的完整路径
        auto parent = current->parent;
        
        // 快速检查点是否在当前路径段的坐标范围内
        bool in_x_range = (point.x >= std::min(parent->pos.x, current->pos.x) && 
                          point.x <= std::max(parent->pos.x, current->pos.x));
        bool in_y_range = (point.y >= std::min(parent->pos.y, current->pos.y) && 
                          point.y <= std::max(parent->pos.y, current->pos.y));
        
        // 如果点不在坐标范围内，直接检查父节点
        if (!in_x_range || !in_y_range) {
            return is_cycle_free(parent, point);
        }
        
        // 点在范围内，比较斜率
        Vertex dir_to_parent = utils::calculateDirection(point, parent->pos);
        Vertex dir_to_current = utils::calculateDirection(point, current->pos);
        
        // 如果方向相反，说明点在路径上
        if (dir_to_parent.x == -dir_to_current.x && dir_to_parent.y == -dir_to_current.y) {
            return false;
        }
        
        return is_cycle_free(parent, point);
    }

    std::vector<Vertex> check_diagonal_forced(const std::vector<std::vector<int>>& grid, int x, int y, int dx, int dy) {
        std::vector<Vertex> forced_neighbors;
        Vertex from(x,y);
        Vertex neighbour1(x - dx, y);
        Vertex forced_neighbour1(x - dx, y + dy);
        Vertex neighbour2(x, y - dy);
        Vertex forced_neighbour2(x + dx, y - dy);
        
        if (!utils::isWalkable(grid, from, neighbour1) && utils::isWalkable(grid, from, forced_neighbour1)) {
            forced_neighbors.push_back(Vertex(-dx, dy));
        }

        if (!utils::isWalkable(grid, from, neighbour2) && utils::isWalkable(grid, from, forced_neighbour2)) {
            forced_neighbors.push_back(Vertex(dx, -dy));
        }

        return forced_neighbors;
    }

    std::vector<Vertex> check_straight_forced(const std::vector<std::vector<int>>& grid, int x, int y, int dx, int dy) {
        std::vector<Vertex> forced_neighbors;
        Vertex from(x, y);
        
        if (dx != 0) {  // 垂直移动（南/北）
            Vertex neighbour1(x, y - 1);  // 西边邻居
            Vertex forced_neighbour1(x + dx, y - 1);  // 西边的强制邻居
            
            if (!utils::isWalkable(grid, from, neighbour1) && 
                utils::isWalkable(grid, from, forced_neighbour1)) {
                forced_neighbors.push_back(Vertex(dx, -1));
            }
            
            Vertex neighbour2(x, y + 1);  // 东边邻居
            Vertex forced_neighbour2(x + dx, y + 1);  // 东边的强制邻居
            
            if (!utils::isWalkable(grid, from, neighbour2) && 
                utils::isWalkable(grid, from, forced_neighbour2)) {
                forced_neighbors.push_back(Vertex(dx, 1));
            }
            
        } else {  // 水平移动（东/西）
            Vertex neighbour1(x - 1, y);  // 北边邻居
            Vertex forced_neighbour1(x - 1, y + dy);  // 北边的强制邻居
            
            if (!utils::isWalkable(grid, from, neighbour1) && 
                utils::isWalkable(grid, from, forced_neighbour1)) {
                forced_neighbors.push_back(Vertex(-1, dy));
            }
            
            Vertex neighbour2(x + 1, y);  // 南边邻居
            Vertex forced_neighbour2(x + 1, y + dy);  // 南边的强制邻居
            
            if (!utils::isWalkable(grid, from, neighbour2) && 
                utils::isWalkable(grid, from, forced_neighbour2)) {
                forced_neighbors.push_back(Vertex(1, dy));
            }
        }
        
        return forced_neighbors;
    }
    
    std::vector<Vertex> get_pruned_neighbors(const std::shared_ptr<AStarNode>& current,
                             const std::vector<std::vector<int>>& grid) {
        std::vector<Vertex> neighbors;
        const Vertex& pos = current->pos;
        
        // If starting node, explore all 8 directions
        if (!current->parent) {
            for (const auto& move : Action::DIRECTIONS_8) {
                if (utils::isWalkable(grid, pos, Vertex(pos.x + move.x, pos.y + move.y))) {
                    neighbors.push_back(move);
                }
            }
            return neighbors;
        }
    
        // Calculate incoming direction
        Vertex dir = utils::calculateDirection(current->parent->pos, current->pos);

        // 1. Current diagonal direction
        if (utils::isWalkable(grid, pos, Vertex(pos.x + dir.x, pos.y + dir.y))) {
                neighbors.push_back(dir);
            }
        
        // Forced neighbors
        if (utils::isDiagonal(dir)) {
            if (utils::isWalkable(grid, pos, Vertex(pos.x + dir.x, pos.y))) {
                neighbors.push_back(Vertex(dir.x, 0));
            }
            if (utils::isWalkable(grid, pos, Vertex(pos.x, pos.y + dir.y))) {
                neighbors.push_back(Vertex(0, dir.y));
            }
            auto forced = check_diagonal_forced(grid, pos.x, pos.y, dir.x, dir.y);
            neighbors.insert(neighbors.end(), forced.begin(), forced.end());

        } else {
            auto forced = check_straight_forced(grid, pos.x, pos.y, dir.x, dir.y);
            neighbors.insert(neighbors.end(), forced.begin(), forced.end());
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
        if (!utils::isWalkable(grid, Vertex(x,y), next)) {
            return Vertex(-1, -1);
        }

        // 4: if n = g then
        // 5: return n
        if (next == goal) {
            return next;
        }

        if (!is_cycle_free(current, next)) {
            return Vertex(-1, -1);
        }
    
        // 6: if ∃ n′ ∈ neighbours(n) s.t. n′ is forced then
        // 7: return n
        if (dx != 0 && dy != 0) {  // Diagonal move
            if(!check_diagonal_forced(grid, next.x, next.y, dx, dy).empty()) {
                return next;
            }
            // 8: if d~ is diagonal then
            // 9: for all i ∈ {1, 2} do
            // 10: if jump(n, d~i, s, g) is not null then
            // 11: return n
            if (jump(next.x, next.y, dx, 0, current, grid, goal).x != -1 ||
                jump(next.x, next.y, 0, dy, current, grid, goal).x != -1) {
                return next;
            }
        } else {
            if(!check_straight_forced(grid, next.x, next.y, dx, dy).empty()) {
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
        std::vector<Vertex> all_turning_points;
        std::vector<Vertex> path;
        {
            auto node = goal_node;
            while (node) {
                all_turning_points.push_back(node->pos);
                node = node->parent;
            }
            std::reverse(all_turning_points.begin(), all_turning_points.end());
        }

        // 移除方向相同的中间跳点
        if (all_turning_points.size() > 2) {
            std::vector<Vertex> filtered_jump_points;
            filtered_jump_points.reserve(all_turning_points.size());  // 预分配内存
            filtered_jump_points.push_back(all_turning_points[0]);  // 始终保留起点

            for (size_t i = 1; i < all_turning_points.size() - 1; ++i) {
                Vertex prev_dir = utils::calculateDirection(all_turning_points[i-1], all_turning_points[i]);
                Vertex next_dir = utils::calculateDirection(all_turning_points[i], all_turning_points[i+1]);

                if (prev_dir != next_dir) {
                    filtered_jump_points.push_back(all_turning_points[i]);
                }
            }

            filtered_jump_points.push_back(all_turning_points.back());  // 始终保留终点
            all_turning_points = std::move(filtered_jump_points);  // 使用移动语义
        }
        // logger::log_info("All turning points: " + logger::vectorToString(all_turning_points));


        // 2. Build full path (insert intermediate points between jump points)
        if (!all_turning_points.empty()) {
            // 预计算路径长度并预分配内存
            size_t path_length = 1;  // 起点
            for (size_t i = 0; i < all_turning_points.size() - 1; ++i) {
                path_length += utils::octileDistance(all_turning_points[i], all_turning_points[i + 1]) + 1;
            }
            path.reserve(path_length);

            for (size_t i = 0; i < all_turning_points.size() - 1; ++i) {
                const Vertex& curr_point = all_turning_points[i];
                const Vertex& next_point = all_turning_points[i + 1];
                Vertex delta = utils::calculateDirection(curr_point, next_point);

                // Add all points between current jump point and next jump point
                Vertex curr = curr_point;
                while (curr != next_point) {
                    path.push_back(curr);
                    curr = Vertex(curr.x + delta.x, curr.y + delta.y);
                }
            }
            path.push_back(all_turning_points.back());  // Add goal
        }

        // 3. Identify symmetric intervals and key jump points
        std::vector<JumpPointWithIndex> turning_points;
        std::vector<Interval> possible_intervals;
        turning_points.reserve(all_turning_points.size());
        possible_intervals.reserve(all_turning_points.size() / 2);

        for (size_t i = 0; i < all_turning_points.size(); ++i) {
            // 找到当前跳点在完整路径中的索引
            auto path_it = std::find(path.begin(), path.end(), all_turning_points[i]);
            size_t path_index = std::distance(path.begin(), path_it);
            
            turning_points.emplace_back(all_turning_points[i], path_index);
            
            if (i + 2 >= all_turning_points.size()) {
                continue;
            }

            const auto& current = all_turning_points[i];
            const auto& next = all_turning_points[i + 1];
            const auto& next2 = all_turning_points[i + 2];
            Vertex dir1 = utils::calculateDirection(current, next);
            Vertex dir2 = utils::calculateDirection(next, next2);

            if (!utils::isDiagonal(dir1) || !utils::isStraight(dir2)) {
                continue;
            }

            // 对于对称区间，添加起点
            // 找到一个（对角线-直线）模式
            std::vector<Vertex> interval_points;
            interval_points.push_back(current);
            interval_points.push_back(next);
            interval_points.push_back(next2);

            // 继续检查后续是否有相同的模式
            size_t j = i + 2;
            while (j + 2 < all_turning_points.size()) {
                const auto& pattern_start = all_turning_points[j];
                const auto& pattern_mid = all_turning_points[j + 1];
                const auto& pattern_end = all_turning_points[j + 2];
                Vertex pattern_dir1 = utils::calculateDirection(pattern_start, pattern_mid);
                Vertex pattern_dir2 = utils::calculateDirection(pattern_mid, pattern_end);
                
                // 检查是否是相同的模式
                if (pattern_dir1 == dir1 && pattern_dir2 == dir2) {
                    // 添加到当前区间
                    interval_points.push_back(pattern_mid);
                    interval_points.push_back(pattern_end);
                    j += 2;
                } else {
                    break;
                }
            }

            // logger::log_info("Interval points: " + logger::vectorToString(interval_points));
            // 记录找到的区间
            possible_intervals.emplace_back(interval_points);
            i = j - 1;
        }

        return JPSPath(std::move(path), std::move(turning_points), std::move(possible_intervals));
    }    

}


JPSPath jump_point_search(const Vertex& start, const Vertex& goal,
                         const std::vector<std::vector<int>>& grid,
                         JPSState& state) {
    auto start_time = std::chrono::steady_clock::now();
    const int MAX_SEARCH_TIME = 300;  // 修改为 5 分钟

    std::vector<std::shared_ptr<AStarNode>> closed_list;
    std::vector<std::shared_ptr<AStarNode>> temp_nodes;

    while (!state.open_list.empty()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        if (elapsed >= MAX_SEARCH_TIME) {
            logger::log_warning("JPS search timed out after " + std::to_string(MAX_SEARCH_TIME) + " seconds");
            return JPSPath({}, {}, {});
        }

        auto current = state.open_list.top();
        state.open_list.pop();
        closed_list.push_back(current);

        // logger::log_info("Current node: (" + std::to_string(current->pos.x) + "," + 
        //                std::to_string(current->pos.y) + "), g-value: " + 
        //                std::to_string(current->g) + ", h-value: " + 
        //                std::to_string(current->h));

        if (current->pos == goal) {
            auto path = reconstruct_path(current);
            // 将所有temp_nodes放回open_list以备下次搜索
            for (auto& node : temp_nodes) {
                state.open_list.push(node);
            }
            state.closed_list.insert(state.closed_list.end(), 
                                    std::make_move_iterator(closed_list.begin()), 
                                    std::make_move_iterator(closed_list.end()));
            
            // // Add log information
            // std::stringstream ss;
            // ss << "JPS found path from (" << start.x << "," << start.y 
            //    << ") to (" << goal.x << "," << goal.y << "):" << std::endl;
            
            // // Print complete path
            // ss << "Full path: ";
            // for (size_t i = 0; i < path.path.size(); ++i) {
            //     ss << "(" << path.path[i].x << "," << path.path[i].y << ")";
            //     if (i < path.path.size() - 1) ss << ",";
            // }
            // ss << std::endl;

            
            // double cost = utils::calculate_path_cost(path.path);
            // ss << "Cost: " << cost << std::endl;
            
            // // Print jump points
            // ss << "Jump points: ";

            // for (size_t i = 0; i < path.jump_points.size(); ++i) {
            //     ss << "(" << path.jump_points[i].x << "," << path.jump_points[i].y << ")";
            //     if (i < path.jump_points.size() - 1) ss << " -> ";
            // }
            // ss << std::endl;
            
            // // Print possible_intervals
            // logger::print_intervals(path.possible_intervals, "Possible intervals: ");
            
            // logger::log_info(ss.str());
            
            return path;
        }
        
        // identify_successors
        std::vector<Vertex> successors = identify_successors(current, grid, goal);
        
        // Expand current node
        for (const auto& successor : successors) {
            // 检查successor是否在closed list中
            auto it = std::find_if(closed_list.begin(), closed_list.end(),
                [&successor](const std::shared_ptr<AStarNode>& closed_node) {
                    return closed_node->pos == successor;
                });
                
            double move_cost = utils::getMoveCost(current->pos, successor);
            double tentative_g = current->g + move_cost;
            double h_value = heuristic(successor, goal);

            auto next_node = std::make_shared<AStarNode>(
                successor, 
                tentative_g,
                h_value,
                current
            );
            // logger::log_info("Generated successor node: (" + std::to_string(successor.x) + "," + 
            //                std::to_string(successor.y) + "), g-value: " + 
            //                std::to_string(tentative_g) + ", movement cost: " + 
            //                std::to_string(move_cost) + ", h-value: " + 
            //                std::to_string(h_value));
            // auto node = current;
            
            // std::stringstream ss;   
            // ss << "parents: ";
            // while (node) {
            //     ss << "(" << node->pos.x << "," << node->pos.y << ")";
            //     node = node->parent;
            // }
            // logger::log_info(ss.str());
            
            // 如果在closed list中，放入temp_nodes
            if (it != closed_list.end()) {
                temp_nodes.push_back(next_node);
            } else {
                // 否则直接加入open_list
                state.open_list.push(next_node);
            }
        }
    }

    logger::log_warning("JPS found no path from (" + 
                      std::to_string(start.x) + "," + std::to_string(start.y) + 
                      ") to (" + std::to_string(goal.x) + "," + std::to_string(goal.y) + ")");
    return JPSPath({}, {}, {});
}

// 定义静态成员变量
const std::unordered_map<int, std::priority_queue<JPSPath, std::vector<JPSPath>, JPSPathComparator>>* JPSPathComparator::solution_context = nullptr;

bool JPSPathComparator::operator()(const JPSPath& a, const JPSPath& b) const {
    double a_cost = utils::calculate_path_cost(a.path);
    double b_cost = utils::calculate_path_cost(b.path);
    
    if (std::abs(a_cost - b_cost) < 1e-6 && solution_context) {
        int a_conflicts = evaluate_conflicts_with_all_combinations(a);
        int b_conflicts = evaluate_conflicts_with_all_combinations(b);
        
        if (a_conflicts != b_conflicts) {
            return a_conflicts > b_conflicts;
        }
        return &a > &b;
    }
    return a_cost > b_cost;
}

void JPSPathComparator::set_solution_context(const std::unordered_map<int, std::priority_queue<JPSPath, std::vector<JPSPath>, JPSPathComparator>>* ctx) {
    solution_context = ctx;
}

int JPSPathComparator::evaluate_conflicts_with_all_combinations(const JPSPath& test_path) const {
    if (!solution_context || test_path.path.empty()) return 0;
    
    // 改用存储路径而不是指针
    std::unordered_map<int, std::vector<JPSPath>> same_cost_paths;
    
    for (const auto& [agent_id, paths] : *solution_context) {
        if (paths.empty()) continue;
        
        auto temp_queue = paths;
        double first_cost = utils::calculate_path_cost(temp_queue.top().path);
        
        std::vector<JPSPath>& agent_paths = same_cost_paths[agent_id];
        while (!temp_queue.empty()) {
            JPSPath current_path = temp_queue.top();
            temp_queue.pop();
            
            if (current_path.path.empty()) continue;
            
            if (std::abs(utils::calculate_path_cost(current_path.path) - first_cost) > 1e-6) break;
            
            // 使用路径内容比较而不是指针比较
            if (current_path.path == test_path.path) {
                same_cost_paths.erase(agent_id);
                break;
            }
            
            agent_paths.push_back(std::move(current_path));
        }
    }
    
    // 计算冲突
    int min_conflicts = INT_MAX;
    std::unordered_map<int, JPSPath> current_combination;
    
    if (!same_cost_paths.empty()) {
        std::function<void(std::unordered_map<int, std::vector<JPSPath>>::iterator)> try_combinations;
        try_combinations = [&](auto it) {
            if (it == same_cost_paths.end()) {
                int conflicts = 0;
                for (const auto& [id1, path1] : current_combination) {
                    conflicts += utils::count_conflicts(test_path.path, path1.path);
                    for (const auto& [id2, path2] : current_combination) {
                        if (id1 < id2) {
                            conflicts += utils::count_conflicts(path1.path, path2.path);
                        }
                    }
                }
                min_conflicts = std::min(min_conflicts, conflicts);
                return;
            }
            
            for (const auto& path : it->second) {
                current_combination.insert_or_assign(it->first, path);  // 或者用emplace
                try_combinations(std::next(it));
            }
        };
        
        try_combinations(same_cost_paths.begin());
    }
    
    return min_conflicts == INT_MAX ? 0 : min_conflicts;
}




