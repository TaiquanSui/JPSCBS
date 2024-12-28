#include "JPS.h"
#include <algorithm>

JPSPath jump_point_search(const Vertex& start, const Vertex& goal, const std::vector<std::vector<int>>& grid, JPSState& state) {
    if (!is_walkable(grid, start.x, start.y) || !is_walkable(grid, goal.x, goal.y)) {
        state.is_complete = true;
        return JPSPath({}, {}, {});
    }

    if (state.open_list.empty()) {
        auto start_node = std::make_shared<AStarNode>(
            start, 0, heuristic(start, goal));
        state.open_list.push(start_node);
    }

    while (!state.open_list.empty()) {
        auto current = state.open_list.top();
        state.open_list.pop();

        if (current->pos == goal) {
            auto path = reconstruct_path(current);
            state.best_path = path.jump_points;
            state.is_complete = true;
            return path;
        }

        for (const auto& dir : Action::DIRECTIONS_8) {
            Vertex jump_point = jump(current->pos.x, current->pos.y, dir.x, dir.y, current, grid, goal);
            if (jump_point.x == -1) continue;

            int tentative_g = current->g + std::abs(current->pos.x - jump_point.x) + std::abs(current->pos.y - jump_point.y);

            auto neighbor_node = std::make_shared<AStarNode>(jump_point, tentative_g, heuristic(jump_point, goal), current);
            state.open_list.push(neighbor_node);
        }
    }

    state.is_complete = true;
    return JPSPath({}, {}, {});
} 

inline bool is_walkable(const std::vector<std::vector<int>>& grid, int x, int y) {
    return x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size() && grid[x][y] == 0;
}

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

inline Vertex calculate_direction(const Vertex& from, const Vertex& to) {
    return Vertex(
        // x方向的单位向量：1（向右）, -1（向左）或 0（竖直）
        (to.x - from.x) ? (to.x - from.x) / std::abs(to.x - from.x) : 0,
        
        // y方向的单位向量：1（向上）, -1（向下）或 0（水平）
        (to.y - from.y) ? (to.y - from.y) / std::abs(to.y - from.y) : 0
    );
}

Vertex jump(int x, int y, int dx, int dy, const std::shared_ptr<AStarNode>& current, 
           const std::vector<std::vector<int>>& grid, const Vertex& goal) {

    int nx = x + dx;
    int ny = y + dy;

    if (!is_walkable(grid, nx, ny)) return Vertex(-1, -1);
    
    Vertex next(nx, ny);
    if (next == goal) return next;
    
    if (is_in_parent_chain(current, next)) return Vertex(-1, -1);
    
    if (dx != 0 && dy != 0) {
        if (check_diagonal_forced(grid, nx, ny, dx, dy)) {
            return next;
        }
    }
    else if (check_straight_forced(grid, nx, ny, dx, dy)) {
        return next;
    }
    
    return jump(nx, ny, dx, dy, current, grid, goal);
}

bool check_diagonal_forced(const std::vector<std::vector<int>>& grid, int x, int y, int dx, int dy) {
    return (!is_walkable(grid, x - dx, y) && is_walkable(grid, x - dx, y + dy)) ||
           (!is_walkable(grid, x, y - dy) && is_walkable(grid, x + dx, y - dy));
}

bool check_straight_forced(const std::vector<std::vector<int>>& grid, int x, int y, int dx, int dy) {
    if (dx != 0) {
        return !is_walkable(grid, x, y + 1) || !is_walkable(grid, x, y - 1);
    }
    return !is_walkable(grid, x + 1, y) || !is_walkable(grid, x - 1, y);
}

JPSPath reconstruct_path(const std::shared_ptr<AStarNode>& goal_node) {
    std::vector<Vertex> path;
    std::vector<Vertex> jump_points;
    std::vector<Interval> possible_intervals;
    
    // 构建跳点列表
    auto node = goal_node;
    while (node) {
        jump_points.push_back(node->pos);
        node = node->parent;
    }
    std::reverse(jump_points.begin(), jump_points.end());
    
    // 构建完整路径
    for (size_t i = 0; i < jump_points.size() - 1; ++i) {
        Vertex curr = jump_points[i];
        Vertex next = jump_points[i + 1];
        Vertex delta = calculate_direction(curr, next);
        
        while (curr != next) {
            path.push_back(curr);
            curr = Vertex(curr.x + delta.x, curr.y + delta.y);
        }
    }
    if (!jump_points.empty()) {
        path.push_back(jump_points.back());
    }
    
    // 寻找可能的对称区间
    for (size_t i = 0; i < jump_points.size() - 1; i++) {
        Vertex v_i = jump_points[i];
        Vertex v_next = jump_points[i + 1];
        Vertex action1 = calculate_direction(v_i, v_next);
        
        // 如果第一个动作是直线移动，跳过
        if (action1.x == 0 || action1.y == 0) {
            continue;
        }
        
        // 检查是否有足够的点来形成区间
        if (i + 2 < jump_points.size()) {
            Vertex v_next2 = jump_points[i + 2];
            Vertex action2 = calculate_direction(v_next, v_next2);
            
            // 如果第二个动作是对角线移动，跳过
            if (action2.x != 0 && action2.y != 0) {
                continue;
            }
            
            // 如果第二个动作是直线移动
            if (action2.x == 0 || action2.y == 0) {
                size_t j = i + 2;
                Vertex prev_action = action2;
                
                while (j + 1 < jump_points.size()) {
                    Vertex current_action = calculate_direction(jump_points[j], jump_points[j + 1]);
                    
                    if (!(current_action == action1 || current_action == action2)) {
                        if (prev_action.x != 0 && prev_action.y != 0) {
                            possible_intervals.emplace_back(v_i, jump_points[j - 1]);
                            i = j - 1;
                        } else {
                            possible_intervals.emplace_back(v_i, jump_points[j]);
                            i = j;
                        }
                        break;
                    }
                    prev_action = current_action;
                    j++;
                }
            }
        }
    }
    
    return JPSPath(path, jump_points, possible_intervals);
}



