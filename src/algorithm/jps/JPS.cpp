#include "../Utility.h"
#include "JPS.h"
#include <algorithm>

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
    // 检查两个可能的强制邻居
    bool forced1 = !utils::isWalkable(grid, x - dx, y) && 
                   utils::isWalkable(grid, x - dx, y + dy);
    bool forced2 = !utils::isWalkable(grid, x, y - dy) && 
                   utils::isWalkable(grid, x + dx, y - dy);
    return forced1 || forced2;
}

bool check_straight_forced(const std::vector<std::vector<int>>& grid, int x, int y, int dx, int dy) {
    if (dx != 0) { // 水平移动
        return utils::isWalkable(grid, x, y + 1) || 
               utils::isWalkable(grid, x, y - 1);
    } else { // 垂直移动
        return utils::isWalkable(grid, x + 1, y) || 
               utils::isWalkable(grid, x - 1, y);
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

    // 防止循环
    if (is_in_parent_chain(current, next)) 
        return Vertex(-1, -1);

    // 6: if ∃ n′ ∈ neighbours(n) s.t. n′ is forced then
    // 7: return n
    if (dx != 0 && dy != 0) {  // 对角线移动
        if (check_diagonal_forced(grid, next.x, next.y, dx, dy)) {
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
    } else {  // 直线移动
        if (check_straight_forced(grid, next.x, next.y, dx, dy)) {
            return next;
        }
    }

    // 12: return jump(n, d~, s, g)
    return jump(next.x, next.y, dx, dy, current, grid, goal);
}


JPSPath reconstruct_path(const std::shared_ptr<AStarNode>& goal_node) {
    // 1. 收集跳点并构建基本路径
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

    // 2. 构建完整路径（在跳点之间插入中间点）
    if (!all_jump_points.empty()) {
        for (size_t i = 0; i < all_jump_points.size() - 1; ++i) {
            const Vertex& curr_point = all_jump_points[i];
            const Vertex& next_point = all_jump_points[i + 1];
            Vertex delta = utils::calculateDirection(curr_point, next_point);

            // 添加当前点到下一个跳点之间的所有点
            Vertex curr = curr_point;
            while (curr != next_point) {
                path.push_back(curr);
                curr = Vertex(curr.x + delta.x, curr.y + delta.y);
            }
        }
        path.push_back(all_jump_points.back());  // 添加终点
    }

    // 3. 识别对称区间和关键跳点
    std::vector<Vertex> jump_points;
    std::vector<Interval> possible_intervals;
    
    for (size_t i = 0; i < all_jump_points.size() - 2; ++i) {
        const auto& current = all_jump_points[i];
        const auto& next = all_jump_points[i + 1];
        const auto& next2 = all_jump_points[i + 2];

        // 获取关键动作方向
        Vertex dir1 = utils::calculateDirection(current, next);
        Vertex dir2 = utils::calculateDirection(next, next2);

        // 如果不是对角线-直线模式，直接添加当前跳点
        if (utils::isStraight(dir1) || utils::isDiagonal(dir2)) {
            jump_points.push_back(current);
            continue;
        }

        // 寻找对称区间
        for (size_t j = i + 2; j + 1 < all_jump_points.size(); ++j) {
            const auto& curr_point = all_jump_points[j];
            const auto& next_point = all_jump_points[j + 1];
            Vertex curr_dir = utils::calculateDirection(curr_point, next_point);

            // 检查是否到达区间终点
            if (curr_dir != dir1 && curr_dir != dir2) {
                // 确定区间终点位置
                bool is_diagonal_end = utils::isDiagonal(all_jump_points[j-1], curr_point);
                const auto& interval_end = is_diagonal_end ? all_jump_points[j - 1] : curr_point;
                
                // 记录区间和更新索引
                possible_intervals.emplace_back(current, interval_end);
                i = j - (is_diagonal_end ? 1 : 0);
                break;
            }
        }

        // 无论是否找到区间，都添加当前跳点
        jump_points.push_back(current);
    }

    // 添加最后两个跳点
    if (all_jump_points.size() >= 2) {
        jump_points.push_back(all_jump_points[all_jump_points.size() - 2]);
    }
    if (!all_jump_points.empty()) {
        jump_points.push_back(all_jump_points.back());
    }

    return JPSPath(path, jump_points, possible_intervals);
}

// 获取需要探索的方向
std::vector<Vertex> get_pruned_neighbors(const std::shared_ptr<AStarNode>& current,
                         const std::vector<std::vector<int>>& grid) {
    std::vector<Vertex> neighbors;
    const Vertex& pos = current->pos;
    
    // 如果是起始节点，探索所有8个方向
    if (!current->parent) {
        for (const auto& move : Action::DIRECTIONS_8) {
            neighbors.push_back(move);
        }
        return neighbors;
    }

    // 计算来向方向
    Vertex dir = utils::calculateDirection(current->parent->pos, current->pos);
    
    if (utils::isDiagonal(dir)) {  // 对角线方向
        // 保持对角线方向
        neighbors.push_back(dir);
        // 水平和垂直分量
        neighbors.push_back(Vertex(dir.x, 0));
        neighbors.push_back(Vertex(0, dir.y));
        
        // 检查强制邻居
        if (!utils::isWalkable(grid, pos.x - dir.x, pos.y)) {
            neighbors.push_back(Vertex(-dir.x, dir.y));
        }
        if (!utils::isWalkable(grid, pos.x, pos.y - dir.y)) {
            neighbors.push_back(Vertex(dir.x, -dir.y));
        }
    } else {  // 直线方向
        // 保持当前方向
        neighbors.push_back(dir);
        
        // 检查是否需要添加对角线方向
        if (dir.x != 0) {  // 水平移动
            if (!utils::isWalkable(grid, pos.x, pos.y + 1)) {
                neighbors.push_back(Vertex(dir.x, 1));
            }
            if (!utils::isWalkable(grid, pos.x, pos.y - 1)) {
                neighbors.push_back(Vertex(dir.x, -1));
            }
        } else {  // 垂直移动
            if (!utils::isWalkable(grid, pos.x + 1, pos.y)) {
                neighbors.push_back(Vertex(1, dir.y));
            }
            if (!utils::isWalkable(grid, pos.x - 1, pos.y)) {
                neighbors.push_back(Vertex(-1, dir.y));
            }
        }
    }
    
    return neighbors;
}

JPSPath jump_point_search(const Vertex& start, const Vertex& goal,
                         const std::vector<std::vector<int>>& grid,
                         JPSState& state) {
    // 首次搜索时初始化起始节点
    if (state.open_list.empty()) {
        auto start_node = std::make_shared<AStarNode>(
            start, 0, heuristic(start, goal));
        state.open_list.push(start_node);
    }

    while (!state.open_list.empty()) {
        auto current = state.open_list.top();
        state.open_list.pop();

        if (current->pos == goal) {
            return reconstruct_path(current);
        }

        // 获取修剪后的邻居方向
        std::vector<Vertex> pruned_dirs = get_pruned_neighbors(current, grid);
        
        // 扩展当前节点
        for (const auto& dir : pruned_dirs) {
            Vertex jump_point = jump(current->pos.x, current->pos.y, 
                                   dir.x, dir.y, current, grid, goal);
            if (jump_point.x == -1) continue;

            double tentative_g = current->g + utils::getMoveCost(current->pos, jump_point);

            auto neighbor_node = std::make_shared<AStarNode>(
                jump_point, 
                tentative_g, 
                heuristic(jump_point, goal),
                current
            );
            state.open_list.push(neighbor_node);
        }
    }

    state.is_complete = true;
    return JPSPath({}, {}, {});
}




