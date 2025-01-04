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
    return (!Utility::isWalkable(grid, Vertex(x - dx, y)) && Utility::isWalkable(grid, Vertex(x - dx, y + dy))) ||
           (!Utility::isWalkable(grid, Vertex(x, y - dy)) && Utility::isWalkable(grid, Vertex(x + dx, y - dy)));
}

bool check_straight_forced(const std::vector<std::vector<int>>& grid, int x, int y, int dx, int dy) {
    if (dx != 0) {
        return !Utility::isWalkable(grid, x, y + 1) || !Utility::isWalkable(grid, x, y - 1);
    }
    return !Utility::isWalkable(grid, x + 1, y) || !Utility::isWalkable(grid, x - 1, y);
}

Vertex jump(int x, int y, int dx, int dy, const std::shared_ptr<AStarNode>& current,
            const std::vector<std::vector<int>>& grid, const Vertex& goal) {

    int nx = x + dx;
    int ny = y + dy;

    if (!Utility::isWalkable(grid, nx, ny)) return Vertex(-1, -1);

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


JPSPath reconstruct_path(const std::shared_ptr<AStarNode>& goal_node) {
    std::vector<Vertex> path;
    std::vector<Vertex> all_jump_points;
    std::vector<Interval> possible_intervals;

    // 首先收集所有跳点
    auto node = goal_node;
    while (node) {
        all_jump_points.push_back(node->pos);
        node = node->parent;
    }
    std::reverse(all_jump_points.begin(), all_jump_points.end());

    // 构建完整路径
    for (size_t i = 0; i < all_jump_points.size() - 1; ++i) {
        Vertex curr = all_jump_points[i];
        Vertex next = all_jump_points[i + 1];
        Vertex delta = Utility::calculateDirection(curr, next);

        while (curr != next) {
            path.push_back(curr);
            curr = Vertex(curr.x + delta.x, curr.y + delta.y);
        }
    }
    if (!all_jump_points.empty()) {
        path.push_back(all_jump_points.back());
    }

    // 寻找可能的对称区间
    std::vector<Vertex> jump_points;  // 最终的跳点列表
    for (size_t i = 0; i < all_jump_points.size() - 1; i++) {
        Vertex v_i = all_jump_points[i];
        Vertex v_next = all_jump_points[i + 1];
        Vertex action1 = Utility::calculateDirection(v_i, v_next);

        // 如果第一个动作是直线移动，添加当前跳点并继续
        if (action1.x == 0 || action1.y == 0) {
            jump_points.push_back(v_i);
            continue;
        }

        // 检查是否有足够的点来形成区间
        if (i + 2 < all_jump_points.size()) {
            Vertex v_next2 = all_jump_points[i + 2];
            Vertex action2 = Utility::calculateDirection(v_next, v_next2);

            // 如果第二个动作是对角线移动，添加当前跳点并继续
            if (action2.x != 0 && action2.y != 0) {
                jump_points.push_back(v_i);
                continue;
            }

            // 如果第二个动作是直线移动
            if (action2.x == 0 || action2.y == 0) {
                size_t j = i + 2;
                Vertex prev_action = action2;

                while (j + 1 < all_jump_points.size()) {
                    Vertex current_action = Utility::calculateDirection(all_jump_points[j], all_jump_points[j + 1]);

                    if (!(current_action == action1 || current_action == action2)) {
                        if (prev_action.x != 0 && prev_action.y != 0) {
                            possible_intervals.emplace_back(v_i, all_jump_points[j - 1]);
                            jump_points.push_back(v_i);  // 添加区间起点
                            i = j - 1;
                        } else {
                            possible_intervals.emplace_back(v_i, all_jump_points[j]);
                            jump_points.push_back(v_i);  // 添加区间起点
                            i = j;
                        }
                        break;
                    }
                    prev_action = current_action;
                    j++;
                }
            }
        } else {
            jump_points.push_back(v_i);
        }
    }

    // 添加最后一个跳点
    if (!all_jump_points.empty()) {
        jump_points.push_back(all_jump_points.back());
    }

    return JPSPath(path, jump_points, possible_intervals);
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

    // 继续搜索直到找到下一个可行路径或搜索完所有可能
    while (!state.open_list.empty()) {
        auto current = state.open_list.top();
        state.open_list.pop();

        if (current->pos == goal) {
            auto path = reconstruct_path(current);
            return path;
        }

        // 扩展当前节点
        for (const auto& dir : Action::DIRECTIONS_8) {
            Vertex jump_point = jump(current->pos.x, current->pos.y, 
                                   dir.x, dir.y, current, grid, goal);
            if (jump_point.x == -1) continue;

            int tentative_g = current->g + std::abs(current->pos.x - jump_point.x) 
                                       + std::abs(current->pos.y - jump_point.y);

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
    return JPSPath({}, {}, {});  // 没有更多路径
}




