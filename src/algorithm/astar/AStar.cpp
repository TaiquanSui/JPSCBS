#include "AStar.h"
#include <memory>

// 基本的A*寻路算法实现
std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal, const std::vector<std::vector<int>>& grid) {
    std::priority_queue<std::shared_ptr<AStarNode>, 
                       std::vector<std::shared_ptr<AStarNode>>, 
                       AStarNodeComparator> open_list;                  
    std::unordered_map<Vertex, int, VertexHash> closed_list;

    auto start_node = std::make_shared<AStarNode>(start, 0, heuristic(start, goal), nullptr, 0);
    open_list.push(start_node);

    while (!open_list.empty()) {
        auto current = open_list.top();
        open_list.pop();

        if (current->pos == goal) {
            std::vector<Vertex> path;
            while (current) {
                path.push_back(current->pos);
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        closed_list[current->pos] = current->g;

        for (const auto& move : Action::MOVEMENTS_9) {
            Vertex neighbor(current->pos.x + move.x, current->pos.y + move.y);

            if (!utils::isWalkable(grid, neighbor)) {
                continue;
            }

            // 计算从起点经过当前节点到邻居节点的代价
            double tentative_g = current->g + utils::getMoveCost(current->pos, neighbor);

            // 如果邻居节点在closed_list中且新路径不更优,则跳过
            if (closed_list.count(neighbor) && closed_list[neighbor] <= tentative_g) {
                continue;
            }

            // 创建新的邻居节点并加入open_list
            auto neighbor_node = std::make_shared<AStarNode>(neighbor, tentative_g, heuristic(neighbor, goal), current, 0);
            open_list.push(neighbor_node);
        }
    }

    return {}; // 没有找到路径
}

std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal, const std::vector<std::vector<int>>& grid, const std::function<bool(const Vertex&, int)>& is_valid, int start_time = 0) {
    std::priority_queue<std::shared_ptr<AStarNode>, 
                       std::vector<std::shared_ptr<AStarNode>>, 
                       AStarNodeComparator> open_list;
    std::unordered_map<Vertex, std::unordered_map<int, int>, VertexHash> closed_list;

    auto start_node = std::make_shared<AStarNode>(
        start, 0, heuristic(start, goal), nullptr, start_time);
    open_list.push(start_node);

    while (!open_list.empty()) {
        auto current = open_list.top();
        open_list.pop();

        if (current->pos == goal) {
            std::vector<Vertex> path;
            while (current) {
                path.push_back(current->pos);
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        if (closed_list[current->pos].count(current->time) && 
            closed_list[current->pos][current->time] <= current->g) {
            continue;
        }
        closed_list[current->pos][current->time] = current->g;

        std::vector<Vertex> movements = Action::MOVEMENTS_9;

        for (const auto& move : movements) {
            Vertex neighbor(current->pos.x + move.x, current->pos.y + move.y);
            int next_time = current->time + 1;

            if (!utils::isWalkable(grid, neighbor)) {
                continue;
            }

            if (!is_valid(neighbor, next_time)) {
                continue;
            }

            double tentative_g = current->g + utils::getMoveCost(current->pos, neighbor);

            if (closed_list[neighbor].count(next_time) && 
                closed_list[neighbor][next_time] <= tentative_g) {
                continue;
            }

            int h_value = heuristic(neighbor, goal);

            auto neighbor_node = std::make_shared<AStarNode>(
                neighbor, 
                tentative_g, 
                h_value,
                current,
                next_time
            );
            open_list.push(neighbor_node);
        }
    }

    return {}; // No path found
}

