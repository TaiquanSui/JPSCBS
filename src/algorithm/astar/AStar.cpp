#include "AStar.h"
#include "../action/Action.h"
#include <memory> // 添加智能指针支持

std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal, const std::vector<std::vector<int>>& grid) {
    auto cmp = [](const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) { return a->f() > b->f(); };

    std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, decltype(cmp)> open_list(cmp);
    std::unordered_map<Vertex, int, VertexHash> closed_list;

    auto start_node = std::make_shared<AStarNode>(start, 0, std::abs(start.x - goal.x) + std::abs(start.y - goal.y));
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

        for (const auto& dir : Action::DIRECTIONS_8) {
            Vertex neighbor(current->pos.x + dir.x, current->pos.y + dir.y);

            if (neighbor.x < 0 || neighbor.x >= grid.size() || neighbor.y < 0 || neighbor.y >= grid[0].size() || grid[neighbor.x][neighbor.y] == 1) {
                continue;
            }

            int tentative_g = current->g + 1;

            if (closed_list.count(neighbor) && closed_list[neighbor] <= tentative_g) {
                continue;
            }

            auto neighbor_node = std::make_shared<AStarNode>(neighbor, tentative_g, std::abs(neighbor.x - goal.x) + std::abs(neighbor.y - goal.y), current);
            open_list.push(neighbor_node);
        }
    }

    return {}; // No path found
}

