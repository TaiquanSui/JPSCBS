#include "AStar.h"
#include "../action/Action.h"
#include <memory>

std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal, const std::vector<std::vector<int>>& grid) {
    std::priority_queue<std::shared_ptr<AStarNode>, 
                       std::vector<std::shared_ptr<AStarNode>>, 
                       AStarNodeComparator> open_list;
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

std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal, const std::vector<std::vector<int>>& grid, const std::function<bool(const Vertex&, int)>& is_valid) {
    std::priority_queue<std::shared_ptr<AStarNode>, 
                       std::vector<std::shared_ptr<AStarNode>>, 
                       AStarNodeComparator> open_list;
    std::unordered_map<Vertex, std::unordered_map<int, int>, VertexHash> closed_list;

    auto start_node = std::make_shared<AStarNode>(
        start, 0, std::abs(start.x - goal.x) + std::abs(start.y - goal.y), 0);
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

        std::vector<Vertex> directions = Action::DIRECTIONS_8;
        directions.push_back(Action::WAIT_MOVE);

        for (const auto& dir : directions) {
            Vertex neighbor(current->pos.x + dir.x, current->pos.y + dir.y);
            int next_time = current->time + 1;

            if (neighbor.x < 0 || neighbor.x >= grid.size() || 
                neighbor.y < 0 || neighbor.y >= grid[0].size() || 
                grid[neighbor.x][neighbor.y] == 1) {
                continue;
            }

            if (!is_valid(neighbor, next_time)) {
                continue;
            }

            int tentative_g = current->g + 1;

            if (closed_list[neighbor].count(next_time) && 
                closed_list[neighbor][next_time] <= tentative_g) {
                continue;
            }

            auto neighbor_node = std::make_shared<AStarNode>(
                neighbor, 
                tentative_g, 
                std::abs(neighbor.x - goal.x) + std::abs(neighbor.y - goal.y),
                next_time,
                current
            );
            open_list.push(neighbor_node);
        }
    }

    return {}; // No path found
}

