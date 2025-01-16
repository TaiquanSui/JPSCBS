#include "AStar.h"
#include <memory>

namespace {
    std::vector<Vertex> reconstruct_path(const std::shared_ptr<AStarNode>& goal_node) {
        std::vector<Vertex> path;
        auto node = goal_node;
        while (node) {
            path.push_back(node->pos);
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    bool check_constraints(const std::vector<Constraint>& constraints, 
                         int agent_id, const Vertex& pos, int time) {
        for (const auto& constraint : constraints) {
            if (constraint.agent == agent_id && 
                constraint.vertex == pos && 
                constraint.time == time) {
                return true;
            }
        }
        return false;
    }
}

std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal,
                          const std::vector<std::vector<int>>& grid) {
    std::priority_queue<std::shared_ptr<AStarNode>, 
                       std::vector<std::shared_ptr<AStarNode>>, 
                       AStarNodeComparator> open_list;
    std::unordered_map<Vertex, double, VertexHash> closed_list;

    auto start_node = std::make_shared<AStarNode>(start, 0, heuristic(start, goal));
    open_list.push(start_node);

    while (!open_list.empty()) {
        auto current = open_list.top();
        open_list.pop();

        if (current->pos == goal) {
            return reconstruct_path(current);
        }

        if (closed_list.count(current->pos) && closed_list[current->pos] <= current->g) {
            continue;
        }
        closed_list[current->pos] = current->g;

        for (const auto& move : Action::MOVEMENTS_9) {
            Vertex neighbor(current->pos.x + move.x, current->pos.y + move.y);

            if (!utils::isWalkable(grid, neighbor)) {
                continue;
            }

            double tentative_g = current->g + utils::getMoveCost(current->pos, neighbor);

            if (closed_list.count(neighbor) && closed_list[neighbor] <= tentative_g) {
                continue;
            }

            auto neighbor_node = std::make_shared<AStarNode>(
                neighbor, tentative_g, heuristic(neighbor, goal), current);
            open_list.push(neighbor_node);
        }
    }

    return {};
}

std::vector<Vertex> a_star(int agent_id,
                          const Vertex& start,
                          const Vertex& goal,
                          const std::vector<std::vector<int>>& grid,
                          const std::vector<Constraint>& constraints,
                          int start_time) {
    std::priority_queue<std::shared_ptr<AStarNode>, 
                       std::vector<std::shared_ptr<AStarNode>>, 
                       AStarNodeComparator> open_list;
    std::unordered_map<Vertex, double, VertexHash> closed_list;

    auto start_node = std::make_shared<AStarNode>(
        start, 0, heuristic(start, goal), nullptr, start_time);
    open_list.push(start_node);

    while (!open_list.empty()) {
        auto current = open_list.top();
        open_list.pop();

        if (current->pos == goal) {
            return reconstruct_path(current);
        }

        if (closed_list.count(current->pos) && closed_list[current->pos] <= current->g) {
            continue;
        }
        closed_list[current->pos] = current->g;

        if (check_constraints(constraints, agent_id, current->pos, current->time)) {
            continue;
        }

        for (const auto& move : Action::MOVEMENTS_9) {
            Vertex next_pos(current->pos.x + move.x, current->pos.y + move.y);
            int next_time = current->time + 1;

            if (!utils::isWalkable(grid, next_pos)) {
                continue;
            }

            if (check_constraints(constraints, agent_id, next_pos, next_time)) {
                continue;
            }

            double tentative_g = current->g + utils::getMoveCost(current->pos, next_pos);

            if (closed_list.count(next_pos) && closed_list[next_pos] <= tentative_g) {
                continue;
            }
            closed_list[next_pos] = tentative_g;

            auto next_node = std::make_shared<AStarNode>(
                next_pos, tentative_g, heuristic(next_pos, goal), current, next_time);
            open_list.push(next_node);
        }
    }

    return {};
}