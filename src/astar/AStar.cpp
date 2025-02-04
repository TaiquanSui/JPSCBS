#include "AStar.h"
#include "../utilities/Log.h"
#include "../utilities/Utility.h"
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
}

std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal,
                          const std::vector<std::vector<int>>& grid) {

    if(!utils::isWalkable(grid, start) || !utils::isWalkable(grid, goal)) {
        return {};
    }
    std::priority_queue<std::shared_ptr<AStarNode>, 
                       std::vector<std::shared_ptr<AStarNode>>, 
                       AStarNodeComparator> open_list;
    std::unordered_map<Vertex, double> closed_list;


    auto start_node = std::make_shared<AStarNode>(start, 0, heuristic(start, goal));
    open_list.push(start_node);

    while (!open_list.empty()) {
        auto current = open_list.top();
        open_list.pop();

        // logger::log_info("Current node: (" + std::to_string(current->pos.x) + "," +
        //                std::to_string(current->pos.y) + "), g值: " +
        //                std::to_string(current->g) + ", h值: " +
        //                std::to_string(current->h));

        if (current->pos == goal) {
            auto path = reconstruct_path(current);
            // logger::log_info("Path found: " + logger::vectorToString(path));
            return path;
        }

        for (const auto& move : Action::MOVEMENTS_9) {
            Vertex next_pos(current->pos.x + move.x, current->pos.y + move.y);

            if (!utils::isWalkable(grid, next_pos)) {
                continue;
            }

            double tentative_g;
            if (utils::isDiagonal(next_pos - current->pos)) {
                tentative_g = current->g + std::sqrt(2.0);
            } else {
                tentative_g = current->g + 1.0;
            }

            if (closed_list.count(next_pos) && closed_list[next_pos] <= tentative_g) {
                continue;
            }

            closed_list[next_pos] = tentative_g;

            auto next_node = std::make_shared<AStarNode>(
                next_pos, tentative_g, heuristic(next_pos, goal), current);
            
            // logger::log_info("Generated successor node: (" + std::to_string(neighbor.x) + "," +
            //                std::to_string(neighbor.y) + "), g-value: " +
            //                std::to_string(tentative_g) +
            //                ", h-value: " + std::to_string(heuristic(neighbor, goal)));
            
            open_list.push(next_node);
        }
    }

    logger::log_info("No path found");
    return {};
}

std::vector<Vertex> a_star(int agent_id,
                          const Vertex& start,
                          const Vertex& goal,
                          const std::vector<std::vector<int>>& grid,
                          const std::vector<Constraint>& constraints,
                          int start_time,
                          const ConflictAvoidanceTable& cat) {

    if(!utils::isWalkable(grid, start) || !utils::isWalkable(grid, goal)) {
        return {};
    }

    const int MAX_TIME = 100000;  // 设置一个合理的最大时间限制

    
    std::priority_queue<std::shared_ptr<AStarNode>, 
                       std::vector<std::shared_ptr<AStarNode>>, 
                       AStarNodeComparator> open_list;
    
    // 修改closed_list的键类型，加入时间维度
    std::unordered_map<StateKey, double, StateKeyHash> closed_list;

    auto start_node = std::make_shared<AStarNode>(
        start, 0, heuristic(start, goal), nullptr, start_time);
    open_list.push(start_node);

    while (!open_list.empty()) {
        auto current = open_list.top();
        open_list.pop();

        // logger::log_info("Current node: (" + std::to_string(current->pos.x) + "," +
        //                std::to_string(current->pos.y) + "), g-value: " +
        //                std::to_string(current->g) + ", h-value: " +
        //                std::to_string(current->h) + ", time: " +
        //                std::to_string(current->time));

        // 如果超过最大时间限制，认为无解
        if (current->time >= MAX_TIME) {
            logger::log_info("Agent " + std::to_string(agent_id) + 
                           " search exceeded maximum time limit, no solution found");
            return {};
        }

        if (current->pos == goal) {
            auto path = reconstruct_path(current);
            // logger::log_info("Agent " + std::to_string(agent_id) + " found path: " + 
            //                logger::vectorToString(path));
            return path;
        }

        for (const auto& move : Action::MOVEMENTS_9) {
            Vertex next_pos(current->pos.x + move.x, current->pos.y + move.y);
            int next_time = current->time + 1;

            // 检查是否超过时间限制
            if (next_time >= MAX_TIME) {
                continue;
            }

            if (!utils::isWalkable(grid, next_pos)) {
                continue;
            }

            if (!utils::is_valid_move(constraints, agent_id, next_pos, next_time)) {
                continue;
            }

            double tentative_g;
            if (utils::isDiagonal(next_pos - current->pos)) {
                tentative_g = current->g + std::sqrt(2.0);
            } else {
                tentative_g = current->g + 1.0;
            }
            StateKey state_key{next_pos, next_time};

            if (closed_list.count(state_key) && closed_list[state_key] <= tentative_g) {
                continue;
            }
            
            closed_list[state_key] = tentative_g;
            
            int conflicts = cat.getConflictCount(next_pos, next_time);
            auto next_node = std::make_shared<AStarNode>(
                next_pos, tentative_g, heuristic(next_pos, goal), current, next_time, conflicts);

            // logger::log_info("Generated successor node: (" + std::to_string(next_pos.x) + "," +
            //                std::to_string(next_pos.y) + "), g-value: " +
            //                std::to_string(tentative_g) +
            //                ", h-value: " + std::to_string(heuristic(next_pos, goal)) +
            //                ", time: " + std::to_string(next_time));
            
            open_list.push(next_node);
        }
    }

    // logger::log_info("Agent " + std::to_string(agent_id) + " found no path");
    return {};
}