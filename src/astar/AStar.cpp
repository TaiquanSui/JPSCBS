#include "AStar.h"
#include "../utilities/Log.h"
#include "../utilities/GridUtility.h"
#include "../heuristic/Heuristic.h"
#include "../action/Action.h"
#include <chrono> 
#include <queue> 
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <functional>

namespace {
    const int MAX_SEARCH_TIME = 30;  // 最大搜索时间（秒）
    
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
    // 记录开始时间
    auto start_time = std::chrono::steady_clock::now();

    if(!utils::isPassable(grid, start) || !utils::isPassable(grid, goal)) {
        return {};
    }
    std::priority_queue<std::shared_ptr<AStarNode>, 
                       std::vector<std::shared_ptr<AStarNode>>, 
                       AStarNodeComparator> open_list;
    std::unordered_map<Vertex, double> closed_list;


    auto start_node = std::make_shared<AStarNode>(start, 0, heuristic(start, goal));
    open_list.push(start_node);

    while (!open_list.empty()) {
        // 检查是否超时
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        if (elapsed >= MAX_SEARCH_TIME) {
            logger::log_warning("A* search timed out after " + std::to_string(MAX_SEARCH_TIME) + " seconds");
            return {};
        }

        auto current = open_list.top();
        open_list.pop();

        // logger::log_info("Current node: (" + std::to_string(current->pos.x) + "," +
        //                std::to_string(current->pos.y) + "), g-value: " +
        //                std::to_string(current->g) + ", h-value: " +
        //                std::to_string(current->h));  

        if (current->pos == goal) {
            auto path = reconstruct_path(current);
            // logger::log_info("Path found: " + logger::vectorToString(path));
            // double cost = utils::calculate_path_cost(path);
            // logger::log_info("Cost: " + std::to_string(cost));
            return path;
        }

        for (const auto& move : Action::MOVEMENTS_9) {
            Vertex next_pos(current->pos.x + move.x, current->pos.y + move.y);

            if (!utils::isWalkable(grid, current->pos, next_pos)) {
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
                          const int start_time,
                          const ConflictAvoidanceTable& cat) {
    // 记录开始时间
    auto search_start_time = std::chrono::steady_clock::now();

    if(!utils::isPassable(grid, start) || !utils::isPassable(grid, goal)) {
        return {};
    }

    std::priority_queue<std::shared_ptr<AStarNode>, 
                       std::vector<std::shared_ptr<AStarNode>>, 
                       AStarNodeComparator> open_list;
    std::unordered_map<StateKey, double, StateKeyHash> closed_list;

    auto start_node = std::make_shared<AStarNode>(
        start, 0, heuristic(start, goal), nullptr, start_time);
    open_list.push(start_node);

    while (!open_list.empty()) {
        // 检查是否超时
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - search_start_time).count();
        if (elapsed >= MAX_SEARCH_TIME) {
            logger::log_warning("Agent " + std::to_string(agent_id) + 
                              " A* search timed out after " + std::to_string(MAX_SEARCH_TIME) + " seconds");
            return {};
        }

        auto current = open_list.top();
        open_list.pop();

        if(start == Vertex(74,40) && goal == Vertex(185,115)){
        logger::log_info("Current node: (" + std::to_string(current->pos.x) + "," +
               std::to_string(current->pos.y) + "), g: " +
               std::to_string(current->g) + ", h: " +
               std::to_string(current->h) + ", f: " +
               std::to_string(current->f()) + ", time: " +
               std::to_string(current->time));
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

            if (!utils::isWalkable(grid, current->pos, next_pos)) {
                continue;
            }

            // 检查约束违反（包括顶点和边约束）
            if (violatesConstraints(current->pos, next_pos, agent_id, current->time, constraints)) {
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
            
            int conflicts = cat.getConflictCount(current->pos, next_pos, next_time);
            auto next_node = std::make_shared<AStarNode>(
                next_pos, tentative_g, heuristic(next_pos, goal), current, next_time, conflicts);

            if(start == Vertex(74,40) && goal == Vertex(185,115)){
            logger::log_info("Generated successor node: (" + std::to_string(next_pos.x) + "," +
                           std::to_string(next_pos.y) + "), g: " +
                           std::to_string(tentative_g) +
                           ", h: " + std::to_string(heuristic(next_pos, goal)) +
                           ", f: " + std::to_string(next_node->f()) +
                           ", time: " + std::to_string(next_time) +
                           ", conflicts: " + std::to_string(conflicts));
            }
            
            open_list.push(next_node);
        }
    }

    // logger::log_info("Agent " + std::to_string(agent_id) + " found no path");
    return {};
}


bool violatesConstraints(const Vertex& current, const Vertex& next, 
                        int agent_id, int time,
                        const std::vector<Constraint>& constraints) {
    for (const auto& constraint : constraints) {
        if (constraint.agent != agent_id) continue;
        
        if (constraint.type == Constraint::VERTEX) {
            // 检查顶点约束
            if (constraint.time == (time + 1) && 
                constraint.vertex_constraint.vertex == next) {
                return true;
            }
        } else {  // EDGE constraint
            // 检查边约束
            if (constraint.time == time && 
                constraint.edge_constraint.v1 == current && 
                constraint.edge_constraint.v2 == next) {
                return true;
            }
        }
    }
    return false;
}