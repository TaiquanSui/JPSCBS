#include "CBS.h"
#include "../astar/AStar.h"
#include <queue>
#include <algorithm>
#include <chrono>
#include <sstream>

std::vector<std::vector<Vertex>> CBS::solve(const std::vector<Agent>& agents,
                                          const std::vector<std::vector<int>>& grid) {
    start_time = std::chrono::steady_clock::now();
    
    CBSNode root;
    
    // Find initial paths for each agent
    for (const auto& agent : agents) {
        auto path = find_path(agent, grid, {});
        if (path.empty()) {
            utils::log_info("cbs no solution");
            return {};
        } // No solution
        root.solution[agent.id] = path;
        root.cost += path.size();
    }

    auto compare = [](const CBSNode& a, const CBSNode& b) { return a.cost > b.cost; };
    std::priority_queue<CBSNode, std::vector<CBSNode>, decltype(compare)> open_list(compare);
    open_list.push(root);

    while (!open_list.empty()) {
        if (is_timeout()) {
            return {};  // timeout return empty result
        }
        
        CBSNode current = open_list.top();
        open_list.pop();

        auto conflicts = detect_conflicts(current);

        if (conflicts.empty()) {
            std::vector<std::vector<Vertex>> result;
            result.reserve(agents.size());
            for (const auto& agent : agents) {
                result.push_back(current.solution[agent.id]);
            }
            
            // 添加日志信息
            std::stringstream ss;
            ss << "Found solution with paths:" << std::endl;
            for (size_t i = 0; i < agents.size(); ++i) {
                ss << "Agent " << agents[i].id << ": ";
                const auto& path = result[i];
                ss << "(" << path[0].x << "," << path[0].y << ")";
                for (size_t j = 1; j < path.size(); ++j) {
                    ss << " -> (" << path[j].x << "," << path[j].y << ")";
                }
                ss << " [length: " << path.size() << "]" << std::endl;
            }
            ss << "Total cost: " << current.cost;
            utils::log_info(ss.str());
            
            return result;
        }

        const Conflict& conflict = conflicts.front();
        
        // Find agents involved in the conflict
        const Agent& agent1 = *std::find_if(agents.begin(), agents.end(),
            [&](const Agent& a) { return a.id == conflict.agent1; });
        const Agent& agent2 = *std::find_if(agents.begin(), agents.end(),
            [&](const Agent& a) { return a.id == conflict.agent2; });

        // Try to find bypass solutions for both agents
        if (use_bypass) {
            for (const Agent& agent : {agent1, agent2}){
                if (find_bypass(current, agent, conflict.vertex, conflict.time, grid)) {
                    open_list.push(current);
                    continue;
                }
            }
        }

        // Create child nodes and add constraints
        for (const Agent& agent : {agent1, agent2}) {
            CBSNode child = current;
            child.constraints.emplace_back(agent.id, conflict.vertex, conflict.time);
            
            auto new_path = find_path(agent, grid, child.constraints);
            
            if (!new_path.empty()) {
                child.solution[agent.id] = new_path;
                child.cost = 0;
                for (const auto& [_, path] : child.solution) {
                    child.cost += path.size() - 1;
                }
                open_list.push(child);
            }
        }
    }

    return {}; // No solution
}

std::vector<Conflict> CBS::detect_conflicts(const CBSNode& node) {
    std::vector<Conflict> conflicts;
    
    // 检查所有智能体对之间的冲突
    for (const auto& [agent1, path1] : node.solution) {
        for (const auto& [agent2, path2] : node.solution) {
            if (agent1 >= agent2) continue;
            
            auto path_conflicts = utils::detect_path_conflicts(agent1, agent2, path1, path2);
            conflicts.insert(conflicts.end(), path_conflicts.begin(), path_conflicts.end());
        }
    }
    
    return conflicts;
}

std::vector<Vertex> CBS::find_path(const Agent& agent,
                                 const std::vector<std::vector<int>>& grid,
                                 const std::vector<Constraint>& constraints) {
    return a_star(agent.id, agent.start, agent.goal, grid, constraints);
}

            
bool CBS::find_bypass(CBSNode& node, const Agent& agent, const Vertex& conflict_vertex, 
                     int conflict_time, const std::vector<std::vector<int>>& grid) {
    auto original_path = node.solution[agent.id];
    
    // Create temporary constraint set
    std::vector<Constraint> temp_constraints = node.constraints;
    // Add constraint for conflict position
    temp_constraints.emplace_back(agent.id, conflict_vertex, conflict_time);
    
    // Search using temporary constraint set
    auto path = a_star(agent.id, agent.start, agent.goal, grid, temp_constraints);

    // If a path of the same length is found, update the solution (but not the constraints)
    if (!path.empty() && path.size() == original_path.size()) {
        node.solution[agent.id] = path;
        return true;
    }

    return false;
}

bool CBS::is_timeout() const {
    auto current_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>
                   (current_time - start_time);
    return duration.count() / 1000.0 > time_limit;
}