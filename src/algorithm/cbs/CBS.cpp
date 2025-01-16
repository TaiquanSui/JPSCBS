#include "CBS.h"
#include "../astar/AStar.h"
#include <queue>
#include <algorithm>

std::vector<std::vector<Vertex>> CBS::solve(const std::vector<Agent>& agents,
                                          const std::vector<std::vector<int>>& grid) {
    CBSNode root;
    
    // Find initial paths for each agent
    for (const auto& agent : agents) {
        auto path = find_path(agent, grid, {});
        if (path.empty()) return {}; // No solution
        root.solution[agent.id] = path;
        root.cost += path.size();
    }

    auto compare = [](const CBSNode& a, const CBSNode& b) { return a.cost > b.cost; };
    std::priority_queue<CBSNode, std::vector<CBSNode>, decltype(compare)> open_list(compare);
    open_list.push(root);

    while (!open_list.empty()) {
        CBSNode current = open_list.top();
        open_list.pop();

        auto conflicts = detect_conflicts(current);

        if (conflicts.empty()) {
            std::vector<std::vector<Vertex>> result;
            result.reserve(agents.size());
            for (const auto& agent : agents) {
                result.push_back(current.solution[agent.id]);
            }
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
    
    // Check conflicts between all agent pairs
    for (const auto& [agent1, path1] : node.solution) {
        for (const auto& [agent2, path2] : node.solution) {
            if (agent1 >= agent2) continue;
            
            size_t max_length = std::max(path1.size(), path2.size());
            
            // Check each time step
            for (size_t t = 0; t < max_length; ++t) {
                // Get current position (if path ends, use goal)
                Vertex pos1 = t < path1.size() ? path1[t] : path1.back();
                Vertex pos2 = t < path2.size() ? path2[t] : path2.back();

                // Check vertex conflict (Vertex conflict)
                if (pos1 == pos2) {
                    conflicts.emplace_back(agent1, agent2, pos1, t);
                    continue;
                }

                // Check following and swapping conflicts
                if (t < max_length - 1) {
                    Vertex next_pos1 = (t + 1) < path1.size() ? path1[t + 1] : path1.back();
                    Vertex next_pos2 = (t + 1) < path2.size() ? path2[t + 1] : path2.back();
                    
                    // Check swapping conflict (Swapping conflict) or following conflict (Following conflict)
                    if (pos1 == next_pos2 && pos2 == next_pos1) { // Swapping conflict
                        // For swapping conflict, choose either swapping position as conflict position
                        conflicts.emplace_back(agent1, agent2, pos1, t);
                        conflicts.emplace_back(agent1, agent2, pos2, t);
                        continue;
                    }
                    if (next_pos1 == pos2) { // agent1 follows agent2
                        conflicts.emplace_back(agent1, agent2, pos2, t);
                        continue;
                    }
                    if (next_pos2 == pos1) { // agent2 follows agent1
                        conflicts.emplace_back(agent1, agent2, pos1, t);
                        continue;
                    }
                }
            }
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
