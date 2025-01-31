#include "CBS.h"
#include "../astar/AStar.h"
#include "../utilities/Log.h"
#include "../utilities/Utility.h"
#include <queue>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <ranges>

std::vector<std::vector<Vertex>> CBS::solve(const std::vector<Agent>& agents,
                                          const std::vector<std::vector<int>>& grid) {
    expanded_nodes = 0;             
    start_time = std::chrono::steady_clock::now();
    
    logger::log_info("Starting CBS solver");
    
    CBSNode root;
    
    // Find initial paths for each agent
    for (const auto& agent : agents) {
        auto path = a_star(agent.start, agent.goal, grid);
        if (path.empty()) {
            logger::log_info("cbs no solution");
            return {};
        } // No solution
        root.solution[agent.id] = path;
        root.cost += utils::calculate_path_cost(path);
    }

    auto compare = [](const CBSNode& a, const CBSNode& b) { return a.cost > b.cost; };
    std::priority_queue<CBSNode, std::vector<CBSNode>, decltype(compare)> open_list(compare);
    open_list.push(root);

    // print_node_info(root, "Initial Node");

    while (!open_list.empty()) {
        if (should_terminate()) {
            logger::log_info("CBS solver interrupted");
            return {};
        }
        
        auto current = open_list.top();
        open_list.pop();
        expanded_nodes++;  // 增加计数器
        // print_node_info(current, "Current Node");

        auto constraints = generate_constraints(current);

        if (constraints.empty()) {
            std::vector<std::vector<Vertex>> result;
            result.reserve(agents.size());
            for (const auto& agent : agents) {
                result.push_back(current.solution[agent.id]);
            }
            
            // print_node_info(current, "Solution Found");
            
            return result;
        }

        // Try to find bypass solutions for all agents
        if (use_bypass) {
            if (find_bypass(current, agents, constraints, grid)) {
                open_list.push(current);
                continue;
            }
        }

        // Create child nodes and add constraints
        for (const auto& constraint : constraints) {
            CBSNode child = current;
            child.constraints.push_back(constraint);
            
            // logger::print_constraints(child.constraints, "Added Constraints");
            
            // Replan path for affected agent
            Agent affected_agent = *std::find_if(agents.begin(), agents.end(),
            [&](const Agent& a) { return a.id == constraint.agent; });
            
            auto new_path = find_path(affected_agent, grid, child.constraints);
            
            if (!new_path.empty()) {
                child.solution[constraint.agent] = new_path;
                child.cost = calculate_sic(child);
                open_list.push(child);
                // print_node_info(child, "New Child Node Generated");
            } else {
                logger::log_warning("Cannot find new path for agent " + std::to_string(constraint.agent) + 
                                 " that satisfies constraints");
            }
        }
    }

    logger::log_info("CBS found no solution");
    return {};
}

std::vector<Constraint> CBS::generate_constraints(const CBSNode& node) {
    std::vector<Constraint> constraints;
    
    int t = std::max_element(
        node.solution.begin(), 
        node.solution.end(),
        [](const auto& a, const auto& b) { 
            return a.second.size() < b.second.size(); 
        })->second.size();

    // Check conflicts between all agent pairs
    for (int i = 0; i < t; ++i) {
        for (const auto& [agent1, path1] : node.solution) {
            for (const auto& [agent2, path2] : node.solution) {
                if (agent1 >= agent2) continue;

                Vertex pos1 = i < path1.size() ? path1[i] : path1.back();
                Vertex pos2 = i < path2.size() ? path2[i] : path2.back();

                if (pos1 == pos2) {
                    constraints.emplace_back(agent1, pos1, i);  
                    constraints.emplace_back(agent2, pos2, i);
                    continue;
                }

                if (i < t - 1) {
                    Vertex next_pos1 = (i + 1) < path1.size() ? path1[i + 1] : path1.back();
                    Vertex next_pos2 = (i + 1) < path2.size() ? path2[i + 1] : path2.back();

                    // check swapping conflict
                    if (pos1 == next_pos2 && pos2 == next_pos1) {
                        constraints.emplace_back(agent1, pos1, i);
                        constraints.emplace_back(agent2, pos2, i);
                        continue;
                    }

                    // check following conflict
                    // if (next_pos1 == pos2) { // agent1 follows agent2
                    //     constraints.emplace_back(agent1, pos2, t+1);
                    //     constraints.emplace_back(agent2, pos2, t);
                    //     continue;
                    // }
                    // if (next_pos2 == pos1) { // agent2 follows agent1
                    //     constraints.emplace_back(agent1, pos1, t);
                    //     constraints.emplace_back(agent2, pos1, t+1);
                    //     continue;
                    // }
                }
            }
        }
    }

    return {};
}

std::vector<Vertex> CBS::find_path(const Agent& agent,
                                 const std::vector<std::vector<int>>& grid,
                                 const std::vector<Constraint>& constraints) {
    // logger::print_constraints(constraints, 
    //     "Finding path for agent " + std::to_string(agent.id) + ", current constraints:");
    return a_star(agent.id, agent.start, agent.goal, grid, constraints);
}

            
bool CBS::find_bypass(CBSNode& node, const std::vector<Agent>& agents, 
                     const std::vector<Constraint>& constraints,
                     const std::vector<std::vector<int>>& grid) {
    // Record original conflict count
    int original_conflicts = count_conflicts(node);
    // logger::log_info("Original conflict count: " + std::to_string(original_conflicts));
    
    // Try bypass for each constraint
    for (const auto& constraint : constraints) {
        // Find affected agent
        Agent affected_agent = *std::find_if(agents.begin(), agents.end(),
            [&](const Agent& a) { return a.id == constraint.agent; });
            
        auto original_path = node.solution[affected_agent.id];
        
        // Create temporary constraint set
        std::vector<Constraint> temp_constraints = node.constraints;
        temp_constraints.emplace_back(affected_agent.id, constraint.vertex, constraint.time);
        
        auto path = a_star(affected_agent.id, affected_agent.start, affected_agent.goal, 
                          grid, temp_constraints);
        // logger::print_constraints(temp_constraints, "Bypass Path Constraints");
        
        // // Original path
        // logger::log_info("Original path: " + logger::vectorToString(original_path) + 
        //                 " Original path cost: " + std::to_string(utils::calculate_path_cost(original_path)));
        // // Bypass path
        // logger::log_info("Bypass path: " + logger::vectorToString(path) + 
        //                 " Bypass path cost: " + std::to_string(utils::calculate_path_cost(path)));

        if (!path.empty() && 
            std::abs(utils::calculate_path_cost(path) - utils::calculate_path_cost(original_path)) < 1e-6) {
            CBSNode temp_node = node;
            temp_node.solution[affected_agent.id] = path;
            
            int new_conflicts = count_conflicts(temp_node);
            // logger::log_info("New conflict count: " + std::to_string(new_conflicts));
            

            if (new_conflicts < original_conflicts) {
                node.solution[affected_agent.id] = path;
                node.cost = calculate_sic(node);
                //print_node_info(node, "Bypass Node (Conflicts Reduced)");
                return true;
            } else {
                //logger::log_info("Bypass path did not reduce conflicts, discarding path");
            }
        }
    }
    return false;
}

bool CBS::is_timeout() const {
    auto current_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>
                   (current_time - start_time);
    return duration.count() / 1000.0 > time_limit;
}

void CBS::print_node_info(const CBSNode& node, const std::string& prefix) {
    if (!prefix.empty()) {
        logger::log_info(prefix + ":");
    }
    logger::log_info("Total cost: " + std::to_string(std::round(node.cost * 1000) / 1000.0));
    logger::log_info("Constraint count: " + std::to_string(node.constraints.size()));
    
    for (const auto& [agent_id, path] : node.solution) {
        std::string path_str = "Path for agent " + std::to_string(agent_id) + ": ";
        for (const auto& pos : path) {
            path_str += "(" + std::to_string(pos.x) + "," + std::to_string(pos.y) + ") ";
        }
        logger::log_info(path_str);
    }
    logger::log_info("------------------------");
}

double CBS::calculate_sic(const CBSNode& node) {
    double cost = 0.0;
    for (const auto& [agent_id, path] : node.solution) {
        cost += utils::calculate_path_cost(path);
    }
    return cost;
}

int CBS::count_conflicts(const CBSNode& node) {
    int total_conflicts = 0;
    const auto& solution = node.solution;
    
    // 检查每对智能体之间的冲突
    for (auto it1 = solution.begin(); it1 != solution.end(); ++it1) {
        auto it2 = it1;
        ++it2;
        for (; it2 != solution.end(); ++it2) {
            total_conflicts += utils::count_conflicts(
                it1->first,  // agent1_id
                it2->first,  // agent2_id
                it1->second, // path1
                it2->second  // path2
            );
        }
    }
    
    return total_conflicts;
}

