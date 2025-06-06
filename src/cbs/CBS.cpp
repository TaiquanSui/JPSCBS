#include "CBS.h"
#include "../astar/AStar.h"
#include "../utilities/Log.h"
#include "../utilities/ConstraintUtility.h"
#include "../utilities/GridUtility.h"
#include "ConflictAvoidanceTable.h"
#include <queue>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <vector>
#include <ranges>


std::vector<std::vector<Vertex>> CBS::solve(const std::vector<Agent>& agents,
                                          const std::vector<std::vector<int>>& grid) {
    expanded_nodes = 0;             
    start_time = std::chrono::steady_clock::now();
    interrupted = false;  // 重置中断状态
    
    // //logger::log_info("Starting CBS solver");
    
    CBSNode root;
    
    // Find initial paths for each agent
    for (const auto& agent : agents) {
        // //logger::log_info("start: (" + std::to_string(agent.start.x) + "," + std::to_string(agent.start.y) + ")");
        // //logger::log_info("goal: (" + std::to_string(agent.goal.x) + "," + std::to_string(agent.goal.y) + ")");
        auto path = a_star(agent.start, agent.goal, grid);
        if (path.empty()) {
            // //logger::log_info("cbs no solution");
            return {};
        } // No solution
        root.solution[agent.id] = path;
        root.cost += utils::calculate_path_cost(path);
    }

    auto compare = [this](const CBSNode& a, const CBSNode& b) { 
        if (std::abs(a.cost - b.cost) < 1e-6) {
            int a_conflicts = count_conflicts(a);
            int b_conflicts = count_conflicts(b);
            if (a_conflicts != b_conflicts) {
                return a_conflicts > b_conflicts;
            }
            // FIFO, 使用内存地址作为稳定的排序依据
            return &a > &b;
        }
        return a.cost > b.cost;
    };
    std::priority_queue<CBSNode, std::vector<CBSNode>, decltype(compare)> open_list(compare);
    open_list.push(root);

    // print_node_info(root, "Initial Node");

    while (!open_list.empty()) {
        
        auto current = open_list.top();
        open_list.pop();
        expanded_nodes++;  // 增加计数器
        print_node_info(current, "Current Node");

        auto constraints = generate_constraints(current);
        //logger::log_info("Generating constraints");
        //logger::print_constraints(constraints, "Constraints");

        if (constraints.empty()) {
            std::vector<std::vector<Vertex>> result;
            result.reserve(agents.size());
            for (const auto& agent : agents) {
                result.push_back(current.solution[agent.id]);
                //logger::log_info("path of agent" + std::to_string(agent.id) + " :" + //logger::vectorToString(current.solution[agent.id]));
            }
            //logger::log_info("expanded nodes:" + std::to_string(expanded_nodes));

            return result;
        }

        if (should_terminate()) {
            // //logger::log_info("CBS solver interrupted");
            return {};
        }

        std::erase_if(constraints, [](const auto& constraint) {
            return constraint.time == 0;
        });

        if (use_bypass) {
            CBSBypassResult bypass_result = find_bypass(current, constraints, agents, grid);
            if (bypass_result.success) {
                open_list.push(current);
                continue;
            }

            for(const auto& result : bypass_result.bypass_paths){
                CBSNode child = current;
                child.constraints.push_back(result.first);
                child.solution[result.first.agent] = result.second;
                child.cost = calculate_sic(child);
                open_list.push(child);
            }
        }else{
            for (const auto& constraint : constraints) {
                if (should_terminate()) {
                    // ////logger::log_info("CBS solver interrupted");
                    return {};
                }

                CBSNode child = current;
                child.constraints.push_back(constraint);

                Agent affected_agent = *std::find_if(agents.begin(), agents.end(),
                [&](const Agent& a) { return a.id == constraint.agent; });

                ConflictAvoidanceTable cat = calculate_cat({affected_agent.id}, child);
                auto new_path = a_star(affected_agent.id, affected_agent.start, affected_agent.goal, 
                              grid, child.constraints, 0, cat);

                if (!new_path.empty()) {
                    ////logger::log_info("New path found for agent " + std::to_string(utils::calculate_path_cost(new_path)));
                    ////logger::log_info("path: " + ////logger::vectorToString(new_path));
                    child.solution[constraint.agent] = new_path;
                    child.cost = calculate_sic(child);
                    open_list.push(child);
                    // print_node_info(child, "New Child Node Generated");
                    ////logger::log_info("New cost: " + std::to_string(child.cost));
                } else {
                    // ////logger::log_warning("Cannot find new path for agent " + std::to_string(constraint.agent) + 
                    //                  " that satisfies constraints");
                }
            }
        }

        
    }

    // //logger::log_info("CBS found no solution");
    return {};
}

std::vector<Constraint> CBS::generate_constraints(const CBSNode& node) {
    return utils::generate_constraints(node);
}

ConflictAvoidanceTable CBS::calculate_cat(const std::unordered_set<int>& excluded_agents, 
                                        const CBSNode& node) const {
    ConflictAvoidanceTable cat;
    
    // 计算其他智能体的路径占用情况
    for (const auto& [agent_id, path] : node.solution) {
        if (excluded_agents.count(agent_id) > 0) continue;  // 跳过被排除的智能体
        
        // 对路径中的每个位置添加约束
        for (size_t t = 0; t < path.size(); ++t) {
            // 添加顶点约束
            cat.addVertexConstraint(path[t], t);
            
            // 添加边约束（对于相邻时间步的位置）
            if (t < path.size() - 1) {
                // 添加双向的边约束
                cat.addEdgeConstraint(path[t], path[t + 1], t);     // 从t到t+1的移动
                cat.addEdgeConstraint(path[t + 1], path[t], t);     // 从t+1到t的移动（防止交换冲突）
            }
            
            // 如果是路径的最后一个位置，添加终点约束
            // if (t == path.size() - 1) {
            //     cat.addGoalConstraint(path[t], t);
            // }
        }
    }
    
    return cat;
}

CBSBypassResult CBS::find_bypass(CBSNode& node, const std::vector<Constraint>& constraints, 
                            const std::vector<Agent>& agents, 
                            const std::vector<std::vector<int>>& grid) {
    int original_conflicts = count_conflicts(node);
    //logger::log_info("Original conflict count: " + std::to_string(original_conflicts));
    
    std::vector<std::pair<Constraint, std::vector<Vertex>>> bypass_paths;

    for (const auto& constraint : constraints) {
        if (should_terminate()) {
            // ////logger::log_info("CBS solver interrupted");
            return {};
        }

        Agent affected_agent = *std::find_if(agents.begin(), agents.end(),
            [&](const Agent& a) { return a.id == constraint.agent; });
            
        auto original_path = node.solution[affected_agent.id];
        
        //logger::print_constraint(constraint, "find bypass for constraint: ");
        std::vector<Constraint> temp_constraints = node.constraints;
        temp_constraints.push_back(constraint);
        
        ConflictAvoidanceTable cat = calculate_cat({affected_agent.id}, node);
        auto new_path = a_star(affected_agent.id, affected_agent.start, affected_agent.goal, 
                          grid, temp_constraints, 0, cat);
        //logger::log_info("new bypass path: " + //logger::vectorToString(new_path));

        if (!new_path.empty()) {
            if (std::abs(utils::calculate_path_cost(new_path) - 
                utils::calculate_path_cost(original_path)) < 1e-6) {
                
                CBSNode temp_node = node;
                temp_node.solution[affected_agent.id] = new_path;
                
                int new_conflicts = count_conflicts(temp_node);
                //logger::log_info("New conflict count: " + std::to_string(new_conflicts));
                
                if (new_conflicts < original_conflicts) {
                    node.solution[affected_agent.id] = new_path;
                    return {true, {}};
                }else{
                    bypass_paths.emplace_back(constraint, new_path);
                }
            }else{
                bypass_paths.emplace_back(constraint, new_path);
            }
        }
    }
    
    return {false, bypass_paths};
}

bool CBS::is_timeout() const {
    auto current_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>
                   (current_time - start_time);
    return duration.count() / 1000.0 > time_limit;
}

void CBS::print_node_info(const CBSNode& node, const std::string& prefix) {
    if (!prefix.empty()) {
        //logger::log_info(prefix + ":");
    }
    //logger::log_info("Total cost: " + std::to_string(std::round(node.cost * 1000) / 1000.0));
    //logger::print_constraints(node.constraints, "Constraints");
    
    for (const auto& [agent_id, path] : node.solution) {
        std::string path_str = "Path for agent " + std::to_string(agent_id) + ": ";
        for (const auto& pos : path) {
            path_str += "(" + std::to_string(pos.x) + "," + std::to_string(pos.y) + ") ";
        }
        //logger::log_info(path_str);
    }
    ////logger::log_info("------------------------");
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
    for (const auto& [agent_id, path1] : node.solution) {
        for (const auto& [other_id, path2] : node.solution) {
            if (other_id <= agent_id) continue;  // 避免重复计算
            total_conflicts += utils::count_conflicts(path1, path2);
        }
    }
    return total_conflicts;
}

