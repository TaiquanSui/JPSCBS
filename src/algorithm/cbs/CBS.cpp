#include "CBS.h"
#include "../astar/AStar.h"
#include "../utilities/Log.h"
#include "../utilities/Utility.h"
#include <queue>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <iomanip>

std::vector<std::vector<Vertex>> CBS::solve(const std::vector<Agent>& agents,
                                          const std::vector<std::vector<int>>& grid) {
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

    print_node_info(root, "初始节点");

    while (!open_list.empty()) {
        if (is_timeout()) {
            logger::log_warning("Time limit exceeded");
            return {};
        }
        
        auto current = open_list.top();
        open_list.pop();
        print_node_info(current, "当前节点");

        auto constraints = generate_constraints(current);

        if (constraints.empty()) {
            std::vector<std::vector<Vertex>> result;
            result.reserve(agents.size());
            for (const auto& agent : agents) {
                result.push_back(current.solution[agent.id]);
            }
            
            print_node_info(current, "找到解");
            
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
            
            logger::print_constraints(child.constraints, "添加约束");
            
            // 为受约束影响的智能体重新规划路径
            Agent affected_agent = *std::find_if(agents.begin(), agents.end(),
            [&](const Agent& a) { return a.id == constraint.agent; });
            
            auto new_path = find_path(affected_agent, grid, child.constraints);
            
            if (!new_path.empty()) {
                child.solution[constraint.agent] = new_path;
                child.cost = calculate_sic(child);
                open_list.push(child);
                print_node_info(child, "新生成的子节点");
            } else {
                logger::log_warning("无法为智能体 " + std::to_string(constraint.agent) + 
                                 " 找到满足约束的新路径");
            }
        }
    }

    logger::log_info("CBS found no solution");
    return {};
}

std::vector<Constraint> CBS::generate_constraints(const CBSNode& node) {
    std::vector<Constraint> constraints;
    
    // 检查所有智能体对之间的冲突
    for (const auto& [agent1, path1] : node.solution) {
        for (const auto& [agent2, path2] : node.solution) {
            if (agent1 >= agent2) continue;
            
            std::vector<Constraint> constraints = utils::generate_constraints_from_conflict(
                agent1, agent2, path1, path2);
            if(!constraints.empty()) {
                logger::print_constraints(constraints, "发现新的约束");
                return constraints;
            }
        }
    }

    return {};
}

std::vector<Vertex> CBS::find_path(const Agent& agent,
                                 const std::vector<std::vector<int>>& grid,
                                 const std::vector<Constraint>& constraints) {
    logger::print_constraints(constraints, 
        "为智能体 " + std::to_string(agent.id) + " 寻找路径，当前约束:");
    return a_star(agent.id, agent.start, agent.goal, grid, constraints);
}

            
bool CBS::find_bypass(CBSNode& node, const std::vector<Agent>& agents, 
                     const std::vector<Constraint>& constraints,
                     const std::vector<std::vector<int>>& grid) {
    // 记录原始节点的冲突数量
    int original_conflicts = count_conflicts(node);
    logger::log_info("原始节点的冲突数量: " + std::to_string(original_conflicts));
    
    // 对每个约束尝试绕行
    for (const auto& constraint : constraints) {
        // 找到受影响的智能体
        Agent affected_agent = *std::find_if(agents.begin(), agents.end(),
            [&](const Agent& a) { return a.id == constraint.agent; });
            
        auto original_path = node.solution[affected_agent.id];
        
        // Create temporary constraint set
        std::vector<Constraint> temp_constraints = node.constraints;
        // Add constraint for conflict position
        temp_constraints.emplace_back(affected_agent.id, constraint.vertex, constraint.time);
        
        // Search using temporary constraint set
        auto path = a_star(affected_agent.id, affected_agent.start, affected_agent.goal, 
                          grid, temp_constraints);
        logger::print_constraints(temp_constraints, "绕行路径约束");
        
        // 原有路径
        logger::log_info("原有路径: " + logger::vectorToString(original_path) + 
                        " 原有路径代价: " + std::to_string(utils::calculate_path_cost(original_path)));
        // 绕行路径
        logger::log_info("绕行路径: " + logger::vectorToString(path) + 
                        " 绕行路径代价: " + std::to_string(utils::calculate_path_cost(path)));

        // 只比较路径代价，不要求路径长度相同
        if (!path.empty() && 
            std::abs(utils::calculate_path_cost(path) - utils::calculate_path_cost(original_path)) < 1e-6) {
            // 创建临时节点来测试新路径
            CBSNode temp_node = node;
            temp_node.solution[affected_agent.id] = path;
            
            // 计算新路径的冲突数量
            int new_conflicts = count_conflicts(temp_node);
            logger::log_info("新路径的冲突数量: " + std::to_string(new_conflicts));
            
            // 只有当冲突数量减少时才接受新路径
            if (new_conflicts < original_conflicts) {
                node.solution[affected_agent.id] = path;
                node.cost = calculate_sic(node);
                print_node_info(node, "绕行节点(冲突减少)");
                return true;
            } else {
                logger::log_info("绕行路径未能减少冲突数量，放弃此路径");
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
    logger::log_info("总代价: " + std::to_string(std::round(node.cost * 1000) / 1000.0));  // 保留3位小数
    logger::log_info("约束数量: " + std::to_string(node.constraints.size()));
    
    for (const auto& [agent_id, path] : node.solution) {
        std::string path_str = "智能体 " + std::to_string(agent_id) + " 的路径: ";
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

