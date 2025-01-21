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
        root.cost += calculate_sic(path);
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
                ss << " [length: " << std::to_string(std::round(calculate_sic(path) * 1000) / 1000.0) << "]" << std::endl;
            }
            ss << "Total cost: " << std::to_string(std::round(current.cost * 1000) / 1000.0);
            logger::log_info(ss.str());
            
            return result;
        }

        logger::log_info("产生constrain: 智能体 " + std::to_string(constraints.front().agent) +
                       " 在位置 (" + std::to_string(constraints.front().vertex.x) + 
                       "," + std::to_string(constraints.front().vertex.y) + 
                       ") 时间 " + std::to_string(constraints.front().time));

        
        // Try to find bypass solutions for both agents
        if (use_bypass) {
            for (const auto& constraint : constraints) {
                Agent affected_agent = *std::find_if(agents.begin(), agents.end(),
                    [&](const Agent& a) { return a.id == constraint.agent; });
                // 对冲突中涉及的每个智能体尝试绕行
                if (find_bypass(current, affected_agent, constraint.vertex, constraint.time, grid)) {
                    open_list.push(current);
                    continue;
                }
            }
        }

        // Create child nodes and add constraints
        for (const auto& constraint : constraints) {
            CBSNode child = current;
            child.constraints.push_back(constraint);
            
            logger::log_info("为智能体 " + std::to_string(constraint.agent) + 
                           " 添加约束: 不能在时间 " + std::to_string(constraint.time) + 
                           " 到达位置 (" + std::to_string(constraint.vertex.x) + 
                           "," + std::to_string(constraint.vertex.y) + ")");
            
            // 为受约束影响的智能体重新规划路径
            Agent affected_agent = *std::find_if(agents.begin(), agents.end(),
            [&](const Agent& a) { return a.id == constraint.agent; });
            
            
            auto new_path = find_path(affected_agent, grid, child.constraints);
            
            if (!new_path.empty()) {
                child.solution[constraint.agent] = new_path;
                child.cost = calculate_sic(new_path);
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
                return constraints;
            }
        }
    }

    return {};
}

std::vector<Vertex> CBS::find_path(const Agent& agent,
                                 const std::vector<std::vector<int>>& grid,
                                 const std::vector<Constraint>& constraints) {
    std::stringstream ss;
    ss << "为智能体 " << agent.id << " 寻找路径，当前约束: [";
    
    for (size_t i = 0; i < constraints.size(); ++i) {
        const auto& constraint = constraints[i];
        ss << "智能体" << constraint.agent 
           << "在时间" << constraint.time 
           << "不能到达位置(" << constraint.vertex.x 
           << "," << constraint.vertex.y << ")";
           
        if (i < constraints.size() - 1) {
            ss << ", ";
        }
    }
    ss << "]";
    
    logger::log_info(ss.str());


    return a_star(agent.id, agent.start, agent.goal, grid, constraints);
}

            
bool CBS::find_bypass(CBSNode& node, const Agent& agent, const Vertex& constrain_vertex, 
                     int constrain_time, const std::vector<std::vector<int>>& grid) {
    auto original_path = node.solution[agent.id];
    
    // Create temporary constraint set
    std::vector<Constraint> temp_constraints = node.constraints;
    // Add constraint for conflict position
    temp_constraints.emplace_back(agent.id, constrain_vertex, constrain_time);
    
    // Search using temporary constraint set
    auto path = a_star(agent.id, agent.start, agent.goal, grid, temp_constraints);

    // 只比较路径代价，不要求路径长度相同
    if (!path.empty() && 
        std::abs(calculate_sic(path) - calculate_sic(original_path)) < 1e-6) {
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

double CBS::calculate_sic(const std::vector<Vertex>& path) {
    double cost = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        if (utils::isDiagonal(path[i+1] - path[i])) {
            cost += std::sqrt(2.0);
        } else {
            cost += 1.0;
        }
    }
    return cost;
}