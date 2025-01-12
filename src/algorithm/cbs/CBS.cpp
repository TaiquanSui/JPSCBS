#include "CBS.h"
#include "../astar/AStar.h"
#include <queue>
#include <algorithm>

std::vector<std::vector<Vertex>> CBS::solve(const std::vector<Agent>& agents,
                                          const std::vector<std::vector<int>>& grid) {
    CBSNode root;
    
    // 为每个智能体找到初始路径
    for (const auto& agent : agents) {
        auto path = find_path(agent, grid, {});
        if (path.empty()) return {}; // 无解
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
        
        // 找到冲突涉及的智能体
        const Agent& agent1 = *std::find_if(agents.begin(), agents.end(),
            [&](const Agent& a) { return a.id == conflict.agent1; });
        const Agent& agent2 = *std::find_if(agents.begin(), agents.end(),
            [&](const Agent& a) { return a.id == conflict.agent2; });

        // 尝试为两个智能体寻找绕路方案
        if (use_bypass) {
            for (const Agent& agent : {agent1, agent2}){
                if (find_bypass(current, agent, conflict.vertex, conflict.time, grid)) {
                    open_list.push(current);
                    continue;
                }
            }
        }

        // 创建子节点并添加约束
        for (const Agent& agent : {agent1, agent2}) {
            CBSNode child = current;
            child.constraints.emplace_back(agent.id, conflict.vertex, conflict.time);
            
            auto new_path = find_path(agent, grid, child.constraints);
            
            if (!new_path.empty()) {
                child.solution[agent.id] = new_path;
                child.cost = 0;
                for (const auto& [_, path] : child.solution) {
                    child.cost += path.size();
                }
                open_list.push(child);
            }
        }
    }

    return {}; // 无解
}

std::vector<Conflict> CBS::detect_conflicts(const CBSNode& node) {
    std::vector<Conflict> conflicts;
    
    // 检查所有智能体对之间的冲突
    for (const auto& [agent1, path1] : node.solution) {
        for (const auto& [agent2, path2] : node.solution) {
            if (agent1 >= agent2) continue;
            
            size_t max_length = std::max(path1.size(), path2.size());
            
            // 检查每个时间步
            for (size_t t = 0; t < max_length; ++t) {
                // 获取当前位置（如果路径结束则使用终点）
                Vertex pos1 = t < path1.size() ? path1[t] : path1.back();
                Vertex pos2 = t < path2.size() ? path2[t] : path2.back();

                // 检查顶点冲突 (Vertex conflict)
                if (pos1 == pos2) {
                    conflicts.emplace_back(agent1, agent2, pos1, t);
                    continue;
                }

                // 检查跟随冲突和交换冲突
                if (t < max_length - 1) {
                    Vertex next_pos1 = (t + 1) < path1.size() ? path1[t + 1] : path1.back();
                    Vertex next_pos2 = (t + 1) < path2.size() ? path2[t + 1] : path2.back();
                    
                    // 检查交换冲突 (Swapping conflict) 或 跟随冲突 (Following conflict)
                    if (pos1 == next_pos2 && pos2 == next_pos1) { // 交换冲突
                        // 对于交换冲突，可以选择任一交换位置作为冲突位置
                        conflicts.emplace_back(agent1, agent2, pos1, t);
                        conflicts.emplace_back(agent1, agent2, pos2, t);
                        continue;
                    }
                    if (next_pos1 == pos2) { // agent1 跟随 agent2
                        conflicts.emplace_back(agent1, agent2, pos2, t);
                        continue;
                    }
                    if (next_pos2 == pos1) { // agent2 跟随 agent1
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
    return a_star(agent, grid, constraints);
}

            
bool CBS::find_bypass(CBSNode& node, const Agent& agent, const Vertex& conflict_vertex, 
                     int conflict_time, const std::vector<std::vector<int>>& grid) {
    auto original_path = node.solution[agent.id];
    
    // 创建临时约束集
    std::vector<Constraint> temp_constraints = node.constraints;
    // 添加冲突位置的约束
    temp_constraints.emplace_back(agent.id, conflict_vertex, conflict_time);
    
    // 使用临时约束集进行搜索
    auto path = a_star(agent, grid, temp_constraints);

    // 如果找到相同长度的路径，则更新解决方案（但不更新约束）
    if (!path.empty() && path.size() == original_path.size()) {
        node.solution[agent.id] = path;
        return true;
    }

    return false;
}
