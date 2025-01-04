#include "CBS.h"
#include "../astar/AStar.h"
#include <queue>
#include <algorithm>

std::vector<std::vector<Vertex>> CBS::solve(const std::vector<std::pair<Vertex, Vertex>>& agents,
                                          const std::vector<std::vector<int>>& grid) {
    CBSNode root;
    
    // 为每个智能体找到初始路径
    for (size_t i = 0; i < agents.size(); ++i) {
        auto path = find_path(agents[i].first, agents[i].second, grid, {}, i);
        if (path.empty()) return {}; // 无解
        root.solution[i] = path;
        root.cost += path.size();
    }

    auto compare = [](const CBSNode& a, const CBSNode& b) { return a.cost > b.cost; };
    std::priority_queue<CBSNode, std::vector<CBSNode>, decltype(compare)> open_list(compare);
    open_list.push(root);

    while (!open_list.empty()) {
        CBSNode current = open_list.top();
        open_list.pop();

        // 检测冲突
        detect_conflicts(current);

        if (current.conflicts.empty()) {
            std::vector<std::vector<Vertex>> result;
            result.reserve(agents.size());
            for (size_t i = 0; i < agents.size(); ++i) {
                result.push_back(current.solution[i]);
            }
            return result;
        }

        // 处理第一个冲突
        const auto& conflict = current.conflicts.front();
        int agent1 = std::get<0>(conflict);
        int agent2 = std::get<1>(conflict);
        Vertex conflict_vertex = std::get<2>(conflict);
        int conflict_time = std::get<3>(conflict);

        // 尝试绕路
        if (use_bypass && find_bypass(current, agent1, conflict_vertex, conflict_time, grid)) {
            open_list.push(current);
            continue;
        }

        // 创建子节点并添加约束
        for (int agent : {agent1, agent2}) {
            CBSNode child = current;
            child.constraints.emplace_back(agent, conflict_vertex, conflict_time);
            
            // 重新规划受影响智能体的路径
            auto new_path = find_path(agents[agent].first, agents[agent].second, 
                                    grid, child.constraints, agent);
            
            if (!new_path.empty()) {
                child.solution[agent] = new_path;
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

void CBS::detect_conflicts(CBSNode& node) {
    node.conflicts.clear();
    for (const auto& [agent1, path1] : node.solution) {
        for (const auto& [agent2, path2] : node.solution) {
            if (agent1 >= agent2) continue;
            
            size_t max_length = std::max(path1.size(), path2.size());
            for (size_t t = 0; t < max_length; ++t) {
                Vertex pos1 = t < path1.size() ? path1[t] : path1.back();
                Vertex pos2 = t < path2.size() ? path2[t] : path2.back();

                if (pos1 == pos2) {
                    node.conflicts.emplace_back(agent1, agent2, pos1, t);
                }

                if (t < max_length - 1) {
                    Vertex next_pos1 = t + 1 < path1.size() ? path1[t + 1] : path1.back();
                    Vertex next_pos2 = t + 1 < path2.size() ? path2[t + 1] : path2.back();
                    
                    if (pos1 == next_pos2 && pos2 == next_pos1) {
                        node.conflicts.emplace_back(agent1, agent2, pos1, t);
                    }
                }
            }
        }
    }
}

std::vector<Vertex> CBS::find_path(const Vertex& start, const Vertex& goal,
                                 const std::vector<std::vector<int>>& grid,
                                 const std::vector<Constraint>& constraints,
                                 int agent_id) {
    auto is_valid = [&](const Vertex& pos, int time) {
        for (const auto& constraint : constraints) {
            if (constraint.agent == agent_id && 
                constraint.vertex == pos && 
                constraint.time == time) {
                return false;
            }
        }
        return true;
    };

    return a_star(start, goal, grid, is_valid);
}

bool CBS::validate_path(const std::vector<Vertex>& path,
                      const std::vector<Constraint>& constraints,
                      int agent_id) {
    for (size_t t = 0; t < path.size(); ++t) {
        for (const auto& constraint : constraints) {
            if (constraint.agent == agent_id &&
                constraint.vertex == path[t] &&
                constraint.time == t) {
                return false;
            }
        }
    }
    return true;
}

bool CBS::find_bypass(CBSNode& node, int agent, const Vertex& conflict_vertex, int conflict_time, const std::vector<std::vector<int>>& grid) {
    auto original_path = node.solution[agent];
    auto path = a_star(original_path[0], original_path.back(), grid);

    if (!path.empty() && path.size() == original_path.size()) {
        node.solution[agent] = path;
        return true;
    }

    return false;
}

void CBS::resolve_conflict(CBSNode& node, const std::tuple<int, int, Vertex, int>& conflict, std::vector<CBSNode>& children, const std::vector<std::vector<int>>& grid) {
    int agent1 = std::get<0>(conflict);
    int agent2 = std::get<1>(conflict);
    Vertex conflict_vertex = std::get<2>(conflict);
    int conflict_time = std::get<3>(conflict);

    for (int agent : {agent1, agent2}) {
        CBSNode child = node;
        auto& path = child.solution[agent];
        if (conflict_time < path.size()) {
            path[conflict_time] = conflict_vertex;
        }
        child.cost += 1;
        children.push_back(child);
    }
}