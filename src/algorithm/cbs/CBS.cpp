#include "CBS.h"
#include "../astar/AStar.h"
#include <queue>
#include <iostream>

std::vector<std::vector<Vertex>> CBS::solve(const std::vector<std::pair<Vertex, Vertex>>& agents, const std::vector<std::vector<int>>& grid) {
    CBSNode root;

    for (size_t i = 0; i < agents.size(); ++i) {
        auto path = a_star(agents[i].first, agents[i].second, grid);
        if (path.empty()) return {}; // No solution
        root.solution[i] = path;
        root.cost += path.size();
    }

    auto compare = [](const CBSNode& a, const CBSNode& b) { return a.cost > b.cost; };
    std::priority_queue<CBSNode, std::vector<CBSNode>, decltype(compare)> open_list(compare);
    open_list.push(root);

    while (!open_list.empty()) {
        CBSNode current = open_list.top();
        open_list.pop();

        // Detect conflicts
        current.conflicts.clear();
        for (const auto& [agent1, path1] : current.solution) {
            for (const auto& [agent2, path2] : current.solution) {
                if (agent1 >= agent2) continue;
                for (size_t t = 0; t < std::max(path1.size(), path2.size()); ++t) {
                    Vertex pos1 = t < path1.size() ? path1[t] : path1.back();
                    Vertex pos2 = t < path2.size() ? path2[t] : path2.back();

                    if (pos1 == pos2) {
                        current.conflicts.emplace_back(agent1, agent2, pos1, t);
                    }
                }
            }
        }

        if (current.conflicts.empty()) {
            std::vector<std::vector<Vertex>> result;
            for (const auto& [_, path] : current.solution) {
                result.push_back(path);
            }
            return result;
        }

        const auto& conflict = current.conflicts.front();
        int agent1 = std::get<0>(conflict);
        Vertex conflict_vertex = std::get<2>(conflict);
        int conflict_time = std::get<3>(conflict);

        if (use_bypass && find_bypass(current, agent1, conflict_vertex, conflict_time, grid)) {
            open_list.push(current);
            continue;
        }

        std::vector<CBSNode> children;
        resolve_conflict(current, conflict, children, grid);
        for (const auto& child : children) {
            open_list.push(child);
        }
    }

    return {}; // No solution found
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