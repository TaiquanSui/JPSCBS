#ifndef JPSCBS_H
#define JPSCBS_H

#include "../Vertex.h"
#include "../jps/JPS.h"
#include "../Agent.h"
#include "../cbs/CBS.h"
#include <unordered_map>
#include <queue>
#include <vector>
#include <tuple>

struct JPSCBSNode {
    // 每个智能体的多条路径（按照代价排序）
    std::unordered_map<int, std::priority_queue<JPSPath, std::vector<JPSPath>, JPSPathComparator>> solution;
    std::vector<Constraint> constraints;
    int cost;
    std::vector<std::tuple<int, int, Vertex, int>> conflicts; // (agent1, agent2, vertex, time)

    JPSCBSNode() : cost(0) {}
};

class JPSCBS {
public:
    std::vector<std::vector<Vertex>> solve(const std::vector<Agent>& agents, 
                                         const std::vector<std::vector<int>>& grid);

private:
    // 存储每个智能体的所有搜索到的路径
    std::unordered_map<int, std::vector<JPSPath>> solutions;
    std::unordered_map<int, JPSState> agent_states;
    std::vector<std::vector<int>> grid;
    
    // 核心函数
    JPSPath search_by_jps(const Agent& agent, const std::vector<std::vector<int>>& grid);
    void update_solutions(JPSCBSNode& node);
    void detect_conflicts(JPSCBSNode& node);
    bool find_alt_symmetric_paths(JPSCBSNode& node, 
                                const std::tuple<int, int, Vertex, int>& conflict);
    
    // 辅助函数
    std::vector<JPSPath> resolve_conflict_locally(const JPSCBSNode& node,
                                                const std::tuple<int, int, Vertex, int>& conflict,
                                                const Constraint& constraint);
    int calculate_sic(const std::unordered_map<int, std::priority_queue<JPSPath, 
                     std::vector<JPSPath>, JPSPathComparator>>& solution);
};

#endif // JPSCBS_H