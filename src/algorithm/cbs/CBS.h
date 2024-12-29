#ifndef CBS_H
#define CBS_H

#include "../Vertex.h"
#include <vector>
#include <unordered_map>

struct Constraint {
    int agent;
    Vertex vertex;
    int time;
    
    Constraint(int a, const Vertex& v, int t) : agent(a), vertex(v), time(t) {}
};

struct CBSNode {
    std::unordered_map<int, std::vector<Vertex>> solution;
    std::vector<Constraint> constraints;
    int cost;
    std::vector<std::tuple<int, int, Vertex, int>> conflicts; // (agent1, agent2, vertex, time)

    CBSNode() : cost(0) {}
};

class CBS {
public:
    explicit CBS(bool use_bypass = true) : use_bypass(use_bypass) {}
    std::vector<std::vector<Vertex>> solve(const std::vector<std::pair<Vertex, Vertex>>& agents, 
                                         const std::vector<std::vector<int>>& grid);

private:
    bool use_bypass;
    
    // 核心函数
    void detect_conflicts(CBSNode& node);
    std::vector<Vertex> find_path(const Vertex& start, const Vertex& goal,
                                const std::vector<std::vector<int>>& grid,
                                const std::vector<Constraint>& constraints,
                                int agent_id);
    bool validate_path(const std::vector<Vertex>& path, 
                      const std::vector<Constraint>& constraints,
                      int agent_id);
    
    // 辅助函数
    bool find_bypass(CBSNode& node, int agent, const Vertex& conflict_vertex, 
                    int conflict_time, const std::vector<std::vector<int>>& grid);
    void resolve_conflict(CBSNode& node, const std::tuple<int, int, Vertex, int>& conflict,
                         std::vector<CBSNode>& children, const std::vector<std::vector<int>>& grid);
};

#endif // CBS_H