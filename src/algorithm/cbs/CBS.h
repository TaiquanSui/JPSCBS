#ifndef CBS_H
#define CBS_H

#include "../Vertex.h"
#include "../Agent.h"
#include <vector>
#include <unordered_map>

struct Conflict {
    int agent1;
    int agent2;
    Vertex vertex;
    int time;
    
    Conflict(int a1, int a2, const Vertex& v, int t) 
        : agent1(a1), agent2(a2), vertex(v), time(t) {}
};

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

    CBSNode() : cost(0) {}
};

class CBS {
public:
    explicit CBS(bool use_bypass = true) : use_bypass(use_bypass) {}
    std::vector<std::vector<Vertex>> solve(const std::vector<Agent>& agents, 
                                         const std::vector<std::vector<int>>& grid);

private:
    bool use_bypass;
    
    std::vector<Conflict> detect_conflicts(const CBSNode& node);
    std::vector<Vertex> find_path(const Agent& agent,
                                const std::vector<std::vector<int>>& grid,
                                const std::vector<Constraint>& constraints);
    bool find_bypass(CBSNode& node, const Agent& agent, const Vertex& conflict_vertex, 
                    int conflict_time, const std::vector<std::vector<int>>& grid);
};

#endif // CBS_H