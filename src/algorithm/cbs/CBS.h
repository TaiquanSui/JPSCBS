#ifndef CBS_H
#define CBS_H

#include "../Vertex.h"
#include "../Agent.h"
#include <vector>
#include <unordered_map>
#include <chrono>

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
    double cost;

    CBSNode() : cost(0.0) {}
};

class CBS {
public:
    explicit CBS(bool use_bypass = true) : use_bypass(use_bypass) {}
    std::vector<std::vector<Vertex>> solve(const std::vector<Agent>& agents, 
                                         const std::vector<std::vector<int>>& grid);
    void set_time_limit(double seconds) { time_limit = seconds; }
    int get_expanded_nodes() const { return expanded_nodes; }
    double get_runtime() const {
        auto current_time = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>
               (current_time - start_time).count() / 1000.0;
    }

private:
    bool use_bypass;
    double time_limit = 30.0;
    std::chrono::steady_clock::time_point start_time;
    int expanded_nodes = 0;  // 添加计数器
    
    std::vector<Constraint> generate_constraints(const CBSNode& node);
    std::vector<Vertex> find_path(const Agent& agent,
                                const std::vector<std::vector<int>>& grid,
                                const std::vector<Constraint>& constraints);
    bool find_bypass(CBSNode& node, const std::vector<Agent>& agents, 
                     const std::vector<Constraint>& constraints,
                     const std::vector<std::vector<int>>& grid);
    
    bool is_timeout() const;
    void print_node_info(const CBSNode& node, const std::string& prefix = "");

    // 计算单条路径的代价
    double calculate_sic(const CBSNode& node);
    int count_conflicts(const CBSNode& node);
};

#endif // CBS_H