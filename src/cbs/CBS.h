#ifndef CBS_H
#define CBS_H

#include "../basic/Vertex.h"
#include "../basic/Agent.h"
#include "../basic/Constraint.h"
#include "../basic/Conflict.h"
#include "ConflictAvoidanceTable.h"
#include <chrono>
#include <atomic>
#include <unordered_map>
#include <unordered_set>


struct CBSNode {
    std::unordered_map<int, std::vector<Vertex>> solution;
    std::vector<Constraint> constraints;
    double cost;

    CBSNode() : cost(0.0) {}
};

struct CBSBypassResult {
    bool success;
    std::vector<std::pair<Constraint, std::vector<Vertex>>> bypass_paths;
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

    void interrupt() { interrupted = true; }
    void reset_interrupt() { interrupted = false; }
    bool should_terminate() const { return interrupted; }

private:
    bool use_bypass;
    double time_limit = 30.0;
    std::chrono::steady_clock::time_point start_time;
    int expanded_nodes = 0;
    std::atomic<bool> interrupted{false};  // 内部中断状态
            
    std::vector<Constraint> generate_constraints(const CBSNode& node);
    CBSBypassResult find_bypass(CBSNode& node, const std::vector<Constraint>& constraints, const std::vector<Agent>& agents, 
                     const std::vector<std::vector<int>>& grid);
    
    bool is_timeout() const;
    void print_node_info(const CBSNode& node, const std::string& prefix = "");

    // 计算单条路径的代价
    double calculate_sic(const CBSNode& node);
    int count_conflicts(const CBSNode& node);

    ConflictAvoidanceTable calculate_cat(const std::unordered_set<int>& excluded_agents, 
                                       const CBSNode& node) const;
};

#endif // CBS_H