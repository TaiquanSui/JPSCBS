#ifndef JPSCBS_H
#define JPSCBS_H

#include "../Vertex.h"
#include "../jps/JPS.h"
#include "../Agent.h"
#include "../cbs/CBS.h"
#include "../astar/Astar.h"
#include "../utilities/Log.h"
#include <unordered_map>
#include <queue>
#include <vector>
#include <tuple>
#include <chrono>
#include <iostream>

struct JPSCBSNode {
    // Multiple paths for each agent (sorted by cost)
    std::unordered_map<int, std::priority_queue<JPSPath, std::vector<JPSPath>, JPSPathComparator>> solution;
    std::vector<Constraint> constraints;
    double cost;

    JPSCBSNode() : cost(0.0) {}
};

struct JPSCBSNodeComparator {
    bool operator()(const std::shared_ptr<JPSCBSNode>& a, 
                   const std::shared_ptr<JPSCBSNode>& b) const {
        return a->cost > b->cost;
    }
};

class JPSCBS {
public:
    std::vector<std::vector<Vertex>> solve(const std::vector<Agent>& agents, 
                                         const std::vector<std::vector<int>>& grid);

    // Set timeout
    void set_time_limit(double seconds) { time_limit = seconds; }

private:
    // Store all paths found for each agent
    std::unordered_map<int, std::vector<JPSPath>> solutions;
    std::unordered_map<int, JPSState> agent_states;
    std::vector<std::vector<int>> grid;
    
    // Timeout related
    std::chrono::steady_clock::time_point start_time;
    double time_limit = 30.0;  // Default 30 seconds timeout
    
    // Core functions
    JPSPath search_by_jps(const Agent& agent);
    void update_solutions(const std::vector<Agent>& agents, JPSCBSNode& node);
    std::vector<Constraint> generate_constraints(const JPSCBSNode& node);
    bool find_alt_symmetric_paths(JPSCBSNode& node, 
                                const std::vector<Constraint>& constraints);
    
    void resolve_conflict_locally(JPSCBSNode& node, const Constraint& constraint);
    double calculate_sic(const JPSCBSNode& node);
    void update_path_with_local_solution(JPSCBSNode& node, 
                                           int agent_id,
                                           const JPSPath& current_path,
                                           std::vector<Vertex>::const_iterator start_it,
                                           std::vector<Vertex>::const_iterator end_it,
                                           const std::vector<Vertex>& local_path);
    bool has_better_solution(const std::vector<Vertex>& new_path, 
                                  const Vertex& jp2, 
                                  const Vertex& next_jp);
    void find_and_update_solution(JPSCBSNode& node, int agent_id, 
                                    const JPSPath& current_path,
                                    const Vertex& start_jp, size_t jp_index,
                                    std::vector<Vertex>::const_iterator start_it,
                                    std::vector<Vertex>::const_iterator end_it,
                                    int start_time);
    
    bool is_timeout() const {
        auto current_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>
                       (current_time - start_time);
        return duration.count() / 1000.0 > time_limit;
    }
    int count_conflicts(const JPSCBSNode& node);

    void print_node_info(const JPSCBSNode& node, const std::string& prefix);
};

#endif // JPSCBS_H