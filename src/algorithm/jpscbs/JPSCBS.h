#ifndef JPSCBS_H
#define JPSCBS_H

#include "../Vertex.h"
#include "../jps/JPS.h"
#include "../Agent.h"
#include "../cbs/CBS.h"
#include "../astar/Astar.h"
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
    int cost;

    JPSCBSNode() : cost(0) {}
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
    
    // Get solving time
    double get_elapsed_time() const;

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
    std::vector<Conflict> detect_conflicts(const JPSCBSNode& node);
    bool find_alt_symmetric_paths(JPSCBSNode& node, 
                                const Conflict& conflict);
    
    void resolve_conflict_locally(JPSCBSNode& node, const Constraint& constraint);
    int calculate_sic(const std::unordered_map<int, std::priority_queue<JPSPath, 
                     std::vector<JPSPath>, JPSPathComparator>>& solution);
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
    bool find_local_bypass(const JPSPath& path, int agent_id, 
                              JPSCBSNode& node, const Vertex& conflict_vertex);
    
};

#endif // JPSCBS_H