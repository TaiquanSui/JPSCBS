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

struct ConstraintInfo {
    Constraint constraint;
    size_t jp1_path_index;    // 改用索引替代迭代器
    size_t jp2_path_index;
    size_t constraint_path_index;
    Vertex jp1;
    Vertex jp2;
    size_t jp1_jumps_index;
    size_t jp2_jumps_index;

    ConstraintInfo(const Constraint& c,
                  size_t jp1_path_index_,
                  size_t jp2_path_index_,
                  size_t constraint_path_index_,
                  const Vertex& jp1_,
                  const Vertex& jp2_,
                  size_t jp1_jumps_index_,
                  size_t jp2_jumps_index_)
        : constraint(c), jp1_path_index(jp1_path_index_), jp2_path_index(jp2_path_index_), 
          constraint_path_index(constraint_path_index_), jp1(jp1_), jp2(jp2_), 
          jp1_jumps_index(jp1_jumps_index_), jp2_jumps_index(jp2_jumps_index_) {}
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
    void validate_and_repair_solutions(const std::vector<Agent>& agents, JPSCBSNode& node);
    std::vector<Constraint> generate_constraints(const JPSCBSNode& node);
    bool find_alt_symmetric_paths(JPSCBSNode& node,
                                const std::vector<ConstraintInfo>& constraint_infos);
    
    void resolve_conflict_locally(JPSCBSNode& node,
                                const ConstraintInfo& constraint_info);
    double calculate_sic(const JPSCBSNode& node);
    void update_path_with_local_solution(JPSCBSNode& node, const ConstraintInfo& info, const std::vector<Vertex>& local_path);
    bool has_better_solution(const std::vector<Vertex>& new_path, 
                                  const Vertex& jp2, 
                                  const Vertex& next_jp);
    
    bool is_timeout() const {
        auto current_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>
                       (current_time - start_time);
        return duration.count() / 1000.0 > time_limit;
    }
    int count_conflicts(const JPSCBSNode& node);

    void print_node_info(const JPSCBSNode& node, const std::string& prefix);

    std::vector<ConstraintInfo> collect_constraint_infos(const JPSCBSNode& node, 
                                                       const std::vector<Constraint>& constraints);

    std::shared_ptr<JPSCBSNode> initialize(const std::vector<Agent>& agents);
};

#endif // JPSCBS_H