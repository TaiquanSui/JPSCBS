#ifndef JPSCBS_H
#define JPSCBS_H

#include "../basic/Vertex.h"
#include "../basic/Agent.h"
#include "../astar/AStar.h"  // 先包含 AStar.h
#include "../jps/JPS.h"      // 最后包含 JPS.h
#include "../basic/Constraint.h"
#include "../basic/Conflict.h"
#include <unordered_map>
#include <queue>
#include <vector>
#include <chrono>
#include <sstream>
#include <atomic>
#include <unordered_set>

struct JPSCBSNode {
    std::unordered_map<int, std::priority_queue<JPSPath, std::vector<JPSPath>, JPSPathComparator>> solution;
    std::vector<Constraint> constraints;
    double cost;

    JPSCBSNode() : cost(0.0) {}

    JPSCBSNode(const JPSCBSNode& other) 
        : solution(other.solution)  // priority_queue会执行深拷贝
        , constraints(other.constraints)  // vector会执行深拷贝
        , cost(other.cost) 
    {}
};

struct JPSCBSNodeComparator {
    bool operator()(const std::shared_ptr<JPSCBSNode>& a, 
                   const std::shared_ptr<JPSCBSNode>& b) const {
        if (std::abs(a->cost - b->cost) < 1e-6) {
            int a_conflicts = count_conflicts(*a);
            int b_conflicts = count_conflicts(*b);
            if (a_conflicts != b_conflicts) {
                return a_conflicts > b_conflicts;
            }
            return a.get() > b.get();
        }
        return a->cost > b->cost;
    }

private:
    static int count_conflicts(const JPSCBSNode& node);  // 只保留声明
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

    std::string toString() const {
        std::stringstream ss;
        ss << "ConstraintInfo{\n"
           << "  constraint: Agent " << constraint.agent 
           << " cannot reach position (" << (constraint.type == Constraint::EDGE ? 
           constraint.edge_constraint.v1.x + "," + constraint.edge_constraint.v1.y 
           : constraint.vertex_constraint.vertex.x + "," + constraint.vertex_constraint.vertex.y) 
           << ") at time " << constraint.time << "\n"
           << "  jp1: (" << jp1.x << "," << jp1.y 
           << ") [path_idx: " << jp1_path_index 
           << ", jumps_idx: " << jp1_jumps_index << "]\n"
           << "  jp2: (" << jp2.x << "," << jp2.y 
           << ") [path_idx: " << jp2_path_index 
           << ", jumps_idx: " << jp2_jumps_index << "]\n"
           << "  constraint_path_index: " << constraint_path_index << "\n"
           << "}";
        return ss.str();
    }
};

// 添加新的返回类型结构体
struct JPSBypassResult {
    bool success;
    std::vector<std::pair<ConstraintInfo, std::vector<Vertex>>> bypass_paths;
};

class JPSCBS {
public:

    std::vector<std::vector<Vertex>> solve(const std::vector<Agent>& agents, 
                                         const std::vector<std::vector<int>>& grid);

    // Set timeout
    void set_time_limit(double seconds) { time_limit = seconds; }

    int get_expanded_nodes() const { return expanded_nodes; }

    void interrupt() { interrupted = true; }
    void reset_interrupt() { interrupted = false; }
    bool should_terminate() const { return interrupted; }

private:
    // Store all paths found for each agent
    std::unordered_map<int, std::vector<JPSPath>> solutions;
    std::unordered_map<int, JPSState> agent_states;
    std::vector<std::vector<int>> grid;
    
    double time_limit = 30.0;
    int expanded_nodes = 0;
    
    std::atomic<bool> interrupted{false};  // 内部中断状态
    
    // Core functions
    JPSPath search_by_jps(const Agent& agent);
    void update_solutions(const std::vector<Agent>& agents, JPSCBSNode& node);
    void validate_and_repair_solutions(const std::vector<Agent>& agents, JPSCBSNode& node);
    std::vector<Constraint> generate_constraints(const JPSCBSNode& node);
    JPSBypassResult find_bypass(JPSCBSNode& node,
                           const std::vector<ConstraintInfo>& constraint_infos);
    
    bool resolve_conflict_locally(JPSCBSNode& node,
                                const ConstraintInfo& constraint_info,
                                const std::vector<Vertex>& local_path);
    double calculate_sic(const JPSCBSNode& node);


    void update_path_with_local_solution(JPSCBSNode& node, const ConstraintInfo& info, const std::vector<Vertex>& local_path);

    bool has_better_solution(const std::vector<Vertex>& new_path,
                            const ConstraintInfo& info,
                            const Vertex& next_jp_decide);
                                  
    std::pair<ConstraintInfo, std::vector<Vertex>> find_alt_path(
        const ConstraintInfo& info,
        const JPSPath& current_path,
        const std::vector<Constraint>& temp_constraints,
        const ConflictAvoidanceTable& cat);
    

    int count_conflicts(const JPSCBSNode& node);
    void print_node_info(const JPSCBSNode& node, const std::string& prefix);
    std::vector<ConstraintInfo> collect_constraint_infos(const JPSCBSNode& node, 
                                                       const std::vector<Constraint>& constraints);
    std::shared_ptr<JPSCBSNode> initialize(const std::vector<Agent>& agents);
    ConflictAvoidanceTable calculate_cat(const std::unordered_set<int>& excluded_agents, const JPSCBSNode& node) const;
};

#endif // JPSCBS_H