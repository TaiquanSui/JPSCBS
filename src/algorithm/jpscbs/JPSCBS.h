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
    // 每个智能体的多条路径（按照代价排序）
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

    // 设置超时时间
    void set_time_limit(double seconds) { time_limit = seconds; }
    
    // 获取求解时间
    double get_elapsed_time() const;

private:
    // 存储每个智能体的所有搜索到的路径
    std::unordered_map<int, std::vector<JPSPath>> solutions;
    std::unordered_map<int, JPSState> agent_states;
    std::vector<std::vector<int>> grid;
    
    // 超时相关
    std::chrono::steady_clock::time_point start_time;
    double time_limit = 30.0;  // 默认30秒超时
    
    // 核心函数
    JPSPath search_by_jps(const Agent& agent);
    void update_solutions(const std::vector<Agent>& agents, JPSCBSNode& node);
    std::vector<Conflict> detect_conflicts(const JPSCBSNode& node);
    bool find_alt_symmetric_paths(JPSCBSNode& node, 
                                const Conflict& conflict);
    
    // 辅助函数
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