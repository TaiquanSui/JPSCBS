#include "JPSCBS.h"

std::vector<std::vector<Vertex>> JPSCBS::solve(const std::vector<Agent>& agents, 
                                              const std::vector<std::vector<int>>& grid) {
    this->grid = grid;
    // 初始化根节点
    JPSCBSNode root;
    
    // 为每个智能体找到初始路径
    for (const auto& agent : agents) {
        auto path = search_by_jps(agent, grid);
        if (path.path.empty()) return {}; // 无解
        root.solution[agent.id].push(path);
        solutions[agent.id].push_back(path);
    }
    
    // 为每个智能体添加一条额外的路径
    for (const auto& agent : agents) {
        auto path = search_by_jps(agent, grid);
        if (!path.path.empty()) {
            root.solution[agent.id].push(path);
            solutions[agent.id].push_back(path);
        }
    }
    
    // 初始化开放列表
    auto compare = [](const JPSCBSNode& a, const JPSCBSNode& b) { 
        return a.cost > b.cost; 
    };
    std::priority_queue<JPSCBSNode, std::vector<JPSCBSNode>, decltype(compare)> open_list(compare);
    open_list.push(root);
    
    // 主循环
    while (!open_list.empty()) {
        JPSCBSNode current = open_list.top();
        open_list.pop();
        
        // 更新当前节点的解
        update_solutions(current);
        
        // 检测冲突
        detect_conflicts(current);
        
        // 如果没有冲突，返回解
        if (current.conflicts.empty()) {
            std::vector<std::vector<Vertex>> result;
            for (const auto& [agent_id, paths] : current.solution) {
                result.push_back(paths.top().path);
            }
            return result;
        }
        
        // 获取第一个冲突
        auto conflict = current.conflicts.front();
        
        // 尝试找到替代的对称路径
        if (find_alt_symmetric_paths(current, conflict)) {
            continue;
        }
        
        // 为冲突中的每个智能体创建新节点
        int agent1 = std::get<0>(conflict);
        int agent2 = std::get<1>(conflict);
        Vertex conflict_vertex = std::get<2>(conflict);
        int conflict_time = std::get<3>(conflict);
        
        for (int agent_id : {agent1, agent2}) {
            JPSCBSNode child = current;
            Constraint constraint(agent_id, conflict_vertex, conflict_time);
            child.constraints.push_back(constraint);
            
            // 局部解决冲突
            auto new_paths = resolve_conflict_locally(current, conflict, constraint);
            for (const auto& path : new_paths) {
                child.solution[agent_id].push(path);
            }
            
            // 计算新节点的代价
            child.cost = calculate_sic(child.solution);
            
            // 将新节点加入开放列表
            open_list.push(child);
        }
    }
    
    return {}; // 无解
}

JPSPath JPSCBS::search_by_jps(const Agent& agent, const std::vector<std::vector<int>>& grid) {
    // 如果状态不存在，创建新的状态
    if (agent_states.count(agent.id) == 0) {
        agent_states[agent.id] = JPSState();
    }
    
    // 使用存储的状态继续搜索
    return jump_point_search(agent.start, agent.goal, grid, agent_states[agent.id]);
}

void JPSCBS::update_solutions(JPSCBSNode& node) {
    for (auto& [agent_id, agent_solutions] : node.solution) {
        while (!agent_solutions.empty() && 
               agent_solutions.top().path.size() > solutions[agent_id].back().path.size()) {
            auto new_path = search_by_jps(Agent(agent_id, 
                                              solutions[agent_id][0].path.front(),
                                              solutions[agent_id][0].path.back()), 
                                        grid);
            if (new_path.path.empty()) break;
            solutions[agent_id].push_back(new_path);
        }
        
        // 添加所有代价更低的路径
        for (const auto& path : solutions[agent_id]) {
            if (path.path.size() < agent_solutions.top().path.size()) {
                agent_solutions.push(path);
            }
        }
    }
}

void JPSCBS::detect_conflicts(JPSCBSNode& node) {
    node.conflicts.clear();
    
    // 遍历所有智能体对
    for (const auto& [agent1_id, paths1] : node.solution) {
        for (const auto& [agent2_id, paths2] : node.solution) {
            if (agent1_id >= agent2_id) continue;  // 避免重复检查
            
            // 获取当前使用的路径（优先队列的顶部）
            const auto& path1 = paths1.top().path;
            const auto& path2 = paths2.top().path;
            
            // 获取最长路径的长度
            size_t max_length = std::max(path1.size(), path2.size());
            
            // 检查每个时间点的冲突
            for (size_t t = 0; t < max_length; ++t) {
                // 获取两个智能体在时间t的位置
                Vertex pos1 = t < path1.size() ? path1[t] : path1.back();  // 如果到达终点则保持不动
                Vertex pos2 = t < path2.size() ? path2[t] : path2.back();
                
                // 检查顶点冲突
                if (pos1 == pos2) {
                    node.conflicts.emplace_back(agent1_id, agent2_id, pos1, t);
                }
            }
        }
    }
}
