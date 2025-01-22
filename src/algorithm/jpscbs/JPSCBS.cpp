#include "JPSCBS.h"

std::vector<std::vector<Vertex>> JPSCBS::solve(const std::vector<Agent>& agents, 
                                              const std::vector<std::vector<int>>& grid) {
    start_time = std::chrono::steady_clock::now();
    this->grid = grid;
    
    logger::log_info("Starting JPSCBS solver");
    
    // Initialize root node and solutions
    auto root = std::make_shared<JPSCBSNode>();
    
    // Find initial paths for each agent
    for (const auto& agent : agents) {
        JPSPath path = search_by_jps(agent);
        if (path.path.empty()) {
            logger::log_info("No initial path found for agent " + std::to_string(agent.id));
            return {};
        }
        
        solutions[agent.id].push_back(path);  // 存储原始路径
        
        // 创建新的优先队列并使用path的深拷贝
        std::priority_queue<JPSPath, std::vector<JPSPath>, JPSPathComparator> paths;
        paths.push(JPSPath(path));  // 创建并存储path的深拷贝
        
        root->solution[agent.id] = std::move(paths);  // 移动优先队列的所有权到root节点
    }

    root->cost = calculate_sic(*root);
    
    std::priority_queue<std::shared_ptr<JPSCBSNode>, 
                       std::vector<std::shared_ptr<JPSCBSNode>>, 
                       JPSCBSNodeComparator> open_list;
    open_list.push(root);
    
    print_node_info(*root, "初始节点");
    
    while (!open_list.empty()) {
        // Check timeout
        if (is_timeout()) {
            logger::log_warning("Time limit exceeded");
            return {};
        }
        
        auto current = open_list.top();
        open_list.pop();
        
        print_node_info(*current, "当前节点");
        logger::print_constraints(current->constraints, "当前节点约束");
        
        // Update current node's solution
        update_solutions(agents, *current);
        
        // 获取所有相关约束
        auto new_constraints = generate_constraints(*current);
        if (new_constraints.empty()) {
            std::vector<std::vector<Vertex>> final_paths;
            final_paths.reserve(agents.size());
            for (const auto& agent : agents) {
                final_paths.push_back(current->solution[agent.id].top().path);
            }
            return final_paths;
        }

        if(find_alt_symmetric_paths(*current, new_constraints)) {
            open_list.push(current);
            continue;
        }   
        
        // 为每个约束创建新的节点
        for (const auto& constraint : new_constraints) {
            logger::print_constraint(constraint, "处理约束");

            auto new_node = std::make_shared<JPSCBSNode>(*current);
            new_node->constraints.push_back(constraint);
            resolve_conflict_locally(*new_node, constraint);
            new_node->cost = calculate_sic(*new_node);
            open_list.push(new_node);
            print_node_info(*new_node, "新生成的子节点");
        }
    }
    
    logger::log_info("JPSCBS found no solution");
    return {};  // No solution
}

JPSPath JPSCBS::search_by_jps(const Agent& agent) {
    // 确保agent_states中有该智能体的状态
    if (agent_states.find(agent.id) == agent_states.end()) {
        agent_states[agent.id] = JPSState();
    }
    
    return jump_point_search(agent.start, agent.goal, grid, agent_states[agent.id]);
}

double JPSCBS::calculate_sic(const JPSCBSNode& node) {
    double total_cost = 0.0;
    for (const auto& [_, paths] : node.solution) {
        if (!paths.empty()) {
            const auto& path = paths.top().path;
            // 计算单条路径的代价
            total_cost += utils::calculate_path_cost(path);
        }
    }
    return total_cost;
}


void JPSCBS::update_solutions(const std::vector<Agent>& agents, JPSCBSNode& node) {
    for (auto& [agent_id, agent_paths] : node.solution) {
        // 如果该智能体的open_list为空，说明无法找到更多路径，跳过更新
        if (agent_states[agent_id].open_list.empty()) {
            continue;
        }
        
        // Check if we need to continue searching for a new path
        while (!agent_paths.empty() && !solutions[agent_id].empty() && 
               agent_paths.top().path.size() > solutions[agent_id].back().path.size()) {
            
            // Continue searching for a new path
            JPSPath new_path = search_by_jps(agents[agent_id]);
            
            if (!new_path.path.empty()) {
                solutions[agent_id].push_back(new_path);
            } else {
                // 如果找不到新路径，说明搜索已完成，清空open_list表示不需要继续搜索
                agent_states[agent_id].clear();
                break;
            }
        }
        
        // 确保i不会超出solutions的范围
        size_t start_idx = std::min(agent_paths.size(), solutions[agent_id].size());
        
        // Add all paths with lower cost to current node's solution
        for (size_t i = start_idx; i < solutions[agent_id].size(); i++) {
            JPSPath path = solutions[agent_id][i];  // 创建一个深拷贝
            if (path.path.size() < agent_paths.top().path.size()) {
                agent_paths.push(std::move(path));  // 使用移动语义避免额外拷贝
            }
        }
    }
}

std::vector<Constraint> JPSCBS::generate_constraints(const JPSCBSNode& node) {
    
    for (const auto& [agent1, paths1] : node.solution) {
        for (const auto& [agent2, paths2] : node.solution) {
            if (agent1 >= agent2) continue;
            
            const auto& path1 = paths1.top().path;
            const auto& path2 = paths2.top().path;

            auto constraints = utils::generate_constraints_from_conflict(agent1, agent2, path1, path2);
            if(!constraints.empty()) {
                logger::print_constraints(constraints, "发现新的约束");
                
                return constraints;
            }
        }
    }
    
    logger::log_info("未发现新的约束");
    return {};
}

bool JPSCBS::find_alt_symmetric_paths(JPSCBSNode& node, const std::vector<Constraint>& constraints) {
    // 记录原始节点的冲突数量
    int original_conflicts = count_conflicts(node);
    logger::log_info("原始节点的冲突数量: " + std::to_string(original_conflicts));

    for(const auto& constraint : constraints) {
        // 创建 current_path 的拷贝而不是引用
        JPSPath current_path = node.solution[constraint.agent].top();
        
        for (const auto& interval : current_path.possible_intervals) {
            // Ensure interval contains at least two points
            if (interval.jump_points.size() < 2) continue;
            
            // Find start and end positions of interval in path
            auto start_it = std::find(current_path.path.begin(), current_path.path.end(), interval.get_start());
            auto end_it = std::find(start_it, current_path.path.end(), interval.get_end());
            
            // Validate iterators
            if (start_it == current_path.path.end() || end_it == current_path.path.end() || start_it >= end_it) {
                continue;
            }
            
            // Check if constraint_vertex is in this path segment
            auto constraint_it = std::find(start_it, end_it + 1, constraint.vertex);
            if (constraint_it == end_it + 1) continue;

            // Calculate start time in path
            int start_time = std::distance(current_path.path.begin(), start_it);

            // Add temporary constraint
            std::vector<Constraint> temp_constraints = node.constraints;
            temp_constraints.emplace_back(constraint.agent, constraint.vertex, constraint.time);

            logger::log_info("尝试找到local bypass");
            logger::log_info("agent_id: " + std::to_string(constraint.agent));
            logger::log_info("interval: " + logger::vectorToString(interval.jump_points));
            logger::log_info("start_time: " + std::to_string(start_time));
            logger::print_constraints(temp_constraints,"temp constrains");

            // Try to find an alternative path
            auto alt_path = a_star(constraint.agent, interval.get_start(), interval.get_end(), 
                                 grid, temp_constraints, start_time);

            if (!alt_path.empty()) {
                // 计算原始路径中被替换部分的代价
                logger::log_info("找到绕行路径: " + logger::vectorToString(alt_path));
                std::vector<Vertex> original_segment(start_it, end_it + 1);
                double original_cost = utils::calculate_path_cost(original_segment);
                double alt_cost = utils::calculate_path_cost(alt_path);

                if (std::abs(alt_cost - original_cost) < 1e-6) {

                    auto& agent_paths = node.solution[constraint.agent];
                    agent_paths.pop();

                    std::vector<Vertex> new_path;
                    new_path.insert(new_path.end(), current_path.path.begin(), start_it);
                    new_path.insert(new_path.end(), alt_path.begin(), alt_path.end());
                    new_path.insert(new_path.end(), end_it + 1, current_path.path.end());

                    JPSPath updated_path = current_path;  // 使用之前保存的拷贝
                    updated_path.path = std::move(new_path);
                    agent_paths.push(std::move(updated_path));

                    // 计算新路径的冲突数量
                    int new_conflicts = count_conflicts(node);
                    logger::log_info("新路径的冲突数量: " + std::to_string(new_conflicts));

                    // 只有当冲突数量减少时才接受新路径
                    if (new_conflicts < original_conflicts) {
                        // 重新计算节点的代价
                        node.cost = calculate_sic(node);
                        print_node_info(node, "找到更好的对称绕行路径");
                        return true;
                    } else {
                        // 如果冲突没有减少，恢复原来的路径
                        agent_paths.pop();
                        agent_paths.push(std::move(current_path));  // 使用移动语义
                        logger::log_info("对称绕行路径未能减少冲突数量，放弃此路径");
                    }
                }else{
                    logger::log_info("绕行路径距离更远");
                }
            }
        }
    }
    return false;
}

void JPSCBS::resolve_conflict_locally(JPSCBSNode& node, const Constraint& constraint) {
    const auto& current_path = node.solution.at(constraint.agent).top();
    const auto& path = current_path.path;
    const auto& jump_points = current_path.jump_points;
    
    auto constraint_it = std::find(path.begin(), path.end(), constraint.vertex);
    if (constraint_it == path.end()) return;
    
    int constraint_idx = std::distance(path.begin(), constraint_it);
    
    for (size_t i = 0; i < jump_points.size() - 1; i++) {
        auto jp1_it = std::find(path.begin(), path.end(), jump_points[i]);
        auto jp2_it = std::find(jp1_it, path.end(), jump_points[i + 1]);
        
        if (jp1_it == path.end() || jp2_it == path.end()) continue;
        
        int jp1_idx = std::distance(path.begin(), jp1_it);
        int jp2_idx = std::distance(path.begin(), jp2_it);
        
        if (jp1_idx <= constraint_idx && constraint_idx <= jp2_idx) {
            find_and_update_solution(node, constraint.agent, current_path, 
                                   jump_points[i], i, jp1_it, jp2_it, jp1_idx);
            break;
        }
    }
}

void JPSCBS::find_and_update_solution(JPSCBSNode& node, int agent_id, 
                                    const JPSPath& current_path,
                                    const Vertex& start_jp, size_t jp_index,
                                    std::vector<Vertex>::const_iterator start_it,
                                    std::vector<Vertex>::const_iterator end_it,
                                    int start_time) {
    // First try to find a path within the current interval
    logger::log_info("本地冲突解决");
    logger::log_info("agent_id: " + std::to_string(agent_id));
    logger::log_info("start_jp: " + std::to_string(start_jp.x) + "," + std::to_string(start_jp.y));
    logger::log_info("current_path.jump_points[jp_index + 1]: " + std::to_string(current_path.jump_points[jp_index + 1].x) + "," + std::to_string(current_path.jump_points[jp_index + 1].y));
    logger::log_info("start_time: " + std::to_string(start_time));
    auto new_local_path = a_star(agent_id, start_jp, current_path.jump_points[jp_index + 1],
                                grid, node.constraints, start_time);
    
    if (new_local_path.empty()) return;
    
    // Check if it's the interval start
    bool is_interval_start = false;
    const Interval* target_interval = nullptr;
    
    for (const auto& interval : current_path.possible_intervals) {
        if (interval.get_start() == current_path.jump_points[jp_index + 1]) {
            is_interval_start = true;
            target_interval = &interval;
            break;
        }
    }
    
    if (!is_interval_start) {
        // Use current found path directly
        update_path_with_local_solution(node, agent_id, current_path, start_it, end_it, new_local_path);
        return;
    }
    
    // If it's the interval start, check subsequent jump points
    if (target_interval && target_interval->jump_points.size() > 1) {
        size_t next_jp_index = 0;
        // Find index of next jump point in jump_points
        for (size_t i = jp_index + 1; i < current_path.jump_points.size(); i++) {
            if (current_path.jump_points[i] == target_interval->jump_points[1]) {
                next_jp_index = i;
                break;
            }
        }
        
        if (has_better_solution(new_local_path, 
                              current_path.jump_points[jp_index + 1],
                              target_interval->jump_points[1])) {
            // Recursively try to find a longer path
            auto next_jp_it = std::find(current_path.path.begin(), current_path.path.end(), 
                                      target_interval->jump_points[1]);
            
            find_and_update_solution(node, agent_id, current_path,
                                   start_jp, next_jp_index - 1, 
                                   start_it, next_jp_it, start_time);
        } else {
            // Use current found path
            update_path_with_local_solution(node, agent_id, current_path,
                                             start_it, end_it, new_local_path);
        }
    }
}


bool JPSCBS::has_better_solution(const std::vector<Vertex>& new_path, 
                               const Vertex& jp2, 
                               const Vertex& next_jp) {
    // Calculate direction from next_jp to jp2
    Vertex direction = utils::calculateDirection(next_jp, jp2);
    
    // If direction is horizontal or vertical, no need to check
    if (direction.x == 0 || direction.y == 0) {
        return false;
    }
    
    // Use floating point for calculation to improve precision
    double dx = static_cast<double>(direction.x);
    double dy = static_cast<double>(direction.y);
    
    for (size_t i = 0; i < new_path.size() - 1; i++) {
        // Use floating point for distance and y coordinate calculation
        double distance = static_cast<double>(new_path[i].x - jp2.x) / dx;
        double y = jp2.y + dy * distance;
        
        if (static_cast<double>(new_path[i].y) >= y) {
            return true;
        }
    }
    return false;
}

// Helper function: update path
void JPSCBS::update_path_with_local_solution(JPSCBSNode& node, 
                                           int agent_id,
                                           const JPSPath& current_path,
                                           std::vector<Vertex>::const_iterator start_it,
                                           std::vector<Vertex>::const_iterator end_it,
                                           const std::vector<Vertex>& local_path) {
    auto& agent_paths = node.solution[agent_id];
    agent_paths.pop();
    
    // Build new path
    std::vector<Vertex> new_path;
    new_path.reserve(current_path.path.size() + local_path.size()); // 预分配空间
    
    // 1. 添加起点到start_it之间的路径
    new_path.insert(new_path.end(), 
                   current_path.path.begin(), 
                   start_it);
    
    // 2. 添加局部路径
    new_path.insert(new_path.end(), 
                   local_path.begin(), 
                   local_path.end());
    
    // 3. 添加end_it到终点的路径
    if (end_it != current_path.path.end()) {
        new_path.insert(new_path.end(), 
                       std::next(end_it), // 跳过end_it本身，避免重复
                       current_path.path.end());
    }
    
    // Update jump points
    std::vector<Vertex> new_jump_points;
    new_jump_points.reserve(current_path.jump_points.size()); // 预分配空间
    
    // Keep jump points before start point
    for (const auto& jp : current_path.jump_points) {
        if (std::find(current_path.path.begin(), start_it, jp) != start_it) {
            new_jump_points.push_back(jp);
        }
    }
    
    // Add jump points after end point
    if (end_it != current_path.path.end()) {
        for (const auto& jp : current_path.jump_points) {
            if (std::find(end_it, current_path.path.end(), jp) != current_path.path.end()) {
                new_jump_points.push_back(jp);
            }
        }
    }
    
    // 确保新路径有效
    if (new_path.empty()) {
        logger::log_warning("生成的新路径为空，保持原路径不变");
        agent_paths.push(current_path);
        return;
    }
    
    // Create updated path
    JPSPath updated_path = current_path;
    updated_path.path = std::move(new_path);
    updated_path.jump_points = std::move(new_jump_points);
    
    agent_paths.push(std::move(updated_path));
}

void JPSCBS::print_node_info(const JPSCBSNode& node, const std::string& prefix) {
    if (!prefix.empty()) {
        logger::log_info(prefix + ":");
    }
    logger::log_info("总代价: " + std::to_string(std::round(node.cost * 1000) / 1000.0));  // 保留3位小数
    
    for (const auto& [agent_id, paths] : node.solution) {
        if (!paths.empty()) {
            std::string path_str = "智能体 " + std::to_string(agent_id) + " 的路径: ";
            for (const auto& pos : paths.top().path) {
                path_str += "(" + std::to_string(pos.x) + "," + std::to_string(pos.y) + ") ";
            }
            logger::log_info(path_str);
        }
    }
    logger::log_info("------------------------");
}

int JPSCBS::count_conflicts(const JPSCBSNode& node) {
    int total_conflicts = 0;
    const auto& solution = node.solution;
    
    // 检查每对智能体之间的冲突
    for (auto it1 = solution.begin(); it1 != solution.end(); ++it1) {
        auto it2 = it1;
        ++it2;
        for (; it2 != solution.end(); ++it2) {
            total_conflicts += utils::count_conflicts(
                it1->first,  // agent1_id
                it2->first,  // agent2_id
                it1->second.top().path,  // path1
                it2->second.top().path   // path2
            );
        }
    }
    
    return total_conflicts;
}