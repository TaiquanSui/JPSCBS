#include "JPSCBS.h"

std::vector<std::vector<Vertex>> JPSCBS::solve(const std::vector<Agent>& agents, 
                                              const std::vector<std::vector<int>>& grid) {
    logger::log_info("Starting JPSCBS solver");
    
    this->grid = grid;
    this->solutions.clear();        // 清空之前的解决方案
    this->agent_states.clear();     // 清空之前的智能体状态
    expanded_nodes = 0;             // 重置节点计数器
    
    // 记录开始时间
    auto start_time = std::chrono::steady_clock::now();
    
    // Initialize root node and solutions
    auto root = initialize(agents);
    if(root == nullptr) {
        logger::log_info("No solution");
        return {};
    }
    print_node_info(*root, "Initial Node");
    
    std::priority_queue<std::shared_ptr<JPSCBSNode>, 
                       std::vector<std::shared_ptr<JPSCBSNode>>, 
                       JPSCBSNodeComparator> open_list;
    open_list.push(root);
    
    while (!open_list.empty()) {
        // 只检查是否应该终止
        if (should_terminate()) {
            logger::log_info("JPSCBS solver interrupted");
            return {};
        }

        auto current = open_list.top();
        if (!current) {
            logger::log_error("遇到空节点，跳过处理");
            open_list.pop();
            continue;
        }
        open_list.pop();
        expanded_nodes++;
        
        print_node_info(*current, "Current Node");
        
        // Update current node's solution
        update_solutions(agents, *current);
        
        // 验证和修复所有路径
        validate_and_repair_solutions(agents, *current);
        
        // 获取所有相关约束
        auto new_constraints = generate_constraints(*current);
        
        if (new_constraints.empty()) {
            std::vector<std::vector<Vertex>> final_paths;
            final_paths.reserve(agents.size());
            double cost = calculate_sic(*current);
            logger::log_info("JPSCBS found solution with cost: " + std::to_string(cost));
            for (const auto& agent : agents) {
                final_paths.push_back(current->solution[agent.id].top().path);
            }
            return final_paths;
        }

        auto constraint_infos = collect_constraint_infos(*current, new_constraints);

        if(find_alt_symmetric_paths(*current, constraint_infos)) {
            open_list.push(current);
            continue;
        }   
        
        // 为每个约束创建新的节点
        for (const auto& info : constraint_infos) {
            logger::log_info("Processing constraint, constraint info:");
            logger::log_info(info.toString());

            auto new_node = std::make_shared<JPSCBSNode>(*current);
            new_node->constraints.push_back(info.constraint);
            resolve_conflict_locally(*new_node, info);
            new_node->cost = calculate_sic(*new_node);
            open_list.push(new_node);
            print_node_info(*new_node, "New Child Node Generated");
        }
    }
    
    logger::log_info("JPSCBS found no solution");
    return {};  // No solution
}

std::shared_ptr<JPSCBSNode> JPSCBS::initialize(const std::vector<Agent>& agents) {
    auto root = std::make_shared<JPSCBSNode>();
    
    // Find initial paths for each agent
    for (const auto& agent : agents) {
        JPSPath path = search_by_jps(agent);
        if (path.path.empty()) {
            logger::log_info("No initial path found for agent " + std::to_string(agent.id));
            return nullptr;
        }
        
        solutions[agent.id].push_back(path);  // 存储原始路径
        
        // 创建新的优先队列并使用path的深拷贝
        std::priority_queue<JPSPath, std::vector<JPSPath>, JPSPathComparator> paths;
        paths.push(JPSPath(path));  // 创建并存储path的深拷贝
        
        root->solution[agent.id] = std::move(paths);  // 移动优先队列的所有权到root节点
    }

    root->cost = calculate_sic(*root);
    return root;
}

void JPSCBS::update_solutions(const std::vector<Agent>& agents, JPSCBSNode& node) {
    for (auto& [agent_id, agent_paths] : node.solution) {
        // 检查 agent_states 中是否存在该智能体
        if (agent_states.find(agent_id) == agent_states.end()) {
            logger::log_warning("can't find agent " + std::to_string(agent_id) + " state");
            continue;
        }
        // 如果该智能体的open_list为空，说明无法找到更多路径，跳过更新
        if (agent_states[agent_id].open_list.empty()) {
            continue;
        }

        double node_solutions_cost = utils::calculate_path_cost(agent_paths.top().path);
        double solutions_cost = utils::calculate_path_cost(solutions[agent_id].back().path);
        // Check if we need to continue searching for a new path
        while (!agent_paths.empty() && !solutions[agent_id].empty() && 
               node_solutions_cost > solutions_cost) {
            
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
            if (utils::calculate_path_cost(path.path) < utils::calculate_path_cost(agent_paths.top().path)) {
                agent_paths.push(std::move(path));  // 使用移动语义避免额外拷贝
            }
        }
    }
}

JPSPath JPSCBS::search_by_jps(const Agent& agent) {
    // 确保agent_states中有该智能体的状态
    if (agent_states.find(agent.id) == agent_states.end()) {
        agent_states[agent.id] = JPSState();
        auto start_node = std::make_shared<AStarNode>(
            agent.start, 0, heuristic(agent.start, agent.goal));
        agent_states[agent.id].open_list.push(start_node);
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


void JPSCBS::validate_and_repair_solutions(const std::vector<Agent>& agents, JPSCBSNode& node) {
    logger::log_info("Starting to validate and repair paths");

    for (const auto& agent : agents) {
        if (node.solution.find(agent.id) == node.solution.end()) {
            logger::log_error("can't find agent " + std::to_string(agent.id) + " solution");
            continue;
        }
        
        auto& agent_paths = node.solution[agent.id];
        
        // 如果没有可用路径，直接返回失败
        if (agent_paths.empty()) {
            logger::log_warning("Agent " + std::to_string(agent.id) + " has no available paths");
            continue;
        }

        while (!agent_paths.empty()) {
            auto current_path = agent_paths.top();
            
            // 检查当前路径是否满足所有约束
            auto violated_constraints = utils::find_violated_constrains(agent.id, current_path.path, node.constraints);
            
            if (violated_constraints.empty()) {
                // 如果路径有效，继续下一个智能体
                logger::log_info("Agent " + std::to_string(agent.id) + " path is valid");
                break;
            } else {
                logger::log_info("Agent " + std::to_string(agent.id) + " path violates constraints, attempting repair");
                
                for (const auto& constraint : violated_constraints) {
                    // 使用A*寻找绕过约束的新路径
                    auto constraint_infos = collect_constraint_infos(node, {constraint});
                    if (constraint_infos.empty()) {
                        logger::log_info("Repair found constraint is problematic");
                    }
                    
                    logger::log_info(constraint_infos[0].toString());

                    if(find_alt_symmetric_paths(node, constraint_infos)) {
                        continue;
                    }
                    resolve_conflict_locally(node, constraint_infos[0]);
                    logger::print_constraint(constraint, "Repaired constraint: ");
                    print_node_info(node, "Repaired node");
                }
                
            }
        }

    }

}


std::vector<Constraint> JPSCBS::generate_constraints(const JPSCBSNode& node) {
    std::vector<Constraint> constraints;

    int t = std::max_element(
        node.solution.begin(), 
        node.solution.end(),
        [](const auto& a, const auto& b) { 
            return a.second.top().path.size() < b.second.top().path.size(); 
        })->second.top().path.size();
    
    for (int i = 0; i < t; ++i) {
        for (const auto& [agent1, paths1] : node.solution) {
            for (const auto& [agent2, paths2] : node.solution) {
                if (agent1 >= agent2) continue;

                const auto& path1 = paths1.top().path;
                const auto& path2 = paths2.top().path;

                Vertex pos1 = i < path1.size() ? path1[i] : path1.back();
                Vertex pos2 = i < path2.size() ? path2[i] : path2.back();

                if (pos1 == pos2) {
                    constraints.emplace_back(agent1, pos1, i);  
                    constraints.emplace_back(agent2, pos2, i);
                    continue;
                }

                if (i < t - 1) {
                    Vertex next_pos1 = (i + 1) < path1.size() ? path1[i + 1] : path1.back();
                    Vertex next_pos2 = (i + 1) < path2.size() ? path2[i + 1] : path2.back();

                    // check swapping conflict
                    if (pos1 == next_pos2 && pos2 == next_pos1) {
                        constraints.emplace_back(agent1, pos1, i);
                        constraints.emplace_back(agent2, pos2, i);
                        continue;
                    }

                    // check following conflict
                    // if (next_pos1 == pos2) { // agent1 follows agent2
                    //     constraints.emplace_back(agent1, pos2, t+1);
                    //     constraints.emplace_back(agent2, pos2, t);
                    //     continue;
                    // }
                    // if (next_pos2 == pos1) { // agent2 follows agent1
                    //     constraints.emplace_back(agent1, pos1, t);
                    //     constraints.emplace_back(agent2, pos1, t+1);
                    //     continue;
                    // }
                }
            }
        }
    }
    
    logger::log_info("No new constraints found");
    return constraints;
}

bool JPSCBS::find_alt_symmetric_paths(JPSCBSNode& node, 
                                    const std::vector<ConstraintInfo>& constraint_infos) {
    int original_conflicts = count_conflicts(node);
    logger::log_info("Original conflict count: " + std::to_string(original_conflicts));

    for(const auto& info : constraint_infos) {
        auto& agent_paths = node.solution[info.constraint.agent];
        auto current_path = agent_paths.top();  // 创建一个副本
        
        bool is_interval_start = false;
        for(const auto& interval : current_path.possible_intervals) {
            if(interval.get_start() == info.jp1) {
                is_interval_start = true;
                break;
            }
        }
        if(!is_interval_start) continue;

        // Add temporary constraint
        std::vector<Constraint> temp_constraints = node.constraints;
        temp_constraints.push_back(info.constraint);

        logger::log_info("Attempting to find local bypass");
        logger::log_info("agent_id: " + std::to_string(info.constraint.agent));
        logger::log_info("start_time: " + std::to_string(info.jp1_path_index));
        logger::log_info("end_time: " + std::to_string(info.jp2_path_index));
        logger::print_constraints(temp_constraints, "Temporary constraints");

        auto alt_path = a_star(info.constraint.agent, info.jp1, info.jp2,
                             grid, temp_constraints, 
                             info.jp1_path_index);

        if (!alt_path.empty()) {
            logger::log_info("Found bypass path: " + logger::vectorToString(alt_path));
            
            // 使用索引创建原始段
            std::vector<Vertex> original_segment(
                current_path.path.begin() + info.jp1_path_index,
                current_path.path.begin() + info.jp2_path_index + 1
            );
            double original_cost = utils::calculate_path_cost(original_segment);
            double alt_cost = utils::calculate_path_cost(alt_path);

            if (std::abs(alt_cost - original_cost) < 1e-6) {
                // 创建新路径
                std::vector<Vertex> new_path;
                new_path.insert(new_path.end(), 
                              current_path.path.begin(), 
                              current_path.path.begin() + info.jp1_path_index);
                new_path.insert(new_path.end(), 
                              alt_path.begin(), 
                              alt_path.end());
                new_path.insert(new_path.end(), 
                              current_path.path.begin() + info.jp2_path_index + 1, 
                              current_path.path.end());

                // 创建更新后的路径对象
                JPSPath updated_path = current_path;
                updated_path.path = std::move(new_path);
                
                // 保存原始路径，以便需要时恢复
                auto original_path = current_path;
                
                // 更新agent的路径
                agent_paths.pop();
                agent_paths.push(std::move(updated_path));

                int new_conflicts = count_conflicts(node);
                logger::log_info("New conflict count: " + std::to_string(new_conflicts));

                if (new_conflicts < original_conflicts) {
                    node.cost = calculate_sic(node);
                    print_node_info(node, "Found better symmetric bypass path");
                    return true;
                } else {
                    // 恢复原始路径
                    agent_paths.pop();
                    agent_paths.push(original_path);
                    logger::log_info("Symmetric bypass path did not reduce conflicts, discarding path");
                }
            }else{
                logger::log_info("Bypass path is longer");
                //从possible_intervals中移除此interval
                current_path.possible_intervals.erase(std::remove_if(current_path.possible_intervals.begin(), 
                                                                     current_path.possible_intervals.end(), 
                                                                     [info](const Interval& interval) {
                                                                         return interval.get_start() == info.jp1;
                                                                     }), 
                                                                     current_path.possible_intervals.end());
            }
        }
    }
    return false;
}

void JPSCBS::resolve_conflict_locally(JPSCBSNode& node, const ConstraintInfo& info) {
    if (node.solution.find(info.constraint.agent) == node.solution.end()) {
        logger::log_error("找不到智能体 " + std::to_string(info.constraint.agent) + " 的解决方案");
        return;
    }

    auto& agent_paths = node.solution[info.constraint.agent];
    if (agent_paths.empty()) {
        logger::log_error("智能体 " + std::to_string(info.constraint.agent) + " 没有可用路径");
        return;
    }

    auto current_path = agent_paths.top();
    if (info.jp1_path_index >= current_path.path.size() || 
        info.jp2_path_index >= current_path.path.size()) {
        logger::log_error("路径索引超出范围");
        return;
    }

    // First try to find a path within the current interval
    logger::log_info("Local conflict resolution");
    logger::log_info("agent_id: " + std::to_string(info.constraint.agent));
    logger::log_info("start_jp: " + std::to_string(info.jp1.x) + "," + std::to_string(info.jp1.y));
    logger::log_info("end_jp: " + std::to_string(info.jp2.x) + "," + std::to_string(info.jp2.y));
    logger::log_info("start_time: " + std::to_string(info.jp1_path_index));
    logger::log_info("end_time: " + std::to_string(info.jp2_path_index));
    logger::print_constraints(node.constraints, "Constraints");
    auto& path = current_path.path;
    
    // 使用索引获取对应的迭代器
    auto jp1_it = path.begin() + info.jp1_path_index;
    auto jp2_it = path.begin() + info.jp2_path_index;
    
    // 尝试找到局部路径
    auto new_local_path = a_star(info.constraint.agent, info.jp1, info.jp2,
                                grid, node.constraints, info.jp1_path_index);
    
    if (new_local_path.empty()) return;
    
    // Check if it's the interval start
    bool is_interval_start = false;
    const Interval* target_interval = nullptr;
    
    for (const auto& interval : current_path.possible_intervals) {
        if (interval.get_start() == info.jp2) {
            is_interval_start = true;
            target_interval = &interval;
            break;
        }
    }

    if (!is_interval_start) {
        // 直接使用找到的路径
        logger::log_info("Not an interval start, using found path directly");
        update_path_with_local_solution(node, info, new_local_path);
        return;
    }
    
    // If it's the interval start, check subsequent jump points
    if (target_interval && target_interval->jump_points.size() > 1) {
        size_t next_jp_index = info.jp2_jumps_index + 1;
        auto next_jp = node.solution[info.constraint.agent].top().jump_points[next_jp_index];
        
        if (has_better_solution(new_local_path, info.jp2, next_jp)) {
            logger::log_info("Found better path");
            auto next_jp_path_index = std::distance(path.begin(), 
                                                    std::find(path.begin(), path.end(), next_jp));
            // Recursively try to find a longer path
            auto new_info = ConstraintInfo(info.constraint,
                                          info.jp1_path_index,
                                          next_jp_path_index,
                                          info.constraint_path_index,
                                          info.jp1,
                                          next_jp,
                                          info.jp1_jumps_index,
                                          next_jp_index);
            logger::log_info("Attempting to recursively find better path");
            resolve_conflict_locally(node, new_info);
        } else {
            // Use current found path
            logger::log_info("No better path found, using current path");
            update_path_with_local_solution(node, info, new_local_path);
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
                                             const ConstraintInfo& info, 
                                             const std::vector<Vertex>& local_path) {
    if (local_path.empty()) {
        logger::log_error("local path is empty");
        return;
    }

    if (node.solution.find(info.constraint.agent) == node.solution.end()) {
        logger::log_error("can't find agent " + std::to_string(info.constraint.agent) + " solution");
        return;
    }

    auto& agent_paths = node.solution[info.constraint.agent];
    if (agent_paths.empty()) {
        logger::log_error("agent path queue is empty");
        return;
    }

    auto current_path = agent_paths.top();
    agent_paths.pop();
    
    // Build new path
    std::vector<Vertex> new_path;
    new_path.reserve(current_path.path.size() + local_path.size()); // 预分配空间
    
    // 1. 添加起点到start_index之间的路径
    new_path.insert(new_path.end(), 
                   current_path.path.begin(), 
                   current_path.path.begin() + info.jp1_path_index);
    
    // 2. 添加局部路径
    new_path.insert(new_path.end(), 
                   local_path.begin(), 
                   local_path.end());
    
    // 3. 添加end_index到终点的路径
    if (info.jp2_path_index + 1 < current_path.path.size()) {
        new_path.insert(new_path.end(), 
                       current_path.path.begin() + info.jp2_path_index + 1,
                       current_path.path.end());
    }

    logger::log_info("update_path_with_local_solution");
    logger::log_info("new_path: " + logger::vectorToString(new_path));
    

    // 一次性移除所有需要删除的区间
    current_path.possible_intervals.erase(
        std::remove_if(current_path.possible_intervals.begin(),
                      current_path.possible_intervals.end(),
                      [&](const Interval& interval) {
                          // 检查区间的起点是否在要删除的跳点范围内
                          auto start = interval.get_start();
                          for (size_t i = info.jp1_jumps_index; i < info.jp2_jumps_index; i++) {
                              if (start == current_path.jump_points[i]) {
                                  return true;
                              }
                          }
                          return false;
                      }),
        current_path.possible_intervals.end()
    );

    // Update jump points
    std::vector<Vertex> new_jump_points;
    new_jump_points.reserve(current_path.jump_points.size()); // 预分配空间
    
    // Keep jump points before start point
    new_jump_points.insert(new_jump_points.end(), 
                           current_path.jump_points.begin(), 
                           current_path.jump_points.begin() + info.jp1_jumps_index + 1);
    
    // Add jump points after end point
    new_jump_points.insert(new_jump_points.end(), 
                           current_path.jump_points.begin() + info.jp2_jumps_index, 
                           current_path.jump_points.end()); 
    
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
    logger::log_info("Total cost: " + std::to_string(std::round(node.cost * 1000) / 1000.0));
    logger::print_constraints(node.constraints);
    
    for (const auto& [agent_id, paths] : node.solution) {
        if (!paths.empty()) {
            std::string path_str = "Path for agent " + std::to_string(agent_id) + ": ";
            std::string jump_points_str = "Jump points: ";
            std::string possible_intervals_str = "Possible intervals: ";
            for (const auto& jp : paths.top().jump_points) {
                jump_points_str += "(" + std::to_string(jp.x) + "," + std::to_string(jp.y) + ") ";
            }
            for (const auto& pos : paths.top().path) {
                path_str += "(" + std::to_string(pos.x) + "," + std::to_string(pos.y) + ") ";
            }
            for (const auto& interval : paths.top().possible_intervals) {
                possible_intervals_str += "[" + std::to_string(interval.get_start().x) + "," + 
                    std::to_string(interval.get_start().y) + "," + 
                    std::to_string(interval.get_end().x) + "," + 
                    std::to_string(interval.get_end().y) + "] ";
            }
            logger::log_info(path_str);
            logger::log_info(jump_points_str);
            logger::log_info(possible_intervals_str);
        }
    }
    logger::log_info("------------------------");
}

int JPSCBS::count_conflicts(const JPSCBSNode& node) {
    int total_conflicts = 0;
    const auto& solution = node.solution;
    
    for (auto it1 = solution.begin(); it1 != solution.end(); ++it1) {
        if (it1->second.empty()) {
            logger::log_warning("agent " + std::to_string(it1->first) + " has no path");
            continue;
        }
        
        auto it2 = it1;
        ++it2;
        for (; it2 != solution.end(); ++it2) {
            if (it2->second.empty()) {
                logger::log_warning("agent " + std::to_string(it2->first) + " has no path");
                continue;
            }
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

std::vector<ConstraintInfo> JPSCBS::collect_constraint_infos(const JPSCBSNode& node, 
                                                           const std::vector<Constraint>& constraints) {
    std::vector<ConstraintInfo> constraint_infos;
    logger::log_info("Starting to generate constraint info");
    logger::print_constraints(constraints);

    for(const auto& constraint : constraints) {
        const auto& current_path = node.solution.at(constraint.agent).top();  // 改为const引用
        const auto& path = current_path.path;
        
        // 找到约束点在路径中的位置
        size_t constraint_idx = 0;
        for (; constraint_idx < path.size(); ++constraint_idx) {
            if (path[constraint_idx] == constraint.vertex) break;
        }
        if (constraint_idx == 0 || constraint_idx == path.size()) continue;

        // 遍历跳点对，找到包含约束点的区间
        for (size_t i = 0; i < current_path.jump_points.size() - 1; i++) {
            auto jp1 = current_path.jump_points[i];
            auto jp2 = current_path.jump_points[i + 1];
            
            auto jp1_pos = std::find(path.begin(), path.end(), jp1);
            auto jp2_pos = std::find(jp1_pos, path.end(), jp2);
            
            if (jp1_pos == path.end() || jp2_pos == path.end()) continue;

            size_t jp1_idx = std::distance(path.begin(), jp1_pos);
            size_t jp2_idx = std::distance(path.begin(), jp2_pos);

            if (jp1_idx < constraint_idx && constraint_idx <= jp2_idx) {
                constraint_infos.emplace_back(
                    constraint,
                    jp1_idx,
                    jp2_idx,
                    constraint_idx,
                    jp1,
                    jp2,
                    i,
                    i + 1
                );
                break;
            }
        }
    }

    for (size_t i = 0; i < constraint_infos.size(); i++)
    {
        logger::log_info("Constraint info: ");
        logger::log_info(constraint_infos[i].toString());
    }
    

    if (constraint_infos.empty()) {
        logger::log_info("collect_constraint_infos: No jump point interval containing constraint point found");
    }

    return constraint_infos;
}



