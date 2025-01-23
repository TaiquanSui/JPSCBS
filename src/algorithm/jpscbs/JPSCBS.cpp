#include "JPSCBS.h"

std::vector<std::vector<Vertex>> JPSCBS::solve(const std::vector<Agent>& agents, 
                                              const std::vector<std::vector<int>>& grid) {
    logger::log_info("Starting JPSCBS solver");
    start_time = std::chrono::steady_clock::now();
    this->grid = grid;
    
    // Initialize root node and solutions
    auto root = initialize(agents);
    print_node_info(*root, "初始节点");
    
    std::priority_queue<std::shared_ptr<JPSCBSNode>, 
                       std::vector<std::shared_ptr<JPSCBSNode>>, 
                       JPSCBSNodeComparator> open_list;
    open_list.push(root);
    
    
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
        
        // 验证和修复所有路径
        validate_and_repair_solutions(agents, *current);
        
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

        auto constraint_infos = collect_constraint_infos(*current, new_constraints);

        if(find_alt_symmetric_paths(*current, constraint_infos)) {
            open_list.push(current);
            continue;
        }   
        
        // 为每个约束创建新的节点
        for (const auto& info : constraint_infos) {
            logger::print_constraint(info.constraint, "处理约束");

            auto new_node = std::make_shared<JPSCBSNode>(*current);
            new_node->constraints.push_back(info.constraint);
            resolve_conflict_locally(*new_node, info);
            new_node->cost = calculate_sic(*new_node);
            open_list.push(new_node);
            print_node_info(*new_node, "新生成的子节点");
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
    logger::log_info("开始验证和修复路径");

    for (const auto& agent : agents) {
        auto& agent_paths = node.solution[agent.id];
        
        // 如果没有可用路径，直接返回失败
        if (agent_paths.empty()) {
            logger::log_warning("Agent " + std::to_string(agent.id) + " 没有可用路径");
            continue;
        }

        while (!agent_paths.empty()) {
            auto current_path = agent_paths.top();
            
            // 检查当前路径是否满足所有约束
            auto violated_constraints = utils::find_violated_constrains(node.constraints, current_path.path);
            
            if (violated_constraints.empty()) {
                // 如果路径有效，将其放回队列并继续下一个智能体
                agent_paths.push(std::move(current_path));
                break;
            } else {
                logger::log_info("Agent " + std::to_string(agent.id) + " 的路径违反约束，尝试修复");
                
                for (const auto& constraint : violated_constraints) {
                    // 使用A*寻找绕过约束的新路径
                    auto constraint_infos = collect_constraint_infos(node, {constraint});
                    if(find_alt_symmetric_paths(node, constraint_infos)) {
                        continue;
                    }

                    resolve_conflict_locally(node, constraint_infos[0]);
                    logger::print_constraint(constraint, "修复完constraint: ");
                    print_node_info(node, "修复完节点");
                }
                
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

bool JPSCBS::find_alt_symmetric_paths(JPSCBSNode& node, 
                                    const std::vector<ConstraintInfo>& constraint_infos) {
    int original_conflicts = count_conflicts(node);
    logger::log_info("原始节点的冲突数量: " + std::to_string(original_conflicts));

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

        logger::log_info("尝试找到local bypass");
        logger::log_info("agent_id: " + std::to_string(info.constraint.agent));
        logger::log_info("start_time: " + std::to_string(info.jp1_path_index));
        logger::log_info("end_time: " + std::to_string(info.jp2_path_index));
        logger::print_constraints(temp_constraints,"temp constrains");

        auto alt_path = a_star(info.constraint.agent, info.jp1, info.jp2,
                             grid, temp_constraints, 
                             info.jp1_path_index);

        if (!alt_path.empty()) {
            logger::log_info("找到绕行路径: " + logger::vectorToString(alt_path));
            
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
                logger::log_info("新路径的冲突数量: " + std::to_string(new_conflicts));

                if (new_conflicts < original_conflicts) {
                    node.cost = calculate_sic(node);
                    print_node_info(node, "找到更好的对称绕行路径");
                    return true;
                } else {
                    // 恢复原始路径
                    agent_paths.pop();
                    agent_paths.push(original_path);
                    logger::log_info("对称绕行路径未能减少冲突数量，放弃此路径");
                }
            }else{
                logger::log_info("绕行路径距离更远");
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
    // First try to find a path within the current interval
    logger::log_info("本地冲突解决");
    logger::log_info("agent_id: " + std::to_string(info.constraint.agent));
    logger::log_info("start_jp: " + std::to_string(info.jp1.x) + "," + std::to_string(info.jp1.y));
    logger::log_info("end_jp: " + std::to_string(info.jp2.x) + "," + std::to_string(info.jp2.y));
    logger::log_info("start_time: " + std::to_string(info.jp1_path_index));
    logger::log_info("end_time: " + std::to_string(info.jp2_path_index));
    logger::print_constraints(node.constraints, "约束");

    auto current_path = node.solution[info.constraint.agent].top();
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
        logger::log_info("不是interval start，直接使用找到的路径");
        update_path_with_local_solution(node, info, new_local_path);
        return;
    }
    
    // If it's the interval start, check subsequent jump points
    if (target_interval && target_interval->jump_points.size() > 1) {
        size_t next_jp_index = info.jp2_jumps_index + 1;
        auto next_jp = node.solution[info.constraint.agent].top().jump_points[next_jp_index];
        
        if (has_better_solution(new_local_path, info.jp2, next_jp)) {
            logger::log_info("有更好的路径");
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
            logger::log_info("尝试递归找到更好路径");
            resolve_conflict_locally(node, new_info);
        } else {
            // Use current found path
            logger::log_info("没有更好的路径，使用找到的路径");
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
    auto& agent_paths = node.solution[info.constraint.agent];
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
                           current_path.jump_points.begin() + info.jp1_jumps_index);
    
    // Add jump points after end point
    new_jump_points.insert(new_jump_points.end(), 
                           current_path.jump_points.begin() + info.jp2_jumps_index + 1, 
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
    logger::log_info("总代价: " + std::to_string(std::round(node.cost * 1000) / 1000.0));  // 保留3位小数
    logger::print_constraints(node.constraints);
    
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

std::vector<ConstraintInfo> JPSCBS::collect_constraint_infos(const JPSCBSNode& node, 
                                                           const std::vector<Constraint>& constraints) {
    std::vector<ConstraintInfo> constraint_infos;

    for(const auto& constraint : constraints) {
        auto& current_path = node.solution.at(constraint.agent).top();
        const auto& path = current_path.path;
        
        // 找到约束点在路径中的位置
        auto constraint_pos = std::find(path.begin(), path.end(), constraint.vertex);
        if (constraint_pos == path.end()) continue;

        size_t constraint_idx = std::distance(path.begin(), constraint_pos);

        // 遍历跳点对，找到包含约束点的区间
        for (size_t i = 0; i < current_path.jump_points.size() - 1; i++) {
            auto jp1 = current_path.jump_points[i];
            auto jp2 = current_path.jump_points[i + 1];
            
            auto jp1_pos = std::find(path.begin(), path.end(), jp1);
            auto jp2_pos = std::find(jp1_pos, path.end(), jp2);
            
            if (jp1_pos == path.end() || jp2_pos == path.end()) continue;

            size_t jp1_idx = std::distance(path.begin(), jp1_pos);
            size_t jp2_idx = std::distance(path.begin(), jp2_pos);

            if (jp1_idx <= constraint_idx && constraint_idx <= jp2_idx) {
                constraint_infos.emplace_back(
                    constraint,
                    jp1_idx,
                    jp2_idx,
                    constraint_idx,
                    jp1,
                    jp2,
                    i,
                    i + 1  // 添加 jp2_jumps_index
                );
                break;
            }
        }
    }

    return constraint_infos;
}



