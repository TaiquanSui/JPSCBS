#include "JPSCBS.h"

std::vector<std::vector<Vertex>> JPSCBS::solve(const std::vector<Agent>& agents, 
                                              const std::vector<std::vector<int>>& grid) {
    //logger::log_info("Starting JPSCBS solver");
    
    this->grid = grid;
    this->solutions.clear();        // 清空之前的解决方案
    this->agent_states.clear();     // 清空之前的智能体状态
    expanded_nodes = 0;             // 重置节点计数器
    interrupted = false;            // 重置中断状态
    
    // 记录开始时间
    auto start_time = std::chrono::steady_clock::now();
    
    // Initialize root node and solutions
    auto root = initialize(agents);
    if(root == nullptr) {
        //logger::log_info("No solution");
        return {};
    }
    
    std::priority_queue<std::shared_ptr<JPSCBSNode>, 
                       std::vector<std::shared_ptr<JPSCBSNode>>, 
                       JPSCBSNodeComparator> open_list;
    open_list.push(root);
    
    while (!open_list.empty()) {

        auto current = open_list.top();
        if (!current) {
            //logger::log_error("遇到空节点，跳过处理");
            open_list.pop();
            continue;
        }
        open_list.pop();
        expanded_nodes++;
        
        print_node_info(*current, "Current Node");
        
        auto new_constraints = generate_constraints(*current);
        
        if (new_constraints.empty()) {
            std::vector<std::vector<Vertex>> final_paths;
            final_paths.reserve(agents.size());
            logger::log_info("JPSCBS found solution with cost: " + std::to_string(current->cost));
            for (const auto& agent : agents) {
                final_paths.push_back(current->solution[agent.id].top().path);
                logger::log_info("path of agent" + std::to_string(agent.id) + " :" + logger::vectorToString(current->solution[agent.id].top().path));
            }
            logger::log_info("expanded nodes:" + std::to_string(expanded_nodes));
            return final_paths;
        }

        if (should_terminate()) {
            //logger::log_info("JPSCBS solver interrupted");
            return {};
        }

        logger::print_constraints(new_constraints,"generated constrains");
        auto constraint_infos = collect_constraint_infos(*current, new_constraints);

        auto bypass_result = find_bypass(*current, constraint_infos);
        if (bypass_result.success) {
            open_list.push(current);
            continue;
        }   

        // generate new node for each constraint
        for (const auto& bypass_path : bypass_result.bypass_paths) {
            if (should_terminate()) {
                //logger::log_info("JPSCBS solver interrupted");
                return {};
            }

            //logger::log_info("Processing constraint, constraint info:");
            //logger::log_info(bypass_path.first.toString());
            
            if(bypass_path.second.empty()) {
                //logger::log_info("Bypass path is empty, skipping");
                continue;
            }

            auto new_node = std::make_shared<JPSCBSNode>(*current);
            new_node->constraints.push_back(bypass_path.first.constraint);

            if(!resolve_conflict_locally(*new_node, bypass_path.first, bypass_path.second)) {
                //logger::log_info("Failed to resolve conflict, skipping");
                continue;
            }

            update_solutions(agents, *new_node);
            validate_and_repair_solutions(agents, *new_node);
            // print_node_info(*new_node, "New Node");
            
            open_list.push(new_node);
            // print_node_info(*new_node, "New Child Node Generated");
        }
    }
    
    //logger::log_info("JPSCBS found no solution");
    return {};  // No solution
}

std::shared_ptr<JPSCBSNode> JPSCBS::initialize(const std::vector<Agent>& agents) {
    auto root = std::make_shared<JPSCBSNode>();
    //logger::log_info("Initializing JPSCBS");
    
    // 清空状态
    agent_states.clear();
    
    for (const auto& agent : agents) {
        // 直接使用operator[]会自动创建JPSState
        auto start_node = std::make_shared<AStarNode>(
            agent.start, 0, heuristic(agent.start, agent.goal));
        agent_states[agent.id].open_list.push(start_node);
        
        // 寻找初始路径
        JPSPath path = search_by_jps(agent);
        if (path.path.empty()) {
            //logger::log_info("No initial path found for agent " + std::to_string(agent.id));
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
            //logger::log_warning("can't find agent " + std::to_string(agent_id) + " state");
            continue;
        }
        // 如果该智能体的open_list为空，说明无法找到更多路径，跳过更新
        if (agent_states[agent_id].open_list.empty()) {
            //logger::log_warning("agent " + std::to_string(agent_id) + " has no more path");
            continue;
        }
        
        double node_solutions_cost = utils::calculate_path_cost(agent_paths.top().path);
        double solutions_cost = utils::calculate_path_cost(solutions[agent_id].back().path);
        // Check if we need to continue searching for a new path
        while (!agent_paths.empty() && !solutions[agent_id].empty() && 
               node_solutions_cost >= solutions_cost) {
            
            // Continue searching for a new path
            //logger::log_info("continue searching for a new path, agent_id: " + std::to_string(agent_id));
            JPSPath new_path = search_by_jps(agents[agent_id]);
            
            if (!new_path.path.empty()) {
                solutions_cost = utils::calculate_path_cost(new_path.path);
                solutions[agent_id].push_back(new_path);   
            } else {
                // 如果找不到新路径，说明搜索已完成，清空open_list表示不需要继续搜索
                //logger::log_info("no more path, clear open_list");
                agent_states[agent_id].clear();
                break;
            }
        }
        
        // 确保i不会超出solutions的范围
        size_t start_idx = std::min(agent_paths.size(), solutions[agent_id].size());
        
        // Add all paths with lower cost to current node's solution
        for (size_t i = start_idx; i < solutions[agent_id].size(); i++) {
            JPSPath path = solutions[agent_id][i];  // 创建一个深拷贝
            if (utils::calculate_path_cost(path.path) <= utils::calculate_path_cost(agent_paths.top().path)) {
                agent_paths.push(std::move(path));  // 使用移动语义避免额外拷贝
            }
        }
    }
}

JPSPath JPSCBS::search_by_jps(const Agent& agent) {
    // 检查状态是否已经正确初始化（包含起始节点等信息）
    auto state_it = agent_states.find(agent.id);
    if (state_it == agent_states.end() || state_it->second.open_list.empty()) {
        //logger::log_error("Agent state not properly initialized: " + std::to_string(agent.id));
        return JPSPath({}, {}, {});
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
    //logger::log_info("Starting to validate and repair paths");

    for (const auto& agent : agents) {
        if (node.solution.find(agent.id) == node.solution.end()) {
            //logger::log_error("can't find agent " + std::to_string(agent.id) + " solution");
            continue;
        }
        
        auto& agent_paths = node.solution[agent.id];
        
        // 如果没有可用路径，直接返回失败
        if (agent_paths.empty()) {
            //logger::log_warning("Agent " + std::to_string(agent.id) + " has no available paths");
            continue;
        }

        while (!agent_paths.empty()) {
            const auto& current_path = agent_paths.top();
            
            // 检查当前路径是否满足所有约束
            auto violated_constraints = utils::find_violated_constrains(agent.id, current_path.path, node.constraints);
            
            if (violated_constraints.empty()) {
                //logger::log_info("Agent " + std::to_string(agent.id) + " path is valid");
                break;
            } 

            // //logger::log_info("Agent " + std::to_string(agent.id) + " path violates constraints, attempting repair");
            
            for (const auto& constraint : violated_constraints) {
                auto constraint_infos = collect_constraint_infos(node, {constraint});

                if (constraint_infos.empty()) {
                    //logger::log_info("Repair found constraint is problematic");
                    continue;
                }

                auto bypass_result = find_bypass(node, constraint_infos);
                if(bypass_result.success) {
                    continue;
                }
                resolve_conflict_locally(node, bypass_result.bypass_paths[0].first, bypass_result.bypass_paths[0].second);
                // //logger::print_constraint(constraint, "Repaired constraint: ");
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
            // 如果agent1已经到达终点，跳过
            if (i >= paths1.top().path.size()) continue;
            
            for (const auto& [agent2, paths2] : node.solution) {
                if (agent1 >= agent2) continue;
                // 如果agent2已经到达终点，跳过
                if (i >= paths2.top().path.size()) continue;

                const auto& path1 = paths1.top().path;
                const auto& path2 = paths2.top().path;

                Vertex pos1 = path1[i];
                Vertex pos2 = path2[i];

                if (pos1 == pos2) {
                    constraints.emplace_back(agent1, pos1, i);  
                    constraints.emplace_back(agent2, pos2, i);
                    return constraints;
                }

                if (i < t - 1 && i + 1 < path1.size() && i + 1 < path2.size()) {
                    Vertex next_pos1 = path1[i + 1];
                    Vertex next_pos2 = path2[i + 1];

                    // check swapping conflict
                    if (pos1 == next_pos2 && pos2 == next_pos1) {
                        constraints.emplace_back(agent1, next_pos1, i + 1);
                        constraints.emplace_back(agent2, next_pos2, i + 1);
                        return constraints;
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
    
    //logger::log_info("No new constraints found");
    return {};
}

BypassResult JPSCBS::find_bypass(JPSCBSNode& node, 
                                const std::vector<ConstraintInfo>& constraint_infos) {
    int original_conflicts = count_conflicts(node);
    //logger::log_info("Original conflict count: " + std::to_string(original_conflicts));

    std::vector<std::pair<ConstraintInfo, std::vector<Vertex>>> bypass_paths;

    for(const auto& info : constraint_infos) {
        auto& agent_paths = node.solution[info.constraint.agent];
        const auto& current_path = agent_paths.top();

        // Add temporary constraint
        std::vector<Constraint> temp_constraints = node.constraints;
        temp_constraints.push_back(info.constraint);

        //logger::log_info("Attempting to find local bypass");
        //logger::log_info("agent_id: " + std::to_string(info.constraint.agent));
        //logger::log_info("start_time: " + std::to_string(info.jp1_path_index));
        //logger::log_info("end_time: " + std::to_string(info.jp2_path_index));
        //logger::print_constraints(temp_constraints, "Temporary constraints");

        // 计算CAT，排除当前智能体
        ConflictAvoidanceTable cat = calculate_cat({info.constraint.agent}, node);

        auto alt_path = a_star(info.constraint.agent, info.jp1, info.jp2,
                             grid, temp_constraints, info.jp1_path_index, cat);

        if (!alt_path.empty()) {
            logger::log_info("Found bypass path: " + logger::vectorToString(alt_path));
            bypass_paths.emplace_back(info, alt_path);
            
            std::vector<Vertex> original_segment(
                current_path.path.begin() + info.jp1_path_index,
                current_path.path.begin() + info.jp2_path_index + 1
            );
            logger::log_info("original segment: " + logger::vectorToString(original_segment));
            double original_cost = utils::calculate_path_cost(original_segment);
            double alt_cost = utils::calculate_path_cost(alt_path);
            logger::log_info("bypass path cost: " + std::to_string(alt_cost));
            logger::log_info("original segment: " + std::to_string(original_cost));

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

                JPSPath updated_path = current_path;
                updated_path.path = std::move(new_path);

                int new_conflicts = count_conflicts(node);
                logger::log_info("New conflict count: " + std::to_string(new_conflicts));

                if (new_conflicts < original_conflicts) {
                    agent_paths.pop();
                    agent_paths.push(std::move(updated_path));
                    node.cost = calculate_sic(node);
                    // print_node_info(node, "Found better symmetric bypass path");
                    return {true, {}};
                } else {
                    logger::log_info("Bypass path did not reduce conflicts, keeping path for later use");
                }
            }else{
                logger::log_info("Longer path, for later use.");
            }
        }
    }

    return {false, bypass_paths};
}

bool JPSCBS::resolve_conflict_locally(JPSCBSNode& node, 
                                    const ConstraintInfo& info,
                                    const std::vector<Vertex>& bypass_paths) {
    // 原有的resolve_conflict_locally逻辑
    if (node.solution.find(info.constraint.agent) == node.solution.end()) {
        //logger::log_error("can't find agent " + std::to_string(info.constraint.agent) + " solution");
        return false;
    }

    auto& agent_paths = node.solution[info.constraint.agent];
    if (agent_paths.empty()) {
        //logger::log_error("agent " + std::to_string(info.constraint.agent) + " has no path");
        return false;
    }

    const auto& current_path = agent_paths.top();
    if (info.jp1_path_index >= current_path.path.size() || 
        info.jp2_path_index >= current_path.path.size()) {
        //logger::log_error("path index out of range");
        return false;
    }

    // First try to find a path within the current interval
    //logger::log_info("Local conflict resolution");
    //logger::log_info("agent_id: " + std::to_string(info.constraint.agent));
    //logger::log_info("start_jp: " + std::to_string(info.jp1.x) + "," + std::to_string(info.jp1.y));
    //logger::log_info("end_jp: " + std::to_string(info.jp2.x) + "," + std::to_string(info.jp2.y));
    //logger::log_info("start_time: " + std::to_string(info.jp1_path_index));
    //logger::log_info("end_time: " + std::to_string(info.jp2_path_index));
    //logger::print_constraints(node.constraints, "Constraints");
    auto& path = current_path.path;
    
    // 使用索引获取对应的迭代器
    auto jp1_it = path.begin() + info.jp1_path_index;
    auto jp2_it = path.begin() + info.jp2_path_index;
    
    if (info.jp2_jumps_index + 1 < current_path.jump_points.size()) {
        size_t next_jp_index = info.jp2_jumps_index + 1;
        auto next_jp = current_path.jump_points[next_jp_index];  // 使用current_path而不是node中的路径
            
        if (has_better_solution(bypass_paths, info.jp2, next_jp)) {
            //logger::log_info("Could find better path");
            auto next_jp_path_index = std::distance(path.begin(), 
                                                    std::find(path.begin(), path.end(), next_jp));
            // Recursively try
            auto new_info = ConstraintInfo(info.constraint,
                                          info.jp1_path_index,
                                          next_jp_path_index,
                                          info.constraint_path_index,
                                          info.jp1,
                                          next_jp,
                                          info.jp1_jumps_index,
                                          next_jp_index);
            logger::log_info("Attempting to recursively find better path");
            ConflictAvoidanceTable cat = calculate_cat({info.constraint.agent}, node);
            auto new_path = a_star(new_info.constraint.agent, new_info.jp1, new_info.jp2,
                             grid, node.constraints, new_info.jp1_path_index, cat);
            return resolve_conflict_locally(node, new_info, new_path);
        } else {
            // Use bypass path directly
            logger::log_info("No better path found, using bypass path");
            update_path_with_local_solution(node, info, bypass_paths);
            return true;
        }
    }else{
        update_path_with_local_solution(node, info, bypass_paths);
    }
    return true;
}



bool JPSCBS::has_better_solution(const std::vector<Vertex>& new_path, 
                               const Vertex& jp2, 
                               const Vertex& next_jp) {
    Vertex direction1 = utils::calculateDirection(next_jp, jp2);
    Vertex direction2 = utils::calculateDirection(jp2, next_jp);
    
    // 如果是水平或垂直线，不需要检查
    if (direction1.x == 0 || direction1.y == 0) {
        return false;
    }
    
    // direction1的x和y分量只可能是1或-1
    int dx = direction1.x;  // 1或-1
    int dy = direction1.y;  // 1或-1
    
    if (dx > 0) {  // 向右的情况
        for (const auto& point : new_path) {
            int distance = point.y - jp2.y;
            if (point.x >= jp2.x + dx * distance) {
                return true;
            }
        }
    } else {  // 向左的情况
        for (const auto& point : new_path) {
            int distance = point.y - jp2.y;
            if (point.x <= jp2.x + dx * distance) {
                return true;
            }
        }
    }
    return false;
}

// Helper function: update path
void JPSCBS::update_path_with_local_solution(JPSCBSNode& node, 
                                             const ConstraintInfo& info, 
                                             const std::vector<Vertex>& local_path) {
    if (local_path.empty()) {
        //logger::log_error("local path is empty");
        return;
    }

    if (node.solution.find(info.constraint.agent) == node.solution.end()) {
        //logger::log_error("can't find agent " + std::to_string(info.constraint.agent) + " solution");
        return;
    }

    auto& agent_paths = node.solution[info.constraint.agent];
    if (agent_paths.empty()) {
        //logger::log_error("agent path queue is empty");
        return;
    }

    const auto& current_path = agent_paths.top();
    
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

    //logger::log_info("update_path_with_local_solution");
    //logger::log_info("new_path: " + logger::vectorToString(new_path));

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
    
    // 创建更新后的路径，使用std::move避免复制
    JPSPath updated_path = std::move(current_path);
    updated_path.path = std::move(new_path);
    updated_path.jump_points = std::move(new_jump_points);
    //logger::log_info("updated_path: " + logger::vectorToString(updated_path.path));

    // 打印更新前的所有路径
    // logger::log_info("Before update - All paths in priority queue:");
    // std::priority_queue<JPSPath, std::vector<JPSPath>, JPSPathComparator> temp_queue = agent_paths;
    // int path_count = 0;
    // while (!temp_queue.empty()) {
    //     logger::log_info("Path " + std::to_string(path_count) + ": " + 
    //                     logger::vectorToString(temp_queue.top().path));
    //     temp_queue.pop();
    //     path_count++;
    // }

    agent_paths.pop();
    agent_paths.push(std::move(updated_path));
    node.cost = calculate_sic(node);

    // 打印更新后的所有路径
    // logger::log_info("After update - All paths in priority queue:");
    // temp_queue = agent_paths; // 重新创建临时队列
    // path_count = 0;
    // while (!temp_queue.empty()) {
    //     logger::log_info("Path " + std::to_string(path_count) + ": " + 
    //                     logger::vectorToString(temp_queue.top().path));
    //     temp_queue.pop();
    //     path_count++;
    // }
}

void JPSCBS::print_node_info(const JPSCBSNode& node, const std::string& prefix) {
    if (!prefix.empty()) {
        logger::log_info(prefix + ":");
    }
    logger::log_info("Total cost: " + std::to_string(std::round(node.cost * 1000) / 1000.0));
    logger::print_constraints(node.constraints, "Constraints");
    
    // 打印当前节点中每个智能体的跳点
    for (const auto& [agent_id, paths] : node.solution) {
        if (!paths.empty()) {
            std::string jump_points_str = "Agent " + std::to_string(agent_id) + " Current JPs: ";
            for (const auto& jp : paths.top().jump_points) {
                jump_points_str += "(" + std::to_string(jp.x) + "," + std::to_string(jp.y) + ") ";
            }
            logger::log_info("Agent " + std::to_string(agent_id) + " path: " + logger::vectorToString(paths.top().path));
            // logger::log_info(jump_points_str);
        }
    }

    // 打印solutions中存储的所有路径信息
    // logger::log_info("All stored solutions:");
    // for (const auto& [agent_id, agent_paths] : solutions) {
    //     logger::log_info("Agent " + std::to_string(agent_id) + " stored paths:");
    //     for (size_t i = 0; i < agent_paths.size(); ++i) {
    //         const auto& path = agent_paths[i];
    //         double path_cost = utils::calculate_path_cost(path.path);
            
    //         std::string path_info = "  Path " + std::to_string(i) + 
    //                               " (cost: " + std::to_string(std::round(path_cost * 1000) / 1000.0) + 
    //                               ") JPs: ";
            
    //         for (const auto& jp : path.jump_points) {
    //             path_info += "(" + std::to_string(jp.x) + "," + std::to_string(jp.y) + ") ";
    //         }
    //         logger::log_info(path_info);
    //     }
    // }
        
    logger::log_info("------------------------");
}

int JPSCBS::count_conflicts(const JPSCBSNode& node) {
    int total_conflicts = 0;
    const auto& solution = node.solution;
    
    for (auto it1 = solution.begin(); it1 != solution.end(); ++it1) {
        if (it1->second.empty()) {
            //logger::log_warning("agent " + std::to_string(it1->first) + " has no path");
            continue;
        }
        
        auto it2 = it1;
        ++it2;
        for (; it2 != solution.end(); ++it2) {
            if (it2->second.empty()) {
                //logger::log_warning("agent " + std::to_string(it2->first) + " has no path");
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
    //logger::log_info("Starting to generate constraint info");
    //logger::print_constraints(constraints);

    for(const auto& constraint : constraints) {
        if (node.solution.find(constraint.agent) == node.solution.end() || 
            node.solution.at(constraint.agent).empty()) {
            //logger::log_warning("Agent " + std::to_string(constraint.agent) + " has no path");
            continue;
        }

        if(constraint.time == 0) {
            continue;
        }

        const auto& current_path = node.solution.at(constraint.agent).top();
        const auto& path = current_path.path;
        const auto& jump_points = current_path.jump_points;
        
        // 找到约束点在路径中的位置
        auto constraint_pos = std::find(path.begin(), path.end(), constraint.vertex);
        if (constraint_pos == path.end()) {
            //logger::log_warning("Constraint vertex not found in path for agent " + 
            //                     std::to_string(constraint.agent));
            continue;
        }
        
        size_t constraint_idx = std::distance(path.begin(), constraint_pos);
        
        // 遍历跳点对，找到包含约束点的区间
        for (size_t i = 0; i < jump_points.size() - 1; i++) {
            auto jp1 = jump_points[i];
            auto jp2 = jump_points[i + 1];
            
            auto jp1_pos = std::find(path.begin(), path.end(), jp1);
            auto jp2_pos = std::find(jp1_pos, path.end(), jp2);
            
            if (jp1_pos == path.end() || jp2_pos == path.end()) {
                //logger::log_warning("Jump point not found in path");
                continue;
            }

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
                //logger::log_info("Found constraint info for agent " + 
                //                 std::to_string(constraint.agent));
                break;
            }
        }
    }

    for (const auto& info : constraint_infos) {
        //logger::log_info("Constraint info: ");
        //logger::log_info(info.toString());
    }

    if (constraint_infos.empty()) {
        //logger::log_warning("No constraint infos generated");
    }

    //logger::log_info("Collected constraint infos done");

    return constraint_infos;
}

ConflictAvoidanceTable JPSCBS::calculate_cat(const std::unordered_set<int>& excluded_agents, const JPSCBSNode& node) const {
    ConflictAvoidanceTable cat;
    
    // 遍历所有智能体的解决方案
    for (const auto& [agent_id, paths] : node.solution) {
        if (excluded_agents.count(agent_id) > 0) continue;  // 跳过被排除的智能体
        
        if (!paths.empty()) {
            const auto& path = paths.top().path;
            // 对路径中的每个位置添加约束
            for (size_t t = 0; t < path.size(); ++t) {
                cat.addConstraint(path[t], t);
                
                // 如果是路径的最后一个位置，添加终点约束
                if (t == path.size() - 1) {
                    cat.addGoalConstraint(path[t], t);
                }
            }
        }
    }
    
    return cat;
}
