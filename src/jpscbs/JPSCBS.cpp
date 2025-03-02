#include "JPSCBS.h"
#include "../heuristic/Heuristic.h"
#include "../utilities/GridUtility.h"
#include "../utilities/ConstraintUtility.h"
#include "../utilities/Log.h"

std::vector<std::vector<Vertex>> JPSCBS::solve(const std::vector<Agent>& agents, 
                                               const std::vector<std::vector<int>>& grid) {
    ////logger::log_info("Starting JPSCBS solver");
    
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
        ////logger::log_info("No solution");
        return {};
    }
    
    std::priority_queue<std::shared_ptr<JPSCBSNode>, 
                       std::vector<std::shared_ptr<JPSCBSNode>>, 
                       JPSCBSNodeComparator> open_list;
    open_list.push(root);
    
    while (!open_list.empty()) {
        auto current = open_list.top();
        if (!current) {
            ////logger::log_error("遇到空节点，跳过处理");
            open_list.pop();
            continue;
        }
        open_list.pop();
        expanded_nodes++;
        
        print_node_info(*current, "Current Node");
        ////logger::print_constraints(current->constraints, "Current Node constraints");

        JPSPathComparator::set_solution_context(&current->solution);
        
        auto new_constraints = generate_constraints(*current);
        
        if (new_constraints.empty()) {
            std::vector<std::vector<Vertex>> final_paths;
            final_paths.reserve(agents.size());
            //logger::log_info("JPSCBS found solution with cost: " + std::to_string(current->cost));
            for (const auto& agent : agents) {
                final_paths.push_back(current->solution[agent.id].top().path);
                //logger::log_info("path of agent" + std::to_string(agent.id) + " :" + //logger::vectorToString(current->solution[agent.id].top().path));
            }
            //logger::log_info("expanded nodes:" + std::to_string(expanded_nodes));
            return final_paths;
        }

        if (should_terminate()) {
            ////logger::log_info("JPSCBS solver interrupted");
            return {};
        }

        //logger::print_constraints(new_constraints,"generated constrains");
        auto constraint_infos = collect_constraint_infos(*current, new_constraints);

        auto bypass_result = find_bypass(*current, constraint_infos);
        if (bypass_result.success) {
            open_list.push(current);
            continue;
        }   

        // generate new node for each constraint
        for (const auto& bypass_path : bypass_result.bypass_paths) {
            if (should_terminate()) {
                ////logger::log_info("JPSCBS solver interrupted");
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

            JPSPathComparator::set_solution_context(&new_node->solution);

            if(!resolve_conflict_locally(*new_node, bypass_path.first, bypass_path.second)) {
                //logger::log_info("Failed to resolve conflict, skipping");
                continue;
            }
            
            open_list.push(new_node);
            // print_node_info(*new_node, "New Child Node Generated");
        }
    }
    
    ////logger::log_info("JPSCBS found no solution");
    return {};  // No solution
}

std::shared_ptr<JPSCBSNode> JPSCBS::initialize(const std::vector<Agent>& agents) {
    auto root = std::make_shared<JPSCBSNode>();
    ////logger::log_info("Initializing JPSCBS");
    
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
            ////logger::log_info("No initial path found for agent " + std::to_string(agent.id));
            return nullptr;
        }
        
        solutions[agent.id].push_back(path);  // 存储原始路径
        
        // 创建新的优先队列并使用path的深拷贝
        std::priority_queue<JPSPath, std::vector<JPSPath>, JPSPathComparator> paths;
        paths.push(std::move(path));  // 直接使用path
        
        root->solution[agent.id] = std::move(paths);  // 移动优先队列的所有权到root节点
    }

    root->cost = calculate_sic(*root);
    return root;
}

void JPSCBS::update_solutions(const std::vector<Agent>& agents, JPSCBSNode& node) {
    //logger::log_info("Updating solutions");
    for (auto& [agent_id, agent_paths] : node.solution) {
        
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
                solutions[agent_id].push_back(std::move(new_path));  
            } else {
                // 如果找不到新路径，说明搜索已完成，清空open_list表示不需要继续搜索
                //logger::log_info("no more path, clear open_list");
                agent_states[agent_id].clear();
                break;
            }
        }

        size_t start_idx = std::min(agent_paths.size(), solutions[agent_id].size());
        // Add all paths with lower cost to current node's solution
        for (size_t i = start_idx; i < solutions[agent_id].size(); i++) {
            JPSPath& path = solutions[agent_id][i];
            if (utils::calculate_path_cost(path.path) <= utils::calculate_path_cost(agent_paths.top().path)) {
                agent_paths.push(path);
            }
        }
    }
}

JPSPath JPSCBS::search_by_jps(const Agent& agent) {
    // 检查状态是否已经正确初始化（包含起始节点等信息）
    auto state_it = agent_states.find(agent.id);
    if (state_it == agent_states.end() || state_it->second.open_list.empty()) {
        ////logger::log_error("Agent state not properly initialized: " + std::to_string(agent.id));
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
            ////logger::log_error("can't find agent " + std::to_string(agent.id) + " solution");
            continue;
        }
        
        auto& agent_paths = node.solution[agent.id];
        
        // 如果没有可用路径，直接返回失败
        if (agent_paths.empty()) {
            ////logger::log_warning("Agent " + std::to_string(agent.id) + " has no available paths");
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

            //logger::log_info("Agent " + std::to_string(agent.id) + " path violates constraints, attempting repair");
            //logger::log_info("Path: " + //logger::vectorToString(current_path.path));
            
            for (const auto& constraint : violated_constraints) {
                auto constraint_infos = collect_constraint_infos(node, {constraint});

                if (constraint_infos.empty()) {
                    ////logger::log_info("Repair found constraint is problematic");
                    continue;
                }

                auto bypass_result = find_bypass(node, constraint_infos);
                if(bypass_result.success) {
                    continue;
                }
                resolve_conflict_locally(node, bypass_result.bypass_paths[0].first, bypass_result.bypass_paths[0].second);
                // ////logger::print_constraint(constraint, "Repaired constraint: ");
            }
                
        }

    }

}


std::vector<Constraint> JPSCBS::generate_constraints(const JPSCBSNode& node) {
    return utils::generate_constraints(node);
}



JPSBypassResult JPSCBS::find_bypass(JPSCBSNode& node,
                                const std::vector<ConstraintInfo>& constraint_infos) {
    int original_conflicts = count_conflicts(node);
    
    std::vector<std::pair<ConstraintInfo, std::vector<Vertex>>> bypass_paths;

    // 预先创建临时约束vector以重用其容量
    std::vector<Constraint> temp_constraints;
    temp_constraints.reserve(node.constraints.size() + 1);

    for(const auto& info : constraint_infos) {
        auto& agent_paths = node.solution[info.constraint.agent];
        JPSPath original_path = agent_paths.top();

        // 重用temp_constraints
        temp_constraints = node.constraints;
        temp_constraints.push_back(info.constraint);

        // 计算CAT，排除当前智能体
        ConflictAvoidanceTable cat = calculate_cat({info.constraint.agent}, node);

        auto bypass_result = find_alt_path(info, original_path, temp_constraints, cat);
        //logger::log_info("bypass found: " + //logger::vectorToString(bypass_result.second));
        
        if (!bypass_result.second.empty()) {
            // 计算当前路径段的代价
            std::vector<Vertex> original_segment(
                original_path.path.begin() + bypass_result.first.jp1_path_index,
                original_path.path.begin() + bypass_result.first.jp2_path_index + 1
            );
            double original_cost = utils::calculate_path_cost(original_segment);
            double alt_cost = utils::calculate_path_cost(bypass_result.second);

            if (std::abs(alt_cost - original_cost) < 1e-6) {
                update_path_with_local_solution(node, bypass_result.first, bypass_result.second);
                int new_conflicts = count_conflicts(node);
                if (new_conflicts < original_conflicts) {
                    node.cost = calculate_sic(node);
                    return {true, {}};
                } else {
                    // 如果没有减少冲突,恢复原来的路径
                    //logger::log_info("Conflict not reduced, restore original path");
                    agent_paths.pop();
                    agent_paths.push(original_path);
                    // 只有在没有找到好的解决方案时才添加到bypass_paths
                    bypass_paths.emplace_back(bypass_result);
                }
            } else {
                // 代价不相等时添加到bypass_paths
                //logger::log_info("Longer bypass path found, for later use");
                bypass_paths.emplace_back(bypass_result);
            }
        }
    }

    return {false, bypass_paths};
}

bool JPSCBS::resolve_conflict_locally(JPSCBSNode& node, 
                                    const ConstraintInfo& info,
                                    const std::vector<Vertex>& local_path) {
    if (node.solution.find(info.constraint.agent) == node.solution.end()) {
        return false;
    }

    auto& agent_paths = node.solution[info.constraint.agent];
    if (agent_paths.empty()) {
        return false;
    }

    // 直接更新路径
    update_path_with_local_solution(node, info, local_path);
    node.cost = calculate_sic(node);
    return true;
}



bool JPSCBS::has_better_solution(const std::vector<Vertex>& new_path,
                               const ConstraintInfo& info,
                               const JPSPath& current_path) {
    // 获取下一个跳点
    size_t next_jp_index = info.jp2_jumps_index + 1;
    if (next_jp_index >= current_path.jump_points.size()) {
        return false;
    }

    const Vertex& jp1 = info.jp1;
    const Vertex& jp2 = info.jp2;
    const Vertex& next_jp = current_path.jump_points[next_jp_index];
    
    // 确定next_jp_decide
    Vertex next_jp_decide = next_jp;
    for (const auto& interval : current_path.possible_intervals) {
        if (!interval.jump_points.empty() && interval.get_start() == jp2) {
            next_jp_decide = interval.jump_points[1];
            break;
        }
    }

    return is_path_crossing_line(new_path, jp1, jp2, next_jp_decide);
}

bool JPSCBS::is_path_crossing_line(const std::vector<Vertex>& path,
                                 const Vertex& jp1,
                                 const Vertex& jp2,
                                 const Vertex& next_jp_decide) const {
    int slope = (next_jp_decide.y - jp2.y) / (next_jp_decide.x - jp2.x);
    //logger::log_info("slope: " + std::to_string(slope));
    
    for (auto it = path.rbegin(); it != path.rend(); ++it) {
        // 跳过jp2点
        if (*it == jp2) continue;
        
        double relative_y = it->y - jp2.y;
        double relative_x = it->x - jp2.x;
        double line_y = slope * relative_x;
        
        if (jp1.y - jp2.y < slope * (jp1.x - jp2.x)) {
            if (relative_y >= line_y) {
                return true;
            }
        } else {
            if (relative_y <= line_y) {
                return true;
            }
        }
    }
    return false;
}


std::pair<ConstraintInfo, std::vector<Vertex>> JPSCBS::find_alt_path(
    const ConstraintInfo& info,
    const JPSPath& current_path,
    const std::vector<Constraint>& temp_constraints,
    const ConflictAvoidanceTable& cat) {
    
    auto alt_path = a_star(info.constraint.agent, info.jp1, info.jp2,
                          grid, temp_constraints, info.jp1_path_index, cat);
    
    if (alt_path.empty()) {
        return {info, {}};
    }

    if (!has_better_solution(alt_path, info, current_path)) {
        return {info, alt_path};
    }

    //logger::log_info("Better solution found, discard jump point: " + 
                     //std::to_string(current_path.jump_points[info.jp2_jumps_index].x) + "," + 
                     //std::to_string(current_path.jump_points[info.jp2_jumps_index].y));

    if (info.jp2_jumps_index + 1 < current_path.jump_points.size()) {
        size_t next_jp_index = info.jp2_jumps_index + 1;
        auto next_jp = current_path.jump_points[next_jp_index];
        auto next_jp_decide = next_jp;

        // 检查jp2是否是某个区间的起始点
        for (const auto& interval : current_path.possible_intervals) {
            if (!interval.jump_points.empty() && interval.get_start() == info.jp2) {
                next_jp_decide = interval.jump_points[1];
                break;
            }
        }

        auto next_jp_path_index = std::distance(current_path.path.begin(),
            std::find(current_path.path.begin(), current_path.path.end(), next_jp));

        // 创建新的约束信息
        auto new_info = ConstraintInfo(info.constraint,
            info.jp1_path_index,
            next_jp_path_index,
            info.constraint_path_index,
            info.jp1,
            next_jp,
            info.jp1_jumps_index,
            next_jp_index);

        return find_alt_path(new_info, current_path, temp_constraints, cat);
    }

    return {info, alt_path};
}

// Helper function: update path
void JPSCBS::update_path_with_local_solution(JPSCBSNode& node, 
                                             const ConstraintInfo& info, 
                                             const std::vector<Vertex>& local_path) {
    if (local_path.empty()) {
        ////logger::log_error("local path is empty");
        return;
    }

    if (node.solution.find(info.constraint.agent) == node.solution.end()) {
        ////logger::log_error("can't find agent " + std::to_string(info.constraint.agent) + " solution");
        return;
    }

    auto& agent_paths = node.solution[info.constraint.agent];
    if (agent_paths.empty()) {
        ////logger::log_error("agent path queue is empty");
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

    ////logger::log_info("update_path_with_local_solution");
    ////logger::log_info("new_path: " + //logger::vectorToString(new_path));

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
    ////logger::log_info("updated_path: " + //logger::vectorToString(updated_path.path));

    agent_paths.pop();
    agent_paths.push(std::move(updated_path));
    node.cost = calculate_sic(node);
}

void JPSCBS::print_node_info(const JPSCBSNode& node, const std::string& prefix) {
    if (!prefix.empty()) {
        //logger::log_info(prefix + ":");
    }
    //logger::log_info("Total cost: " + std::to_string(std::round(node.cost * 1000) / 1000.0));
    //logger::print_constraints(node.constraints, "Constraints");
    
    // 打印当前节点中每个智能体的跳点
    for (const auto& [agent_id, paths] : node.solution) {
        if (!paths.empty()) {
            std::string jump_points_str = "Agent " + std::to_string(agent_id) + " Current JPs: ";
            for (const auto& jp : paths.top().jump_points) {
                jump_points_str += "(" + std::to_string(jp.x) + "," + std::to_string(jp.y) + ") ";
            }
            //logger::log_info(jump_points_str);
            //logger::log_info("Agent " + std::to_string(agent_id) + " path: " + //logger::vectorToString(paths.top().path));
        }
    }

    // 打印solutions中存储的所有路径信息
    // //logger::log_info("All stored solutions:");
    // for (const auto& [agent_id, agent_paths] : solutions) {
    //     //logger::log_info("Agent " + std::to_string(agent_id) + " stored paths:");
    //     for (size_t i = 0; i < agent_paths.size(); ++i) {
    //         const auto& path = agent_paths[i];
    //         double path_cost = utils::calculate_path_cost(path.path);
            
    //         std::string path_info = "  Path " + std::to_string(i) + 
    //                               " (cost: " + std::to_string(std::round(path_cost * 1000) / 1000.0) + 
    //                               ") JPs: ";
            
    //         for (const auto& jp : path.jump_points) {
    //             path_info += "(" + std::to_string(jp.x) + "," + std::to_string(jp.y) + ") ";
    //         }
    //         //logger::log_info(path_info);
    //     }
    // }
        
    //logger::log_info("------------------------");
}


std::vector<ConstraintInfo> JPSCBS::collect_constraint_infos(const JPSCBSNode& node, 
                                                           const std::vector<Constraint>& constraints) {
    std::vector<ConstraintInfo> constraint_infos;
    ////logger::log_info("Starting to generate constraint info");
    ////logger::print_constraints(constraints);

    for(const auto& constraint : constraints) {
        if (node.solution.find(constraint.agent) == node.solution.end() || 
            node.solution.at(constraint.agent).empty()) {
            ////logger::log_warning("Agent " + std::to_string(constraint.agent) + " has no path");
            continue;
        }

        if(constraint.time == 0) {
            continue;
        }

        const auto& current_path = node.solution.at(constraint.agent).top();
        const auto& path = current_path.path;
        const auto& jump_points = current_path.jump_points;
        
        // 找到约束点在路径中的位置
        auto constraint_pos = [&]() -> std::vector<Vertex>::const_iterator {
            if (constraint.type == Constraint::VERTEX) {
                return std::find(path.begin(), path.end(), constraint.vertex_constraint.vertex);
            } else { // EDGE pick the second vertex
                return std::find(path.begin(), path.end(), constraint.edge_constraint.v2);
            }
        }();

        if (constraint_pos == path.end()) {
            ////logger::log_warning("Constraint vertex not found in path for agent " + 
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
                ////logger::log_warning("Jump point not found in path");
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
                ////logger::log_info("Found constraint info for agent " + 
                //                 std::to_string(constraint.agent));
                break;
            }
        }
    }

    for (const auto& info : constraint_infos) {
        ////logger::log_info("Constraint info: ");
        ////logger::log_info(info.toString());
    }

    if (constraint_infos.empty()) {
        ////logger::log_warning("No constraint infos generated");
    }

    ////logger::log_info("Collected constraint infos done");

    return constraint_infos;
}

int JPSCBS::count_conflicts(const JPSCBSNode& node) {
    int total_conflicts = 0;
    for (const auto& [agent_id, paths] : node.solution) {
        if (paths.empty()) continue;
        const auto& path1 = paths.top().path;
        
        for (const auto& [other_id, other_paths] : node.solution) {
            if (other_id <= agent_id || other_paths.empty()) continue;
            total_conflicts += utils::count_conflicts(path1, other_paths.top().path);
        }
    }
    return total_conflicts;
}

int JPSCBSNodeComparator::count_conflicts(const JPSCBSNode& node) {
    int total_conflicts = 0;
    for (const auto& [agent_id, paths] : node.solution) {
        if (paths.empty()) continue;
        const auto& path1 = paths.top().path;
        
        for (const auto& [other_id, other_paths] : node.solution) {
            if (other_id <= agent_id || other_paths.empty()) continue;
            total_conflicts += utils::count_conflicts(path1, other_paths.top().path);
        }
    }
    return total_conflicts;
}

ConflictAvoidanceTable JPSCBS::calculate_cat(const std::unordered_set<int>& excluded_agents, const JPSCBSNode& node) const {
    ConflictAvoidanceTable cat;
    
    for (const auto& [agent_id, paths] : node.solution) {
        if (excluded_agents.count(agent_id) > 0) continue;
        
        if (!paths.empty()) {
            const auto& path = paths.top().path;
            for (size_t t = 0; t < path.size(); ++t) {
                // 添加顶点约束
                cat.addVertexConstraint(path[t], t);
                
                // 添加边约束（对于相邻时间步的位置）
                if (t < path.size() - 1) {
                    // 添加双向的边约束
                    cat.addEdgeConstraint(path[t], path[t + 1], t);     // 从t到t+1的移动
                    cat.addEdgeConstraint(path[t + 1], path[t], t);     // 从t+1到t的移动（防止交换冲突）
                }
                
                // // 如果是路径的最后一个位置，添加终点约束
                // if (t == path.size() - 1) {
                //     cat.addGoalConstraint(path[t], t);
                // }
            }
        }
    }
    
    return cat;
}
