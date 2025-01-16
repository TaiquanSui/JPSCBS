#include "JPSCBS.h"

std::vector<std::vector<Vertex>> JPSCBS::solve(const std::vector<Agent>& agents, 
                                              const std::vector<std::vector<int>>& grid) {
    start_time = std::chrono::steady_clock::now();
    this->grid = grid;
    
    utils::log_info("Starting JPSCBS solver");
    
    // Initialize root node and solutions
    auto root = std::make_shared<JPSCBSNode>();
    
    // Find initial paths for each agent
    for (const auto& agent : agents) {
        JPSPath path = search_by_jps(agent);
        if (path.path.empty()) {
            utils::log_error("No initial path found for agent " + std::to_string(agent.id));
            return {};
        }
        
        // Validate initial path
        if (!utils::validatePath(path.path, agent.start, agent.goal, grid)) {
            utils::log_error("Invalid initial path for agent " + std::to_string(agent.id));
            return {};
        }
        
        solutions[agent.id].push_back(path);
        std::priority_queue<JPSPath, std::vector<JPSPath>, JPSPathComparator> paths;
        paths.push(path);
        root->solution[agent.id] = paths;
    }
    
    root->cost = calculate_sic(root->solution);
    
    std::priority_queue<std::shared_ptr<JPSCBSNode>, 
                       std::vector<std::shared_ptr<JPSCBSNode>>, 
                       JPSCBSNodeComparator> open_list;
    open_list.push(root);
    
    while (!open_list.empty()) {
        // Check timeout
        if (utils::getElapsedTime(start_time) > time_limit) {
            utils::log_warning("Time limit exceeded");
            return {};
        }
        
        auto current = open_list.top();
        open_list.pop();
        
        // Update current node's solution
        update_solutions(agents, *current);
        
        // Detect conflicts
        auto conflicts = detect_conflicts(*current);
        
        if (conflicts.empty()) {
            std::vector<std::vector<Vertex>> final_paths;
            for (const auto& agent : agents) {
                final_paths.push_back(current->solution[agent.id].top().path);
            }
            return final_paths;
        }
        
        const auto& conflict = conflicts[0];
        
        if (find_alt_symmetric_paths(*current, conflict)) {
            continue;
        }
        
        std::vector<Constraint> new_constraints = {
            Constraint{conflict.agent1, conflict.vertex, conflict.time},
            Constraint{conflict.agent2, conflict.vertex, conflict.time}
        };
        
        for (const auto& constraint : new_constraints) {
            auto new_node = std::make_shared<JPSCBSNode>(*current);
            new_node->constraints.push_back(constraint);
            resolve_conflict_locally(*new_node, constraint);
            new_node->cost = calculate_sic(new_node->solution);
            open_list.push(new_node);
        }
    }
    
    return {};  // No solution
}

JPSPath JPSCBS::search_by_jps(const Agent& agent) {
    // Get or create agent's search state
    auto& state = agent_states[agent.id];
    
    // Use JPS to search for a new path
    return jump_point_search(agent.start, agent.goal, grid, state);
}

int JPSCBS::calculate_sic(const std::unordered_map<int, std::priority_queue<JPSPath, 
                         std::vector<JPSPath>, JPSPathComparator>>& solution) {
    int total_cost = 0;
    for (const auto& [_, paths] : solution) {
        if (!paths.empty()) {
            total_cost += paths.top().path.size() - 1;
        }
    }
    return total_cost;
}


void JPSCBS::update_solutions(const std::vector<Agent>& agents, JPSCBSNode& node) {
    for (auto& [agent_id, agent_paths] : node.solution) {
        // Check if we need to continue searching for a new path
        while (!agent_paths.empty() && !solutions[agent_id].empty() && 
               agent_paths.top().path.size() > solutions[agent_id].back().path.size()) {
            
            // Continue searching for a new path
            JPSPath new_path = search_by_jps(agents[agent_id]);
            
            if (!new_path.path.empty()) {
                solutions[agent_id].push_back(new_path);
            } else {
                break;
            }
        }
        
        // Add all paths with lower cost to current node's solution
        for (const auto& path : solutions[agent_id]) {
            if (path.path.size() < agent_paths.top().path.size()) {
                agent_paths.push(path);
            }
        }
    }
}


std::vector<Conflict> JPSCBS::detect_conflicts(const JPSCBSNode& node) {
    std::vector<Conflict> conflicts;
    
    // Check conflicts between all agent pairs
    for (const auto& [agent1_id, paths1] : node.solution) {
        for (const auto& [agent2_id, paths2] : node.solution) {
            if (agent1_id >= agent2_id) continue;
            
            const auto& path1 = paths1.top().path;
            const auto& path2 = paths2.top().path;
            
            size_t max_length = std::max(path1.size(), path2.size());
            
            for (size_t t = 0; t < max_length; ++t) {
                Vertex pos1 = t < path1.size() ? path1[t] : path1.back();
                Vertex pos2 = t < path2.size() ? path2[t] : path2.back();
                
                // Check vertex conflict
                if (pos1 == pos2) {
                    conflicts.emplace_back(agent1_id, agent2_id, pos1, t);
                    continue;
                }
                
                // Check swap conflict and following conflict
                if (t < max_length - 1) {
                    Vertex next_pos1 = (t + 1) < path1.size() ? path1[t + 1] : path1.back();
                    Vertex next_pos2 = (t + 1) < path2.size() ? path2[t + 1] : path2.back();
                    
                    if (pos1 == next_pos2 && pos2 == next_pos1) { // Swap conflict
                        conflicts.emplace_back(agent1_id, agent2_id, pos1, t);
                        continue;
                    }
                    if (next_pos1 == pos2) { // agent1 follows agent2
                        conflicts.emplace_back(agent1_id, agent2_id, pos2, t);
                        continue;
                    }
                    if (next_pos2 == pos1) { // agent2 follows agent1
                        conflicts.emplace_back(agent1_id, agent2_id, pos1, t);
                        continue;
                    }
                }
            }
        }
    }
    
    return conflicts;
}


bool JPSCBS::find_alt_symmetric_paths(JPSCBSNode& node, const Conflict& conflict) {
    const auto& path1 = node.solution[conflict.agent1].top();
    const auto& path2 = node.solution[conflict.agent2].top();
    
    return find_local_bypass(path1, conflict.agent1, node, conflict.vertex) ||
           find_local_bypass(path2, conflict.agent2, node, conflict.vertex);
}

bool JPSCBS::find_local_bypass(const JPSPath& path, int agent_id, 
                                  JPSCBSNode& node, const Vertex& conflict_vertex) {
    for (const auto& interval : path.possible_intervals) {
        // Ensure interval contains at least two points
        if (interval.jump_points.size() < 2) continue;
        
        // Find start and end positions of interval in path
        auto start_it = std::find(path.path.begin(), path.path.end(), interval.get_start());
        auto end_it = std::find(start_it, path.path.end(), interval.get_end());
        
            // Validate iterators
        if (start_it == path.path.end() || end_it == path.path.end() || start_it >= end_it) {
            continue;
        }
        
        // Calculate start time in path
        int start_time = std::distance(path.path.begin(), start_it);
        
        // Check if conflict_vertex is in this path segment
        auto conflict_it = std::find(start_it, end_it + 1, conflict_vertex);
        if (conflict_it == end_it + 1) continue;
        
        // Add temporary constraint
        std::vector<Constraint> temp_constraints = node.constraints;
        temp_constraints.emplace_back(agent_id, conflict_vertex, start_time);
        
        // Try to find an alternative path
        auto alt_path = a_star(agent_id, interval.get_start(), interval.get_end(), 
                             grid, temp_constraints, start_time);
        
        if (!alt_path.empty()) {
            auto& agent_paths = node.solution[agent_id];
            agent_paths.pop();

            std::vector<Vertex> new_path;
            new_path.insert(new_path.end(), path.path.begin(), start_it);
            new_path.insert(new_path.end(), alt_path.begin(), alt_path.end());
            new_path.insert(new_path.end(), end_it + 1, path.path.end());

            JPSPath updated_path = path;
            updated_path.path = new_path;
            agent_paths.push(updated_path);
            return true;
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
    new_path.insert(new_path.end(), current_path.path.begin(), start_it);
    new_path.insert(new_path.end(), local_path.begin(), local_path.end());
    new_path.insert(new_path.end(), end_it + 1, current_path.path.end());
    
    // Update jump points
    std::vector<Vertex> new_jump_points;
    auto is_jump_point = [&](const Vertex& v) {
        return std::find(current_path.jump_points.begin(), 
                        current_path.jump_points.end(), v) != current_path.jump_points.end();
    };
    
    // Keep jump points before start point
    for (const auto& jp : current_path.jump_points) {
        if (std::find(current_path.path.begin(), start_it, jp) != start_it) {
            new_jump_points.push_back(jp);
        }
    }
    
    // Add jump points from local_path
    for (const auto& v : local_path) {
        if (is_jump_point(v)) {
            new_jump_points.push_back(v);
        }
    }
    
    // Add jump points after end point
    for (const auto& jp : current_path.jump_points) {
        if (std::find(end_it + 1, current_path.path.end(), jp) != current_path.path.end()) {
            new_jump_points.push_back(jp);
        }
    }
    
    // Update possible intervals
    std::vector<Interval> new_intervals;
    for (const auto& interval : current_path.possible_intervals) {
        // Check if interval is completely outside the modified area
        if (std::find(current_path.path.begin(), start_it, interval.get_start()) != start_it ||
            std::find(end_it + 1, current_path.path.end(), interval.get_end()) != current_path.path.end()) {
            new_intervals.push_back(interval);
        }
    }
    
    // Create updated path
    JPSPath updated_path = current_path;
    updated_path.path = std::move(new_path);
    updated_path.jump_points = std::move(new_jump_points);
    updated_path.possible_intervals = std::move(new_intervals);
    
    agent_paths.push(std::move(updated_path));
}
