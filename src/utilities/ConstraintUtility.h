#ifndef CONSTRAINTUTILITY_H
#define CONSTRAINTUTILITY_H

#include "../basic/Vertex.h"
#include "../jpscbs/JPSCBS.h"
#include "GridUtility.h"
#include <vector>

namespace utils {
    inline std::vector<Constraint> find_violated_constrains(int agent_id, 
                                    const std::vector<Vertex>& path,
                                    const std::vector<Constraint>& constraints) {
        std::vector<Constraint> violated_constraints;
        for (const auto& constraint : constraints) {
            if(constraint.agent != agent_id) continue;
            
            // 检查顶点约束
            if (constraint.type == Constraint::VERTEX) {
                if (constraint.time < path.size() && 
                    path[constraint.time] == constraint.vertex_constraint.vertex) {
                    violated_constraints.push_back(constraint);
                }
            }
            // 检查边约束
            else if (constraint.type == Constraint::EDGE) {
                if (constraint.time < path.size() - 1 && 
                    path[constraint.time] == constraint.edge_constraint.v1 &&
                    path[constraint.time + 1] == constraint.edge_constraint.v2) {
                    violated_constraints.push_back(constraint);
                }
            }
        }
        return violated_constraints;
    }

    inline int count_conflicts(const std::vector<Vertex>& path1, 
                             const std::vector<Vertex>& path2) {
        int conflicts = 0;
        int min_length = std::min(path1.size(), path2.size());
        
        for (int i = 0; i < min_length; ++i) {
            // 检查顶点冲突
            if (path1[i] == path2[i]) {
                conflicts++;
            }
            
            // 检查交换冲突、跟随冲突和对角线交叉冲突
            if (i < min_length - 1) {
                // 交换冲突
                if (path1[i] == path2[i+1] && path1[i+1] == path2[i]) {
                    conflicts++;
                }
                
                // 跟随冲突
                // if (path1[i+1] == path2[i] || path2[i+1] == path1[i]) {
                //     conflicts++;
                // }
                
                // 对角线交叉冲突
                Vertex dir1 = path1[i+1] - path1[i];
                Vertex dir2 = path2[i+1] - path2[i];
                if (isDiagonal(dir1) && isDiagonal(dir2)) {
                    // 检查水平分量相交
                    if (path1[i] + Vertex(dir1.x,0) == path2[i] && 
                        Vertex(-dir1.x,dir1.y) == dir2) {
                        conflicts++;
                    }
                    // 检查垂直分量相交
                    if (path1[i] + Vertex(0,dir1.y) == path2[i] && 
                        Vertex(dir1.x,-dir1.y) == dir2) {
                        conflicts++;
                    }
                }
            }
        }
        
        return conflicts;
    }

    

    // 检查并生成顶点冲突的约束
    inline bool check_vertex_conflict(const Vertex& pos1, const Vertex& pos2, int time,
                                    int agent1, int agent2, std::vector<Constraint>& constraints) {
        if (pos1 == pos2) {
            constraints.emplace_back(agent1, pos1, time);
            constraints.emplace_back(agent2, pos2, time);
            return true;
        }
        return false;
    }

    // 检查并生成交换冲突的约束
    inline bool check_edge_conflict(const Vertex& pos1, const Vertex& next_pos1,
                                  const Vertex& pos2, const Vertex& next_pos2,
                                  int time, int agent1, int agent2,
                                  std::vector<Constraint>& constraints) {
        if (pos1 == next_pos2 && pos2 == next_pos1) {
            constraints.emplace_back(agent1, pos1, next_pos1, time);
            constraints.emplace_back(agent2, pos2, next_pos2, time);
            return true;
        }
        return false;
    }

    // 检查并生成跟随冲突的约束
    inline bool check_following_conflict(const Vertex& pos1, const Vertex& next_pos1,
                                      const Vertex& pos2, const Vertex& next_pos2,
                                      int time, int agent1, int agent2,
                                      std::vector<Constraint>& constraints) {
        if (next_pos1 == pos2) {  // agent1 follows agent2
            constraints.emplace_back(agent1, pos2, time + 1);
            constraints.emplace_back(agent2, pos2, time);
            return true;
        }
        if (next_pos2 == pos1) {  // agent2 follows agent1
            constraints.emplace_back(agent1, pos1, time);
            constraints.emplace_back(agent2, pos1, time + 1);
            return true;
        }
        return false;
    }

    // 检查并生成对角线交叉冲突的约束
    inline bool check_diagonal_conflict(const Vertex& pos1, const Vertex& next_pos1,
                                      const Vertex& pos2, const Vertex& next_pos2,
                                      int time, int agent1, int agent2,
                                      std::vector<Constraint>& constraints) {
        Vertex dir1 = next_pos1 - pos1;
        Vertex dir2 = next_pos2 - pos2;
        if (isDiagonal(dir1) && isDiagonal(dir2)) {
            if (pos1 + Vertex(dir1.x,0) == pos2 && Vertex(-dir1.x,dir1.y) == dir2) {
                constraints.emplace_back(agent1, pos1, next_pos1, time);
                constraints.emplace_back(agent2, pos2, next_pos2, time);
                return true;
            }
            if (pos1 + Vertex(0,dir1.y) == pos2 && Vertex(dir1.x,-dir1.y) == dir2) {
                constraints.emplace_back(agent1, pos1, next_pos1, time);
                constraints.emplace_back(agent2, pos2, next_pos2, time);
                return true;
            }
        }
        return false;
    }

    // 生成所有类型的约束
    template<typename NodeType>
    inline std::vector<Constraint> generate_constraints(const NodeType& node) {
        std::vector<Constraint> constraints;
        
        // 找到最长路径的长度
        int max_length = 0;
        for (const auto& [_, paths] : node.solution) {
            if constexpr (std::is_same_v<NodeType, JPSCBSNode>) {
                if (!paths.empty()) {
                    max_length = std::max(max_length, static_cast<int>(paths.top().path.size()));
                }
            } else {
                max_length = std::max(max_length, static_cast<int>(paths.size()));
            }
        }

        for (int i = 0; i < max_length; ++i) {
            for (const auto& [agent1, paths1] : node.solution) {
                const auto& path1 = [&]() {
                    if constexpr (std::is_same_v<NodeType, JPSCBSNode>) {
                        return paths1.empty() ? std::vector<Vertex>{} : paths1.top().path;
                    } else {
                        return paths1;
                    }
                }();
                
                if (i >= path1.size()) continue;

                for (const auto& [agent2, paths2] : node.solution) {
                    if (agent1 >= agent2) continue;

                    const auto& path2 = [&]() {
                        if constexpr (std::is_same_v<NodeType, JPSCBSNode>) {
                            return paths2.empty() ? std::vector<Vertex>{} : paths2.top().path;
                        } else {
                            return paths2;
                        }
                    }();

                    if (i >= path2.size()) continue;

                    // 检查顶点冲突
                    if (check_vertex_conflict(path1[i], path2[i], i, agent1, agent2, constraints)) {
                        return constraints;
                    }

                    if (i < max_length - 1 && i + 1 < path1.size() && i + 1 < path2.size()) {
                        // 检查交换冲突
                        if (check_edge_conflict(path1[i], path1[i+1], path2[i], path2[i+1], 
                                             i, agent1, agent2, constraints)) {
                            return constraints;
                        }

                        // 检查跟随冲突
                        // if (check_following_conflict(path1[i], path1[i+1], path2[i], path2[i+1],
                        //                           i, agent1, agent2, constraints)) {
                        //     return constraints;
                        // }

                        // 检查对角线交叉冲突
                        if (check_diagonal_conflict(path1[i], path1[i+1], path2[i], path2[i+1],
                                                i, agent1, agent2, constraints)) {
                            return constraints;
                        }
                    }
                }
            }
        }
        
        return constraints;
    }
}

#endif //CONSTRAINTUTILITY_H
