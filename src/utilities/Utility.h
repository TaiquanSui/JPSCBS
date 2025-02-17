#ifndef UTILITY_H
#define UTILITY_H

#include "../Vertex.h"
#include "../cbs/CBS.h"
#include <vector>
#include <cmath>
#include <chrono>


namespace utils {
    inline Vertex calculateDirection(const Vertex& from, const Vertex& to) {
        int dx = (to.x > from.x) ? 1 : ((to.x < from.x) ? -1 : 0);
        int dy = (to.y > from.y) ? 1 : ((to.y < from.y) ? -1 : 0);
        return Vertex(dx, dy);
    }

    inline bool isDiagonal(const Vertex& from, const Vertex& to) {
        Vertex dir = calculateDirection(from, to);
        return dir.x != 0 && dir.y != 0 && abs(dir.x) == abs(dir.y);
    }

    inline bool isStraight(const Vertex& from, const Vertex& to) {
        Vertex dir = calculateDirection(from, to);
        return dir.x == 0 || dir.y == 0;
    }

    inline bool isDiagonal(const Vertex& dir) {
        return dir.x != 0 && dir.y != 0 && abs(dir.x) == abs(dir.y);
    }

    inline bool isStraight(const Vertex& dir) {
        return (dir.x == 0 && dir.y != 0) || (dir.x != 0 && dir.y == 0);
    }

    inline bool isPassable (const std::vector<std::vector<int>>& grid, const Vertex& pos) {
        return pos.y >= 0 && pos.y < grid.size() && 
               pos.x >= 0 && pos.x < grid[0].size() && 
               grid[pos.y][pos.x] == 0;
    }

    inline bool isPassable(const std::vector<std::vector<int>>& grid, int x, int y) {
        return y >= 0 && y < grid.size() && 
               x >= 0 && x < grid[0].size() && 
               grid[y][x] == 0;
    }

    inline bool isWalkable(const std::vector<std::vector<int>>& grid, const Vertex& from, const Vertex& to) {
        if (!isPassable(grid, to)) return false;
        if (!isPassable(grid, from)) return false;
        Vertex dir = to-from;
        if (isDiagonal(dir) 
        && !isPassable(grid, from.x + dir.x, from.y) 
        && !isPassable(grid, from.x, from.y + dir.y)){
            return false;
        }
        return true;
    }

    inline int chebyshevDistance(const Vertex& a, const Vertex& b) {
        return std::max(std::abs(a.x - b.x), std::abs(a.y - b.y));
    }

    inline double euclideanDistance(const Vertex& a, const Vertex& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    inline double octileDistance(const Vertex& a, const Vertex& b) {
        int dx = std::abs(a.x - b.x);
        int dy = std::abs(a.y - b.y);
        return std::sqrt(2.0) * std::min(dx, dy) + std::max(dx, dy) - std::min(dx, dy);
    }

    inline double getMoveCost(const Vertex& from, const Vertex& to) {
        int dx = std::abs(to.x - from.x);
        int dy = std::abs(to.y - from.y);
        return (dx && dy && dx == dy) ? dx * std::sqrt(2.0) : (dx + dy);
    }

    inline bool validatePath(const std::vector<Vertex>& path, 
                           const Vertex& start, 
                           const Vertex& goal,
                           const std::vector<std::vector<int>>& grid) {
        if (path.empty()) return false;
        
        // Check start and goal
        if (path.front() != start || path.back() != goal) {
            return false;
        }
        
        // Check path continuity
        for (size_t i = 0; i < path.size() - 1; ++i) {
            const auto& current = path[i];
            const auto& next = path[i + 1];
            
            // Check if adjacent points are valid moves
            if (abs(current.x - next.x) > 1 || abs(current.y - next.y) > 1) {
                return false;
            }
            
            // Check if crossing walls
            if (!isWalkable(grid, current, next)) {
                return false;
            }
        }
        
        return true;
    }

    inline bool validate_constrains(const std::vector<Constraint>& constraints, 
                                    const std::vector<Vertex>& path) {
        for (const auto& constraint : constraints) {
            for (size_t i = 0; i < path.size(); ++i) {
                if (path[i] == constraint.vertex && i == constraint.time) {
                    return false;
                }
            }   
        }
        return true;
    }

    inline std::vector<Constraint> find_violated_constrains(int agent_id, 
                                    const std::vector<Vertex>& path,
                                    const std::vector<Constraint>& constraints) {
        std::vector<Constraint> violated_constraints;
        for (const auto& constraint : constraints) {
            if(constraint.agent != agent_id) continue;
            for (size_t i = 0; i < path.size(); ++i) {
                if (path[i] == constraint.vertex && i == constraint.time) {
                    violated_constraints.push_back(constraint);
                }
            }   
        }
        return violated_constraints;
    }

    inline bool is_valid_move(const std::vector<Constraint>& constraints, 
                      const int agent_id, const Vertex& pos, const int time) {
        for (const auto& constraint : constraints) {
            if (constraint.agent == agent_id && 
                constraint.vertex == pos && 
                constraint.time == time) {
                return false;
            }
        }
        return true;
    }

    inline double getElapsedTime(const std::chrono::steady_clock::time_point& start_time) {
        auto current_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>
                       (current_time - start_time);
        return duration.count() / 1000.0;
    }

    inline int count_conflicts(
        const std::vector<Vertex>& path1,
        const std::vector<Vertex>& path2
    ) {
        int conflict_count = 0;
        
        size_t max_length = std::max(path1.size(), path2.size());
        
        // check each time step
        for (size_t t = 0; t < max_length; ++t) {
            // get current position (if path ends, use goal)
            Vertex pos1 = t < path1.size() ? path1[t] : path1.back();
            Vertex pos2 = t < path2.size() ? path2[t] : path2.back();

            // check vertex conflict
            if (pos1 == pos2) {
                conflict_count++;
                continue;
            }

            // check following and swapping conflicts
            if (t < max_length - 1) {
                Vertex next_pos1 = (t + 1) < path1.size() ? path1[t + 1] : path1.back();
                Vertex next_pos2 = (t + 1) < path2.size() ? path2[t + 1] : path2.back();
                
                // check swapping conflict
                if (pos1 == next_pos2 && pos2 == next_pos1) {
                    conflict_count++;
                    continue;
                }
                
                // check following conflict
                // if (next_pos1 == pos2) { // agent1 follows agent2
                //     conflict_count++;
                //     continue;
                // }
                // if (next_pos2 == pos1) { // agent2 follows agent1
                //     conflict_count++;
                //     continue;
                // }
            }
        }
    
        return conflict_count;
    }

    inline double calculate_path_cost(const std::vector<Vertex>& path) {
        double cost = 0;
        for (size_t i = 0; i < path.size() - 1; ++i) {
            if (isDiagonal(path[i+1] - path[i])) {
                cost += std::sqrt(2.0);
            } else {
                cost += 1.0;
            }
        }
        return cost;
    }   

}

#endif // UTILITY_H 