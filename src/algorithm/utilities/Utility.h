#ifndef UTILITY_H
#define UTILITY_H

#include "../Vertex.h"
#include "../cbs/CBS.h"
#include <vector>
#include <cmath>
#include <chrono>


namespace utils {
    inline bool isWalkable(const std::vector<std::vector<int>>& grid, const Vertex& pos) {
        return pos.x >= 0 && pos.x < grid.size() && 
               pos.y >= 0 && pos.y < grid[0].size() && 
               grid[pos.x][pos.y] == 0;
    }

    inline bool isWalkable(const std::vector<std::vector<int>>& grid, int x, int y) {
        return x >= 0 && x < grid.size() && 
               y >= 0 && y < grid[0].size() && 
               grid[x][y] == 0;
    }

    inline int chebyshevDistance(const Vertex& a, const Vertex& b) {
        return std::max(std::abs(a.x - b.x), std::abs(a.y - b.y));
    }

    inline double euclideanDistance(const Vertex& a, const Vertex& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    inline Vertex calculateDirection(const Vertex& from, const Vertex& to) {
        int dx = (to.x > from.x) ? 1 : ((to.x < from.x) ? -1 : 0);
        int dy = (to.y > from.y) ? 1 : ((to.y < from.y) ? -1 : 0);
        return Vertex(dx, dy);
    }

    inline double getMoveCost(const Vertex& from, const Vertex& to) {
        int dx = std::abs(to.x - from.x);
        int dy = std::abs(to.y - from.y);
        return (dx && dy && dx == dy) ? dx * std::sqrt(2.0) : (dx + dy);
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
        return dir.x == 0 || dir.y == 0;
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
            if (!isWalkable(grid, next.x, next.y)) {
                return false;
            }
        }
        
        return true;
    }

    inline bool is_valid_move(const std::vector<Constraint>& constraints, 
                      int agent_id, const Vertex& pos, int time) {
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

    // detect path conflicts and return constraints for both agents
    inline std::vector<Constraint> generate_constraints_from_conflict(
        int agent1_id, int agent2_id,
        const std::vector<Vertex>& path1,
        const std::vector<Vertex>& path2
    ) {
        std::vector<Constraint> constraints;  // 存储所有约束
        size_t max_length = std::max(path1.size(), path2.size());
        
        // check each time step
        for (size_t t = 0; t < max_length; ++t) {
            // get current position (if path ends, use goal)
            Vertex pos1 = t < path1.size() ? path1[t] : path1.back();
            Vertex pos2 = t < path2.size() ? path2[t] : path2.back();

            // check vertex conflict
            if (pos1 == pos2) {
                constraints.emplace_back(agent1_id, pos1, t);
                constraints.emplace_back(agent2_id, pos2, t);
                continue;
            }

            // check following and swapping conflicts
            if (t < max_length - 1) {
                Vertex next_pos1 = (t + 1) < path1.size() ? path1[t + 1] : path1.back();
                Vertex next_pos2 = (t + 1) < path2.size() ? path2[t + 1] : path2.back();
                
                // check swapping conflict
                if (pos1 == next_pos2 && pos2 == next_pos1) {
                    constraints.emplace_back(agent1_id, pos1, t);
                    constraints.emplace_back(agent2_id, pos2, t);
                    continue;
                }
                
                // check following conflict
                if (next_pos1 == pos2) { // agent1 follows agent2
                    constraints.emplace_back(agent1_id, pos2, t+1);
                    constraints.emplace_back(agent2_id, pos2, t);
                    continue;
                }
                if (next_pos2 == pos1) { // agent2 follows agent1
                    constraints.emplace_back(agent1_id, pos1, t+1);
                    constraints.emplace_back(agent2_id, pos1, t);
                    continue;
                }
            }
        }
        return constraints;
    }

}

#endif // UTILITY_H 