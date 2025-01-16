#ifndef UTILITY_H
#define UTILITY_H

#include "Vertex.h"
#include <vector>
#include <cmath>
#include <chrono>
#include <iostream>

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

    inline int manhattanDistance(const Vertex& a, const Vertex& b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
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
        return (dx && dy && dx == dy) ? dx * std::sqrt(2.0) : dx;  // 对角线移动代价为√2，直线移动代价为1
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
        
        // 检查起点和终点
        if (path.front() != start || path.back() != goal) {
            return false;
        }
        
        // 检查路径连续性
        for (size_t i = 0; i < path.size() - 1; ++i) {
            const auto& current = path[i];
            const auto& next = path[i + 1];
            
            // 检查相邻点是否合法移动
            if (abs(current.x - next.x) > 1 || abs(current.y - next.y) > 1) {
                return false;
            }
            
            // 检查是否穿墙
            if (!isWalkable(grid, next.x, next.y)) {
                return false;
            }
        }
        
        return true;
    }

    inline bool validate_constraints(const std::vector<Vertex>& path,
                                const std::vector<Constraint>& constraints) {
        for (const auto& constraint : constraints) {
            if (constraint.time < path.size() && 
                path[constraint.time] == constraint.vertex) {
                log_error("Path violates constraint at time " + 
                         std::to_string(constraint.time));
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

    // 日志级别枚举
    enum class LogLevel {
        INFO,
        WARNING,
        ERROR
    };

    // 日志记录函数
    inline void log(LogLevel level, const std::string& message) {
        switch (level) {
            case LogLevel::INFO:
                std::cout << "[INFO] " << message << std::endl;
                break;
            case LogLevel::WARNING:
                std::cout << "\033[33m[WARNING] " << message << "\033[0m" << std::endl;
                break;
            case LogLevel::ERROR:
                std::cout << "\033[31m[ERROR] " << message << "\033[0m" << std::endl;
                break;
        }
    }

    // 便捷的日志记录函数
    inline void log_info(const std::string& message) {
        log(LogLevel::INFO, message);
    }

    inline void log_warning(const std::string& message) {
        log(LogLevel::WARNING, message);
    }

    inline void log_error(const std::string& message) {
        log(LogLevel::ERROR, message);
    }
}

#endif // UTILITY_H 