#ifndef UTILITY_H
#define UTILITY_H

#include "Vertex.h"
#include <vector>
#include <cmath>

class Utility {
public:
    static inline bool isWalkable(const std::vector<std::vector<int>>& grid, const Vertex& pos) {
        return pos.x >= 0 && pos.x < grid.size() && 
               pos.y >= 0 && pos.y < grid[0].size() && 
               grid[pos.x][pos.y] == 0;
    }

    static inline bool isWalkable(const std::vector<std::vector<int>>& grid, int x, int y) {
        return x >= 0 && x < grid.size() && 
               y >= 0 && y < grid[0].size() && 
               grid[x][y] == 0;
    }

    static inline int manhattanDistance(const Vertex& a, const Vertex& b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

    static inline double euclideanDistance(const Vertex& a, const Vertex& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    static inline Vertex calculateDirection(const Vertex& from, const Vertex& to) {
        int dx = (to.x > from.x) ? 1 : ((to.x < from.x) ? -1 : 0);
        int dy = (to.y > from.y) ? 1 : ((to.y < from.y) ? -1 : 0);
        return Vertex(dx, dy);
    }

    static inline double getMoveCost(const Vertex& from, const Vertex& to) {
        int dx = std::abs(to.x - from.x);
        int dy = std::abs(to.y - from.y);
        return (dx && dy) ? std::sqrt(2.0) : 1.0;  // 对角线移动代价为√2，直线移动代价为1
    }
};

#endif // UTILITY_H 