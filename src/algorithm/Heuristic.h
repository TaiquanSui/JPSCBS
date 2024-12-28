#ifndef UTILITY_H
#define UTILITY_H

#include "Vertex.h"
#include <cmath>

inline int heuristic(const Vertex& a, const Vertex& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

#endif // UTILITY_H
