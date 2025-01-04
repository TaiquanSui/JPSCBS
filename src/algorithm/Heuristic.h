#ifndef HEURISTIC_H
#define HEURISTIC_H

#include "Utility.h"
#include "Vertex.h"
#include <cmath>

inline int heuristic(const Vertex& a, const Vertex& b) {
    return Utility::manhattanDistance(a, b);
}

#endif // HEURISTIC_H
