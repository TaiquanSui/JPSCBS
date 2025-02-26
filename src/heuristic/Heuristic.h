#ifndef HEURISTIC_H
#define HEURISTIC_H

#include "../utilities/GridUtility.h"
#include "../basic/Vertex.h"
#include <cmath>

inline double heuristic(const Vertex& a, const Vertex& b) {
    return utils::octileDistance(a, b);
}

#endif // HEURISTIC_H
