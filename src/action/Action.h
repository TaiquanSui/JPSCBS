#ifndef ACTION_H
#define ACTION_H

#include "../basic/Vertex.h"
#include <vector>
#include <string>
#include <unordered_map>

namespace Action {
    enum class Move {
        MOVE_N,    // North
        MOVE_E,    // East
        MOVE_S,    // South
        MOVE_W,    // West
        MOVE_NE,   // Northeast
        MOVE_SE,   // Southeast
        MOVE_NW,   // Northwest
        MOVE_SW,   // Southwest
        WAIT       // Wait
    };

    extern const std::vector<Vertex> DIRECTIONS_8;
    extern const std::vector<Vertex> DIRECTIONS_STRAIGHT;
    extern const std::vector<Vertex> DIRECTIONS_DIAGONAL;
    extern const std::vector<Vertex> MOVEMENTS_9;
    extern const std::vector<Vertex> MOVEMENTS_5;
    extern const std::unordered_map<Move, Vertex> MOVEMENT_MAP;  // Direction map
}

#endif // ACTION_H