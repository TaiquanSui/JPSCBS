#include "Action.h"

namespace Action {
    const Vertex WAIT_MOVE(0, 0);

    const std::vector<Vertex> DIRECTIONS_8 = {
        Vertex(0, 1),   // NORTH
        Vertex(1, 0),   // EAST
        Vertex(0, -1),  // SOUTH
        Vertex(-1, 0),  // WEST
        Vertex(1, 1),   // NORTHEAST
        Vertex(1, -1),  // SOUTHEAST
        Vertex(-1, 1),  // NORTHWEST
        Vertex(-1, -1)  // SOUTHWEST
    };

    const std::vector<Vertex> DIRECTIONS_4 = {
        Vertex(0, 1),   // NORTH
        Vertex(1, 0),   // EAST
        Vertex(0, -1),  // SOUTH
        Vertex(-1, 0)   // WEST
    };

    const std::unordered_map<Move, Vertex> DIRECTION_MAP = {
        {Move::MOVE_N,  Vertex(0, 1)},
        {Move::MOVE_E,  Vertex(1, 0)},
        {Move::MOVE_S,  Vertex(0, -1)},
        {Move::MOVE_W,  Vertex(-1, 0)},
        {Move::MOVE_NE, Vertex(1, 1)},
        {Move::MOVE_SE, Vertex(1, -1)},
        {Move::MOVE_NW, Vertex(-1, 1)},
        {Move::MOVE_SW, Vertex(-1, -1)},
        {Move::WAIT,    WAIT_MOVE}
    };
} 