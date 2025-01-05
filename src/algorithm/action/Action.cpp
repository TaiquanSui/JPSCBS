#include "Action.h"

namespace Action {

    const std::vector<Vertex> DIRECTIONS_8 = {
        Vertex(0, 1),   // NORTH
        Vertex(1, 0),   // EAST
        Vertex(0, -1),  // SOUTH
        Vertex(-1, 0),  // WEST
        Vertex(1, 1),   // NORTHEAST
        Vertex(1, -1),  // SOUTHEAST
        Vertex(-1, 1),  // NORTHWEST
        Vertex(-1, -1),  // SOUTHWEST
    };

    const std::vector<Vertex> DIRECTIONS_4 = {
        Vertex(0, 1),   // NORTH
        Vertex(1, 0),   // EAST
        Vertex(0, -1),  // SOUTH
        Vertex(-1, 0),  // WEST
    };

    const std::vector<Vertex> MOVEMENTS_9 = {
        Vertex(0, 1),   // NORTH
        Vertex(1, 0),   // EAST
        Vertex(0, -1),  // SOUTH
        Vertex(-1, 0),  // WEST
        Vertex(1, 1),   // NORTHEAST
        Vertex(1, -1),  // SOUTHEAST
        Vertex(-1, 1),  // NORTHWEST
        Vertex(-1, -1),  // SOUTHWEST
        Vertex(0, 0)     // WAIT
    };

    const std::vector<Vertex> MOVEMENTS_5 = {
        Vertex(0, 1),   // NORTH
        Vertex(1, 0),   // EAST
        Vertex(0, -1),  // SOUTH
        Vertex(-1, 0),   // WEST
        Vertex(0, 0)     // WAIT
    };

    const std::unordered_map<Move, Vertex> MOVEMENT_MAP = {
        {Move::MOVE_N,  Vertex(0, 1)},
        {Move::MOVE_E,  Vertex(1, 0)},
        {Move::MOVE_S,  Vertex(0, -1)},
        {Move::MOVE_W,  Vertex(-1, 0)},
        {Move::MOVE_NE, Vertex(1, 1)},
        {Move::MOVE_SE, Vertex(1, -1)},
        {Move::MOVE_NW, Vertex(-1, 1)},
        {Move::MOVE_SW, Vertex(-1, -1)},
        {Move::WAIT,    Vertex(0, 0)}
    };
} 