#include "Action.h"

namespace Action {

    const std::vector<Vertex> DIRECTIONS_8 = {
        Vertex(1, 0),   // SOUTH
        Vertex(0, 1),   // EAST
        Vertex(-1, 0),  // NORTH
        Vertex(0, -1),  // WEST
        Vertex(1, 1),   // SOUTHEAST
        Vertex(1, -1),  // SOUTHWEST
        Vertex(-1, 1),  // NORTHEAST
        Vertex(-1, -1), // NORTHWEST
    };

    const std::vector<Vertex> DIRECTIONS_STRAIGHT = {
        Vertex(1, 0),   // SOUTH
        Vertex(0, 1),   // EAST
        Vertex(-1, 0),  // NORTH
        Vertex(0, -1),  // WEST
    };

    const std::vector<Vertex> DIRECTIONS_DIAGONAL = {
        Vertex(1, 1),   // SOUTHEAST
        Vertex(1, -1),  // SOUTHWEST
        Vertex(-1, 1),  // NORTHEAST
        Vertex(-1, -1), // NORTHWEST
    };

    const std::vector<Vertex> MOVEMENTS_9 = {
        Vertex(1, 0),   // SOUTH
        Vertex(0, 1),   // EAST
        Vertex(-1, 0),  // NORTH
        Vertex(0, -1),  // WEST
        Vertex(1, 1),   // SOUTHEAST
        Vertex(1, -1),  // SOUTHWEST
        Vertex(-1, 1),  // NORTHEAST
        Vertex(-1, -1), // NORTHWEST
        Vertex(0, 0)    // WAIT
    };

    const std::vector<Vertex> MOVEMENTS_5 = {
        Vertex(1, 0),   // SOUTH
        Vertex(0, 1),   // EAST
        Vertex(-1, 0),  // NORTH
        Vertex(0, -1),  // WEST
        Vertex(0, 0)    // WAIT
    };

    const std::unordered_map<Move, Vertex> MOVEMENT_MAP = {
        {Move::MOVE_N,  Vertex(-1, 0)},  // NORTH
        {Move::MOVE_E,  Vertex(0, 1)},   // EAST
        {Move::MOVE_S,  Vertex(1, 0)},   // SOUTH
        {Move::MOVE_W,  Vertex(0, -1)},  // WEST
        {Move::MOVE_NE, Vertex(-1, 1)},  // NORTHEAST
        {Move::MOVE_SE, Vertex(1, 1)},   // SOUTHEAST
        {Move::MOVE_NW, Vertex(-1, -1)}, // NORTHWEST
        {Move::MOVE_SW, Vertex(1, -1)},  // SOUTHWEST
        {Move::WAIT,    Vertex(0, 0)}    // WAIT
    };
} 