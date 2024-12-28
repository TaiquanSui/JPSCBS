#ifndef ACTION_H
#define ACTION_H

#include "../Vertex.h"
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

    extern const std::vector<Vertex> DIRECTIONS_8;  // 8方向移动
    extern const std::vector<Vertex> DIRECTIONS_4;  // 4方向移动
    extern const Vertex WAIT_MOVE;                  // 等待动作
    extern const std::unordered_map<Move, Vertex> DIRECTION_MAP;  // 方向映射
}

#endif // ACTION_H