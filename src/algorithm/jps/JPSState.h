#ifndef JPSSTATE_H
#define JPSSTATE_H

#include <memory>

#include "../astar/AStar.h"
#include <queue>
#include <vector>

struct JPSState {
    std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, AStarNodeComparator> open_list;
    std::vector<Vertex> best_path;
    bool is_complete;

    JPSState() : is_complete(false) {}
};

#endif // JPSSTATE_H