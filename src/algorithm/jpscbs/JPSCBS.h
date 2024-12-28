#ifndef JPSCBS_H
#define JPSCBS_H

#include "../Vertex.h"
#include "../jps/JPS.h"
#include <unordered_map>
#include <queue>
#include <vector>
#include <tuple>

struct JPSCBSNode {
    std::unordered_map<int, std::priority_queue<JPSPath, std::vector<JPSPath>, JPSPathComparator>> solution;
    int cost;
    std::vector<std::tuple<int, int, Vertex, int>> conflicts; // (agent1, agent2, vertex, time)

    JPSCBSNode() : cost(0) {}
};



#endif // JPSCBS_H