#ifndef AGENT_H
#define AGENT_H

#include "Vertex.h"
#include <vector>

struct Agent {
    int id; // Unique identifier for the agent
    Vertex start;
    Vertex goal;

    Agent(int id, Vertex start, Vertex goal)
        : id(id), start(start), goal(goal) {}
};

#endif // AGENT_H
