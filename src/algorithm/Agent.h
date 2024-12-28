#ifndef AGENT_H
#define AGENT_H

#include "Vertex.h"
#include "jps/JPSState.h"
#include <vector>

struct Agent {
    int id; // Unique identifier for the agent
    Vertex start;
    Vertex goal;
    JPSState state;

    Agent(int id, Vertex start, Vertex goal)
        : id(id), start(start), goal(goal) {}

    // Get the current best path for the agent
    std::vector<Vertex> get_best_path() const {
        return state.best_path;
    }

    // Check if the search for this agent is complete
    bool is_search_complete() const {
        return state.is_complete;
    }
};

#endif // AGENT_H
