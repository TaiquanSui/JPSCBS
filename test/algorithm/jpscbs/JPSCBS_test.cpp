#include <gtest/gtest.h>
#include "src/algorithm/jpscbs/JPSCBS.h"
#include "src/algorithm/Utility.h"

class JPSCBSTest : public ::testing::Test {
protected:
    JPSCBS solver;
    std::vector<std::vector<int>> grid;
    
    void SetUp() override {
        // Create a simple 4x4 grid
        grid = {
            {0, 0, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 0, 0},
            {0, 0, 0, 0}
        };
    }
};

TEST_F(JPSCBSTest, SimplePathFinding) {
    std::vector<Agent> agents = {
        Agent{0, Vertex{0, 0}, Vertex{3, 3}},
        Agent{1, Vertex{0, 3}, Vertex{3, 0}}
    };
    
    auto paths = solver.solve(agents, grid);
    ASSERT_FALSE(paths.empty());
    ASSERT_EQ(paths.size(), 2);
}

TEST_F(JPSCBSTest, NoSolution) {
    std::vector<Agent> agents = {
        Agent{0, Vertex{0, 0}, Vertex{0, 1}},
        Agent{1, Vertex{0, 1}, Vertex{0, 0}}
    };
    
    auto paths = solver.solve(agents, grid);
    ASSERT_TRUE(paths.empty());
}

TEST_F(JPSCBSTest, TimeLimit) {
    solver.set_time_limit(0.001);  // Set a very short timeout
    
    std::vector<Agent> agents;
    // Add multiple agents to make the problem complex
    for (int i = 0; i < 10; ++i) {
        agents.emplace_back(i, Vertex{0, i}, Vertex{3, 3-i});
    }
    
    auto paths = solver.solve(agents, grid);
    ASSERT_TRUE(paths.empty());
}

TEST_F(JPSCBSTest, PathValidation) {
    Agent agent{0, Vertex{0, 0}, Vertex{3, 3}};
    std::vector<Vertex> path = {Vertex{0, 0}, Vertex{0, 1}, Vertex{1, 1}, Vertex{1, 2}, Vertex{2, 2}, Vertex{3, 3}};
    
    ASSERT_TRUE(utils::validatePath(path, agent.start, agent.goal, grid));
} 