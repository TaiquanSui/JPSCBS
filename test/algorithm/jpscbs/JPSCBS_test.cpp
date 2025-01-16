#include <gtest/gtest.h>
#include "src/algorithm/jpscbs/JPSCBS.h"

class JPSCBSTest : public ::testing::Test {
protected:
    JPSCBS solver;
    std::vector<std::vector<int>> grid;
    
    void SetUp() override {
        // 创建一个简单的4x4网格
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
    solver.set_time_limit(0.001);  // 设置很短的超时时间
    
    std::vector<Agent> agents;
    // 添加多个智能体使问题变得复杂
    for (int i = 0; i < 10; ++i) {
        agents.emplace_back(i, Vertex{0, i}, Vertex{3, 3-i});
    }
    
    auto paths = solver.solve(agents, grid);
    ASSERT_TRUE(paths.empty());
}

TEST_F(JPSCBSTest, PathValidation) {
    Agent agent{0, Vertex{0, 0}, Vertex{3, 3}};
    std::vector<Vertex> path = {{0, 0}, {0, 1}, {1, 1}, {1, 2}, {2, 2}, {3, 3}};
    
    ASSERT_TRUE(solver.validate_path(path, agent));
} 