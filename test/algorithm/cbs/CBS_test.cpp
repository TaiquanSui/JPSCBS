#include <gtest/gtest.h>
#include "../../../src/algorithm/cbs/CBS.h"

class CBSTest : public ::testing::Test {
protected:
    CBS solver{true};  // 启用优化
    std::vector<std::vector<int>> grid;
    
    void SetUp() override {
        grid = {
            {0, 0, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 0, 0},
            {0, 0, 0, 0}
        };
    }
};

TEST_F(CBSTest, SimplePathFinding) {
    std::vector<Agent> agents = {
        Agent{0, Vertex{0, 0}, Vertex{3, 3}},
        Agent{1, Vertex{0, 3}, Vertex{3, 0}}
    };
    
    auto paths = solver.solve(agents, grid);
    ASSERT_FALSE(paths.empty());
    ASSERT_EQ(paths.size(), 2);
}

TEST_F(CBSTest, NoSolution) {
    std::vector<Agent> agents = {
        Agent{0, Vertex{0, 0}, Vertex{0, 1}},
        Agent{1, Vertex{0, 1}, Vertex{0, 0}}
    };
    
    auto paths = solver.solve(agents, grid);
    ASSERT_TRUE(paths.empty());
}

TEST_F(CBSTest, ConflictResolution) {
    std::vector<Agent> agents = {
        Agent{0, Vertex{0, 0}, Vertex{2, 2}},
        Agent{1, Vertex{0, 2}, Vertex{2, 0}}
    };
    
    auto paths = solver.solve(agents, grid);
    ASSERT_FALSE(paths.empty());
    
    // 验证路径长度合理
    for (const auto& path : paths) {
        ASSERT_LE(path.size(), 5);  // 最长路径不应超过5步
    }
} 