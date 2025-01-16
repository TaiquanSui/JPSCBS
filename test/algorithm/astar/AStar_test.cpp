#include <gtest/gtest.h>
#include "../../../src/algorithm/astar/AStar.h"

class AStarTest : public ::testing::Test {
protected:
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

TEST_F(AStarTest, SimplePathFinding) {
    Vertex start{0, 0};
    Vertex goal{3, 3};
    
    auto path = a_star(start, goal, grid);
    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.front(), start);
    ASSERT_EQ(path.back(), goal);
}

TEST_F(AStarTest, NoPath) {
    // 创建一个被障碍物包围的目标点
    grid[2][2] = 1;
    grid[2][3] = 1;
    grid[3][2] = 1;
    grid[3][3] = 1;
    
    Vertex start{0, 0};
    Vertex goal{3, 3};
    
    auto path = a_star(start, goal, grid);
    ASSERT_TRUE(path.empty());
}

TEST_F(AStarTest, PathWithConstraints) {
    Vertex start{0, 0};
    Vertex goal{3, 3};
    std::vector<Constraint> constraints = {
        {0, Vertex{1, 1}, 2}  // 在时间2禁止经过(1,1)
    };
    
    auto path = a_star(0, start, goal, grid, constraints);
    ASSERT_FALSE(path.empty());
    
    // 验证路径不违反约束
    if (path.size() > 2) {
        ASSERT_NE(path[2], Vertex(1, 1));
    }
} 