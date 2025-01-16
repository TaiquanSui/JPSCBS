#include <gtest/gtest.h>
#include "../../../src/algorithm/jps/JPS.h"

class JPSTest : public ::testing::Test {
protected:
    std::vector<std::vector<int>> grid;
    JPSState state;
    
    void SetUp() override {
        grid = {
            {0, 0, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 0, 0},
            {0, 0, 0, 0}
        };
    }
};

TEST_F(JPSTest, SimplePathFinding) {
    Vertex start{0, 0};
    Vertex goal{3, 3};
    
    auto result = jump_point_search(start, goal, grid, state);
    ASSERT_FALSE(result.path.empty());
    ASSERT_EQ(result.path.front(), start);
    ASSERT_EQ(result.path.back(), goal);
}

TEST_F(JPSTest, JumpPointsExist) {
    Vertex start{0, 0};
    Vertex goal{3, 3};
    
    auto result = jump_point_search(start, goal, grid, state);
    ASSERT_FALSE(result.jump_points.empty());
    ASSERT_EQ(result.jump_points.front(), start);
    ASSERT_EQ(result.jump_points.back(), goal);
}

TEST_F(JPSTest, NoPath) {
    // 创建一个被障碍物包围的目标点
    grid[2][2] = 1;
    grid[2][3] = 1;
    grid[3][2] = 1;
    grid[3][3] = 1;
    
    Vertex start{0, 0};
    Vertex goal{3, 3};
    
    auto result = jump_point_search(start, goal, grid, state);
    ASSERT_TRUE(result.path.empty());
} 