#include <gtest/gtest.h>
#include "../../../src/astar/AStar.h"

#include "src/utilities/GridUtility.h"

class AStarTest : public ::testing::Test {
protected:
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

TEST_F(AStarTest, SimplePathFinding_BasicAStar) {
    Vertex start{0, 0};
    Vertex goal{3, 3};
    
    auto path = a_star(start, goal, grid);
    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.front(), start);
    ASSERT_EQ(path.back(), goal);
    ASSERT_TRUE(utils::validatePath(path, start, goal, grid));
}

TEST_F(AStarTest, SimplePathFinding_ConstrainedAStar) {
    Vertex start{0, 0};
    Vertex goal{3, 3};
    
    auto path = a_star(0,start, goal, grid,{});
    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.front(), start);
    ASSERT_EQ(path.back(), goal);
    ASSERT_TRUE(utils::validatePath(path, start, goal, grid));
}

TEST_F(AStarTest, NoPath) {
    // Create a goal point surrounded by obstacles
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
        {0, Vertex{1, 0}, 1},
        {0, Vertex{0, 1}, 1},
        {0, Vertex{1, 0}, 2},
        {0, Vertex{0, 1}, 2},
        {0, Vertex{1, 0}, 3},
        {0, Vertex{0, 1}, 3}
    };
    
    auto path = a_star(0, start, goal, grid, constraints);
    ASSERT_FALSE(path.empty());
    
    // 验证路径不违反约束
    if (path.size() > 2) {
        ASSERT_NE(path[1], Vertex(1, 0));
        ASSERT_NE(path[1], Vertex(0, 1));
    }
    
    // 验证路径的合法性
    ASSERT_TRUE(utils::validatePath(path, start, goal, grid));
} 