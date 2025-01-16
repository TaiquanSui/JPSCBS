#include <gtest/gtest.h>
#include "../../../src/algorithm/astar/AStar.h"

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

TEST_F(AStarTest, SimplePathFinding) {
    Vertex start{0, 0};
    Vertex goal{3, 3};
    
    auto path = a_star(start, goal, grid);
    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.front(), start);
    ASSERT_EQ(path.back(), goal);
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
        {0, Vertex{1, 1}, 2}  // Forbid passing through (1,1) at time 2
    };
    
    auto path = a_star(0, start, goal, grid, constraints);
    ASSERT_FALSE(path.empty());
    
    // Verify path doesn't violate constraints
    if (path.size() > 2) {
        ASSERT_NE(path[2], Vertex(1, 1));
    }
} 