#include <gtest/gtest.h>
#include "../../../src/algorithm/jps/JPS.h"

class JPSTest : public ::testing::Test {
protected:
    std::vector<std::vector<int>> grid;
    std::priority_queue<std::shared_ptr<AStarNode>, 
                       std::vector<std::shared_ptr<AStarNode>>, 
                       AStarNodeComparator> open_list;
    
    void SetUp() override {
        grid = {
            {0, 0, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 0, 0},
            {0, 0, 0, 0}
        };
        // 每个测试前清空 open_list
        while (!open_list.empty()) {
            open_list.pop();
        }
    }
};

TEST_F(JPSTest, SimplePathFinding) {
    Vertex start{0, 0};
    Vertex goal{3, 3};
    
    auto result = jump_point_search(start, goal, grid, open_list);
    ASSERT_FALSE(result.path.empty());
    ASSERT_EQ(result.path.front(), start);
    ASSERT_EQ(result.path.back(), goal);
}

TEST_F(JPSTest, JumpPointsExist) {
    Vertex start{0, 0};
    Vertex goal{3, 3};
    
    auto result = jump_point_search(start, goal, grid, open_list);
    ASSERT_FALSE(result.jump_points.empty());
    ASSERT_EQ(result.jump_points.front(), start);
    ASSERT_EQ(result.jump_points.back(), goal);
}

TEST_F(JPSTest, NoPath) {
    // Create a goal point surrounded by obstacles
    grid[2][2] = 1;
    grid[2][3] = 1;
    grid[3][2] = 1;
    grid[3][3] = 1;
    
    Vertex start{0, 0};
    Vertex goal{3, 3};
    
    auto result = jump_point_search(start, goal, grid, open_list);
    ASSERT_TRUE(result.path.empty());
}

TEST_F(JPSTest, StateReuse) {
    Vertex start{0, 0};
    Vertex goal{3, 3};
    
    // 第一次搜索
    auto result1 = jump_point_search(start, goal, grid, open_list);
    ASSERT_FALSE(result1.path.empty());
    
    // 第二次搜索
    auto result2 = jump_point_search(start, goal, grid, open_list);
    ASSERT_FALSE(result2.path.empty());
} 