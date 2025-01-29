#include <gtest/gtest.h>
#include "../../../src/algorithm/jpscbs/JPSCBS.h"
#include "../../../src/algorithm/utilities/Utility.h"
#include "../../../src/algorithm/utilities/Log.h"
#include <sstream>
#include <chrono>

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
        solver.set_time_limit(0.1);
    }

    double measureExecutionTime(const std::function<void()>& func) {
        auto start = std::chrono::high_resolution_clock::now();
        func();
        auto end = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    }
};

TEST_F(JPSCBSTest, SimpleNoConflictPath) {
    // 两个智能体在对角线上,不会产生冲突
    std::vector<Agent> agents = {
        Agent{0, Vertex{0, 0}, Vertex{0, 2}},
        Agent{1, Vertex{3, 3}, Vertex{3, 1}}
    };

    double execution_time = measureExecutionTime([&]() {
        auto paths = solver.solve(agents, grid);
        ASSERT_FALSE(paths.empty());
        ASSERT_EQ(paths.size(), 2);
        
        // 验证路径长度
        ASSERT_EQ(paths[0].size(), 3); // (0,0)->(0,1)->(0,2)
        ASSERT_EQ(paths[1].size(), 3); // (3,3)->(3,2)->(3,1)
    });
    
    std::cout << "SimpleNoConflictPath execution time: " << execution_time << "ms" << std::endl;
}

TEST_F(JPSCBSTest, ConflictAvoidance) {
    std::vector<Agent> agents = {
        Agent{0, Vertex{0, 0}, Vertex{3, 3}},
        Agent{1, Vertex{0, 3}, Vertex{3, 0}}
    };

    double execution_time = measureExecutionTime([&]() {
        auto paths = solver.solve(agents, grid);
        ASSERT_FALSE(paths.empty());
        ASSERT_EQ(paths.size(), 2);
        
        // 验证路径不相交
        for(size_t i = 0; i < paths[0].size(); i++) {
            for(size_t j = 0; j < paths[1].size(); j++) {
                ASSERT_FALSE(paths[0][i] == paths[1][j] && i == j);
            }
        }
    });
    
    std::cout << "ConflictAvoidance execution time: " << execution_time << "ms" << std::endl;
}

TEST_F(JPSCBSTest, MultiAgentScenario) {
    // 测试3个智能体的场景
    grid = {
        {0, 0, 0, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 0, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 0, 0, 0}
    };
    
    std::vector<Agent> agents = {
        Agent{0, Vertex{0, 0}, Vertex{4, 4}},
        Agent{1, Vertex{0, 4}, Vertex{4, 0}},
        Agent{2, Vertex{2, 2}, Vertex{2, 4}}
    };

    double execution_time = measureExecutionTime([&]() {
        auto paths = solver.solve(agents, grid);
        ASSERT_FALSE(paths.empty());
        ASSERT_EQ(paths.size(), 3);
        
        // 验证所有路径都到达目标点
        ASSERT_EQ(paths[0].back(), Vertex(4,4));
        ASSERT_EQ(paths[1].back(), Vertex(4,0));
        ASSERT_EQ(paths[2].back(), Vertex(2,4));
    });
    
    std::cout << "MultiAgentScenario execution time: " << execution_time << "ms" << std::endl;
}

TEST_F(JPSCBSTest, SymmetricIntervalHandling) {
    // 创建一个有对称区间的场景
    grid = {
        {0, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0}
    };
    
    std::vector<Agent> agents = {
        Agent{0, Vertex{0, 0}, Vertex{3, 3}},
        Agent{1, Vertex{1, 1}, Vertex{2, 2}}
    };

    double execution_time = measureExecutionTime([&]() {
        auto paths = solver.solve(agents, grid);
        ASSERT_FALSE(paths.empty());
        ASSERT_EQ(paths.size(), 2);
        
        // 验证路径合法性
        for(const auto& path : paths) {
            for(size_t i = 0; i < path.size() - 1; i++) {
                // 检查相邻点之间的距离是否合法
                double dist = utils::octileDistance(path[i], path[i+1]);
                ASSERT_LE(dist, 1.5); // 允许对角线移动
            }
        }
    });
    
    std::cout << "SymmetricIntervalHandling execution time: " << execution_time << "ms" << std::endl;
}
