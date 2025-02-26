#ifndef CONFLICTAVOIDANCETABLE_H
#define CONFLICTAVOIDANCETABLE_H

#include <unordered_map>
#include <iostream>
#include "../basic/Vertex.h"

// 哈希函数，使 std::pair<Vertex, int> 可用于 unordered_map
struct PairHash {
    template <class T2>
    std::size_t operator()(const std::pair<Vertex, T2>& p) const {
        auto hash1 = std::hash<Vertex>{}(p.first);
        auto hash2 = std::hash<T2>{}(p.second);
        return hash1 ^ (hash2 << 1);
    }
};

// 边的哈希函数
struct EdgeHash {
    std::size_t operator()(const std::pair<std::pair<Vertex, Vertex>, int>& edge) const {
        auto hash1 = std::hash<Vertex>{}(edge.first.first);
        auto hash2 = std::hash<Vertex>{}(edge.first.second);
        auto hash3 = std::hash<int>{}(edge.second);
        return hash1 ^ (hash2 << 1) ^ (hash3 << 2);
    }
};

class ConflictAvoidanceTable {
private:
    std::unordered_map<std::pair<Vertex, int>, int, PairHash> vertex_cat;  // (location, time) -> conflict count
    std::unordered_map<std::pair<std::pair<Vertex, Vertex>, int>, int, EdgeHash> edge_cat;  // ((v1, v2), time) -> conflict count
    std::unordered_map<Vertex, std::pair<int, int>> goal_occupancy;  // location -> (time, count)

public:
    // 构造函数
    ConflictAvoidanceTable() = default;

    // 添加顶点约束
    void addVertexConstraint(Vertex location, int time);

    // 添加边约束
    void addEdgeConstraint(Vertex v1, Vertex v2, int time);

    // 添加终点约束
    void addGoalConstraint(Vertex location, int time);

    // 移除顶点约束
    void removeVertexConstraint(Vertex location, int time);

    // 移除边约束
    void removeEdgeConstraint(Vertex v1, Vertex v2, int time);

    // 获取冲突次数（包括顶点和边约束）
    int getConflictCount(Vertex current, Vertex next, int time) const;

    // 清空 CAT
    void clear();

    // 打印 CAT（调试用）
    void printCAT() const;
};

#endif
