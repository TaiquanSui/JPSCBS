#ifndef CONFLICTAVOIDANCETABLE_H
#define CONFLICTAVOIDANCETABLE_H

#include <unordered_map>
#include <iostream>
#include "../Vertex.h"

// 哈希函数，使 std::pair<Vertex, int> 可用于 unordered_map
struct PairHash {
    template <class T2>
    std::size_t operator()(const std::pair<Vertex, T2>& p) const {
        auto hash1 = std::hash<Vertex>{}(p.first);
        auto hash2 = std::hash<T2>{}(p.second);
        return hash1 ^ (hash2 << 1);
    }
};

class ConflictAvoidanceTable {
private:
    std::unordered_map<std::pair<Vertex, int>, int, PairHash> cat;  // (location, time) -> conflict count
    std::unordered_map<Vertex, std::pair<int, int>> goal_occupancy;  // location -> (time, count)

public:
    // 构造函数
    ConflictAvoidanceTable() = default;

    // 添加冲突
    void addConstraint(Vertex location, int time);

    // 添加终点约束
    void addGoalConstraint(Vertex location, int time);

    // 移除冲突
    void removeConstraint(Vertex location, int time);

    // 获取冲突次数
    int getConflictCount(Vertex location, int time) const;

    // 清空 CAT
    void clear();

    // 打印 CAT（调试用）
    void printCAT() const;
};

#endif
