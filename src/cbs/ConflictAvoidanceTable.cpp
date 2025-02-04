#include "ConflictAvoidanceTable.h"

// 添加冲突
void ConflictAvoidanceTable::addConstraint(Vertex location, int time) {
    std::pair<Vertex, int> key = {location, time};
    cat[key]++; // 记录该位置在该时间点的被占用次数
}

// 添加终点约束
void ConflictAvoidanceTable::addGoalConstraint(Vertex location, int time) {
    goal_occupancy[location] = {time, goal_occupancy[location].second + 1};
}

// 移除冲突
void ConflictAvoidanceTable::removeConstraint(Vertex location, int time) {
    std::pair<Vertex, int> key = {location, time};
    if (cat.find(key) != cat.end()) {
        cat[key]--;
        if (cat[key] == 0) {
            cat.erase(key); // 如果次数降到 0，删除该键
        }
    }
}

// 查询冲突次数
int ConflictAvoidanceTable::getConflictCount(Vertex location, int time) const {
    int count = 0;
    
    // 检查普通约束
    std::pair<Vertex, int> key = {location, time};
    auto it = cat.find(key);
    if (it != cat.end()) {
        count += it->second;
    }
    
    // 检查终点约束
    auto goal_it = goal_occupancy.find(location);
    if (goal_it != goal_occupancy.end() && time >= goal_it->second.first) {
        count += goal_it->second.second;
    }
    
    return count;
}

// 清空 CAT
void ConflictAvoidanceTable::clear() {
    cat.clear();
    goal_occupancy.clear();
}

// 打印 CAT 状态（用于调试）
void ConflictAvoidanceTable::printCAT() const {
    std::cout << "Conflict Avoidance Table:\n";
    std::cout << "Regular constraints:\n";
    for (const auto& entry : cat) {
        std::cout << "Location: (" << entry.first.first.x << "," << entry.first.first.y 
                  << "), Time: " << entry.first.second
                  << ", Count: " << entry.second << "\n";
    }
    
    std::cout << "\nGoal constraints:\n";
    for (const auto& entry : goal_occupancy) {
        std::cout << "Location: (" << entry.first.x << "," << entry.first.y 
                  << "), From Time: " << entry.second.first
                  << ", Count: " << entry.second.second << "\n";
    }
}
