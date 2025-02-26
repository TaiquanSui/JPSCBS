#include "ConflictAvoidanceTable.h"

// 添加顶点约束
void ConflictAvoidanceTable::addVertexConstraint(Vertex location, int time) {
    std::pair<Vertex, int> key = {location, time};
    vertex_cat[key]++;
}

// 添加边约束
void ConflictAvoidanceTable::addEdgeConstraint(Vertex v1, Vertex v2, int time) {
    std::pair<std::pair<Vertex, Vertex>, int> key = {{v1, v2}, time};
    edge_cat[key]++;
}

// 添加终点约束
void ConflictAvoidanceTable::addGoalConstraint(Vertex location, int time) {
    goal_occupancy[location] = {time, goal_occupancy[location].second + 1};
}

// 移除顶点约束
void ConflictAvoidanceTable::removeVertexConstraint(Vertex location, int time) {
    std::pair<Vertex, int> key = {location, time};
    if (vertex_cat.find(key) != vertex_cat.end()) {
        vertex_cat[key]--;
        if (vertex_cat[key] == 0) {
            vertex_cat.erase(key);
        }
    }
}

// 移除边约束
void ConflictAvoidanceTable::removeEdgeConstraint(Vertex v1, Vertex v2, int time) {
    std::pair<std::pair<Vertex, Vertex>, int> key = {{v1, v2}, time};
    if (edge_cat.find(key) != edge_cat.end()) {
        edge_cat[key]--;
        if (edge_cat[key] == 0) {
            edge_cat.erase(key);
        }
    }
}

// 获取冲突次数
int ConflictAvoidanceTable::getConflictCount(Vertex current, Vertex next, int time) const {
    int count = 0;
    
    // 检查顶点约束
    std::pair<Vertex, int> vertex_key = {next, time};
    auto vertex_it = vertex_cat.find(vertex_key);
    if (vertex_it != vertex_cat.end()) {
        count += vertex_it->second;
    }
    
    // 检查边约束
    std::pair<std::pair<Vertex, Vertex>, int> edge_key = {{current, next}, time};
    auto edge_it = edge_cat.find(edge_key);
    if (edge_it != edge_cat.end()) {
        count += edge_it->second;
    }
    
    // 检查终点约束
    auto goal_it = goal_occupancy.find(next);
    if (goal_it != goal_occupancy.end() && time >= goal_it->second.first) {
        count += goal_it->second.second;
    }
    
    return count;
}

// 清空 CAT
void ConflictAvoidanceTable::clear() {
    vertex_cat.clear();
    edge_cat.clear();
    goal_occupancy.clear();
}

// 打印 CAT 状态（用于调试）
void ConflictAvoidanceTable::printCAT() const {
    std::cout << "Conflict Avoidance Table:\n";
    std::cout << "Vertex constraints:\n";
    for (const auto& entry : vertex_cat) {
        std::cout << "Location: (" << entry.first.first.x << "," << entry.first.first.y 
                  << "), Time: " << entry.first.second
                  << ", Count: " << entry.second << "\n";
    }
    
    std::cout << "\nEdge constraints:\n";
    for (const auto& entry : edge_cat) {
        std::cout << "Edge: (" << entry.first.first.first.x << "," << entry.first.first.first.y 
                  << ") -> (" << entry.first.first.second.x << "," << entry.first.first.second.y
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
