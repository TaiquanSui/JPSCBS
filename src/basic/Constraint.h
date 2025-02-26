#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "Vertex.h"

struct Constraint {
    enum Type { 
        VERTEX,         // 顶点约束（包括跟随冲突）
        EDGE           // 边约束（包括交换冲突和对角线交叉冲突）
    } type;
    
    int agent;
    int time;
    
    union {
        // 用于顶点约束
        struct {
            Vertex vertex;
        } vertex_constraint;
        
        // 用于边约束
        struct {
            Vertex v1;
            Vertex v2;
        } edge_constraint;
    };

    // 顶点约束构造函数
    Constraint(int a, const Vertex& v, int t) 
        : type(VERTEX), agent(a), time(t) {
        vertex_constraint.vertex = v;
    }

    // 边约束构造函数
    Constraint(int a, const Vertex& from, const Vertex& to, int t) 
        : type(EDGE), agent(a), time(t) {
        edge_constraint.v1 = from;
        edge_constraint.v2 = to;
    }
};


#endif //CONSTRAINT_H
