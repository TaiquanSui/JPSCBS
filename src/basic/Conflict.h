#ifndef CONFLICT_H
#define CONFLICT_H

#include "Vertex.h"

struct Conflict {
    enum Type { 
        VERTEX,         // 顶点冲突（包括跟随冲突）
        EDGE           // 边冲突（包括交换冲突和对角线交叉冲突）
    } type;
    int agent1;
    int agent2;
    int time;
    
    union {
        // 用于顶点冲突
        struct {
            Vertex vertex;
        } vertex_conflict;
        
        // 用于边冲突
        struct {
            Vertex v1;
            Vertex v2;
        } edge_conflict;
    };

    // 顶点冲突构造函数
    Conflict(int a1, int a2, const Vertex& v, int t) 
        : type(VERTEX), agent1(a1), agent2(a2), time(t) {
        vertex_conflict.vertex = v;
    }

    // 边冲突构造函数
    Conflict(int a1, int a2, const Vertex& from, const Vertex& to, int t) 
        : type(EDGE), agent1(a1), agent2(a2), time(t) {
        edge_conflict.v1 = from;
        edge_conflict.v2 = to;
    }
};

#endif //CONFLICT_H
