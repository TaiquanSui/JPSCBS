#ifndef VERTEX_H
#define VERTEX_H

#include <functional>

struct Vertex {
    int x, y;

    explicit Vertex(int x = 0, int y = 0) : x(x), y(y) {}

    bool operator==(const Vertex& other) const {
        return x == other.x && y == other.y;
    }

    Vertex operator-(const Vertex& other) const {
        return Vertex(x - other.x, y - other.y);
    }

    bool operator!=(const Vertex& other) const {
        return !(*this == other);
    }
};

struct VertexHash {
    size_t operator()(const Vertex& p) const {
        return std::hash<int>()(p.x) ^ std::hash<int>()(p.y);
    }
};

#endif //VERTEX_H
