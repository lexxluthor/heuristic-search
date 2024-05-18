#ifndef NODE_H
#define NODE_H

#include <unordered_set>
#include <tuple>
#include <cstddef>
#include "gl_const.h"

struct Node
{
    int     i, j; //grid cell coordinates
    int     g; //f-, g- and h-values of the search node
    double  F, H;
    Node    *parent; //backpointer to the predecessor node (e.g. the node which g-value was used to set the g-velue of the current node)
    int     conflictsCount;

    Node(int x = 0, int y = 0, Node *p = nullptr, int g_ = 0, double H_ = 0, int ConflictsCount = 0) {
        i = x;
        j = y;
        parent = p;
        g = g_;
        H = H_;
        F = (g_ == CN_INFINITY) ? g_ : g_ + H_;
        conflictsCount = ConflictsCount;
    }

    bool operator== (const Node &other) const {
        return i == other.i && j == other.j;
    }
    bool operator!= (const Node &other) const {
        return i != other.i || j != other.j;
    }
    bool operator< (const Node &other) const {
        return std::tuple<int, int, int, int>(F, -g, i, j) <
            std::tuple<int, int, int, int>(other.F, -other.g, other.i, other.j);
    }

    virtual int convolution(int width, int height, bool withTime = false) const {
        int res = withTime ? width * height * g : 0;
        return res + i * width + j;
    }

    virtual int getHC() const {
        return 0;
    }
};

static size_t elegantPairing(const int &k1, const int &k2) 
{
    return k1 < k2 ? k2 * k2 + k1 : k1 * k1 + k1 + k2;
}

template <>
struct std::hash<Node>
{
    size_t operator()(const Node& node) const {
        return elegantPairing(node.i, node.j);
    }
};

template <>
struct std::hash<std::pair<Node, Node>>
{
    size_t operator()(const pair<Node, Node>& pair) const {
        return elegantPairing(std::hash<Node>{}(pair.first), std::hash<Node>{}(pair.second));
    }
};

#endif 
