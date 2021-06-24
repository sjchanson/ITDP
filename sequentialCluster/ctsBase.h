/**
 * @file ctsBase.h
 * @author SJchan (13560469332@163.com)
 * @brief
 * @version 0.1
 * @date 2021-06-23
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <stack>
#include <vector>

#include "sequentialOperator.h"

#pragma once

typedef long long DBU;

class ctsEdge;

// fun-2
template <class T>

class Point {
public:
    Point(T x, T y) : _x(x), _y(y) {}

    T getX() const { return _x; }
    T getY() const { return _y; }

    T computeDist(const Point<T> &other) const {
        T dx = (getX() > other.getX()) ? (getX() - other.getX()) : (other.getX() - getX());
        T dy = (getY() > other.getY()) ? (getY() - other.getY()) : (other.getY() - getY());

        return dx + dy;
    }

    T computeDistX(const Point<T> &other) const {
        T dx = (getX() > other.getX()) ? (getX() - other.getX()) : (other.getX() - getX());
        return dx;
    }

    T computeDistY(const Point<T> &other) const {
        T dy = (getY() > other.getY()) ? (getY() - other.getY()) : (other.getY() - getY());
        return dy;
    }

    bool operator<(const Point<T> &other) const {
        if (getX() != other.getX()) {
            return getX() < other.getX();
        } else {
            return getY() < other.getY();
        }
    }

    friend ostream &operator<<(ostream &out, const Point<T> &point) {
        out << "[(" << point.getX() << ", " << point.getY() << ")]";
        return out;
    }

    bool isUnLegal() const { return _x == -1 && _y == -1; }

private:
    T _x;
    T _y;
};

class ctsVertex {
public:
    ctsVertex();
    ctsVertex(DBU x, DBU y);
    ~ctsVertex() = default;

    void add_src_edges(ctsEdge *e) { _src_edges.push_back(e); }
    void add_sink_edges(ctsEdge *e) { _sink_edges.push_back(e); }

    vector<ctsEdge *> get_src_edges() const { return _src_edges; }
    vector<ctsEdge *> get_sink_edges() const { return _sink_edges; }
    int get_id() const { return _id; }

    void set_skew(double skew) { _skew = skew; }
    void set_id(int id) { _id = id; }

private:
    int _id;
    Point<DBU> _coord;  // location.
    vector<ctsEdge *> _src_edges;
    vector<ctsEdge *> _sink_edges;
    double _skew;
};

class ctsEdge {
public:
    ctsEdge() = delete;
    ctsEdge(ctsVertex *src, ctsVertex *sink);
    ~ctsEdge();

    ctsVertex *get_src() const { return _src; }
    ctsVertex *get_sink() const { return _sink; }

private:
    ctsVertex *_src;
    ctsVertex *_sink;
};

class ctsSingleClus {
public:
    ctsSingleClus();
    ctsSingleClus(std::unordered_set<sequentialFlipFlop *> flipflops);
    ~ctsSingleClus();

private:
    std::vector<ctsVertex *> _binary_tree;
    std::unordered_set<sequentialFlipFlop *> _origin_flipflops;

    std::vector<sequentialFlipFlop *> _binary_flipflops;  // for those which paticipate in binary tree.

    std::vector<ctsVertex *> _vertexes;  // all Vertices with connection within the cluster.
    std::vector<ctsEdge *> _edges;       // all Edge connection.

    std::set<std::pair<ctsVertex *, ctsVertex *> > _pairs;

    // three type of special case.
    std::vector<ctsVertex *> _special_forks;
    std::vector<ctsVertex *> _special_merges;
    std::vector<ctsVertex *> _special_cascades;

    void init();  // flipflops of a cluster -> binary tree.
    void splitStructre();
    void analyStructre();
    // uncomplete.
    void analyDFS(std::stack<ctsVertex *> &stack, std::map<int, bool> &is_visited);
};

class ctsBase {
public:
    ctsBase() = delete;
    ctsBase(std::unordered_set<sequentialCluster *, baseHash, baseEqual> clusters);
    ~ctsBase();

private:
    std::vector<ctsSingleClus *> _binary_tree;
    std::unordered_set<sequentialCluster *, baseHash, baseEqual> _clusters;

    void init();  // clusters -> binary tree.
};