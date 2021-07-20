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

#include <iostream>
#include <queue>
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

class ClusterVertex {
public:
    ClusterVertex();
    explicit ClusterVertex(sequentialFlipFlop *flipflop);
    explicit ClusterVertex(DBU x, DBU y, string name);
    ~ClusterVertex() = default;

    bool isLegal();
    // Getter
    Point<DBU> *get_point() const { return _point; }
    string get_name() const { return _name; }
    int get_level() const { return _level; }

    // Setter
    void set_point(Point<DBU> *point) { _point = point; }
    void set_skew(const double skew) { _skew = skew; }
    void set_name(const string name) { _name = name; }
    void set_level(const int level) { _level = level; }

    void add_src_edges(ctsEdge *e) { _src_edges.push_back(e); }
    void add_sink_edges(ctsEdge *e) { _sink_edges.push_back(e); }

    vector<ctsEdge *> get_src_edges() const { return _src_edges; }
    vector<ctsEdge *> get_sink_edges() const { return _sink_edges; }

private:
    string _name;
    int _level;
    Point<DBU> *_point;  // location.
    vector<ctsEdge *> _src_edges;
    vector<ctsEdge *> _sink_edges;
    double _skew;
};

class ctsEdge {
public:
    ctsEdge();
    ctsEdge(ClusterVertex *src, ClusterVertex *sink);
    ~ctsEdge();

    ClusterVertex *get_src() const { return _src; }
    ClusterVertex *get_sink() const { return _sink; }

private:
    ClusterVertex *_src;
    ClusterVertex *_sink;
};

class ClusterVertexPair {
public:
    ClusterVertexPair();
    ClusterVertexPair(ClusterVertex *v1, ClusterVertex *v2);
    ~ClusterVertexPair() = default;

    ClusterVertex *vertex_1;
    ClusterVertex *vertex_2;
    DBU distance;
};

class ctsSingleClus {
public:
    ctsSingleClus();
    ctsSingleClus(
        std::unordered_set<sequentialFlipFlop *, sequentialFlipFlop::basePtrHash, sequentialFlipFlop::basePtrEqual>
            flipflops,
        string name);
    ~ctsSingleClus();

    string get_name() const { return _name; }

    std::vector<ClusterVertex *> get_binary_tree() const { return _binary_tree; }
    void analyseSinkRelationship(ofstream &sheet_stream);
    void constructPerfectBinaryTree();

private:
    string _name;
    int _transition_point_cnt;
    int _top_level;

    std::vector<ClusterVertex *> _binary_tree;
    std::unordered_set<sequentialFlipFlop *, sequentialFlipFlop::basePtrHash, sequentialFlipFlop::basePtrEqual>
        _origin_flipflops;
    std::vector<ClusterVertexPair> _binary_pair;

    std::list<sequentialFlipFlop *> _binary_flipflops;  // for those which paticipate in binary tree.
    ClusterVertex *_root_vertex;
    std::vector<ClusterVertex *> _vertexes;  // all Vertices with connection within the cluster.

    std::vector<ctsEdge *> _edges;  // all Edge connection.

    void analyseBFS(std::queue<ClusterVertex *> &queue);
    ClusterVertex *buildVertexes(sequentialFlipFlop *from, sequentialFlipFlop *to, int transition_cnt);
    std::list<ClusterVertex *> updateUpLevelVertexes(std::list<ClusterVertex *> cur_vertexes, int level);
};

class ctsBase {
public:
    ctsBase() = delete;
    ctsBase(std::unordered_set<sequentialCluster *, baseHash, baseEqual> clusters);
    ~ctsBase();

    std::vector<ctsSingleClus *> get_sub_clusters() const { return _sub_clusters; }

private:
    std::vector<ctsSingleClus *> _sub_clusters;
    std::unordered_set<sequentialCluster *, baseHash, baseEqual> _clusters;

    void init();  // clusters -> binary tree.
};
