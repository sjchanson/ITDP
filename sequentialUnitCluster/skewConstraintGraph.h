/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-04 13:57:50
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-07 11:59:34
 * @Description:
 */

#ifndef ITDP_SKEW_CONSTRAINT_GRAPH
#define ITDP_SKEW_CONSTRAINT_GRAPH

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "../include/common/logger.h"
#include "../include/common/parameter.h"
#include "sequentialElement.h"

namespace itdp {

class SequentialEdge;

class SequentialVertex {
public:
    SequentialVertex() = delete;
    SequentialVertex(SequentialElement* node);
    ~SequentialVertex() = default;

    // getter.
    std::string get_name() const { return _name; }
    SequentialElement* get_node() const { return _node; }

    bool isStart() const { return _is_start == 1; }
    bool isEnd() const { return _is_end == 1; }

    // setter.
    // special case need to set the set start/end.
    void set_start() { _is_start = 1; }
    void set_end() { _is_end = 1; }
    void add_src_edge(SequentialEdge* src_edge) { _src_edges.push_back(src_edge); }
    void add_sink_edge(SequentialEdge* sink_edge) { _sink_edges.push_back(sink_edge); }

private:
    std::string _name;
    SequentialElement* _node;
    unsigned _is_start : 1;
    unsigned _is_end : 1;
    std::vector<SequentialEdge*> _src_edges;
    std::vector<SequentialEdge*> _sink_edges;
    std::list<SequentialVertex*> _ancestors;    // Record all Vertexs that can reach this.
    std::list<SequentialVertex*> _descendants;  // Record all Vertexs that can be reached at this.
};

class SequentialEdge {
public:
    SequentialEdge() = delete;
    SequentialEdge(SequentialVertex* src, SequentialVertex* sink);
    ~SequentialEdge() = default;

    // getter.
    std::string get_name() const { return _name; }
    SequentialVertex* get_src_vertex() const { return _src; }  // the vertex need to be modified.
    SequentialVertex* get_sink_vertex() const { return _sink; }
    double get_setup_skew() const { return _setup_skew; }

    // setter.

private:
    std::string _name;
    SequentialVertex* _src;
    SequentialVertex* _sink;
    double _setup_skew;
};

class SkewConstraintGraph {
public:
    SkewConstraintGraph() = delete;
    SkewConstraintGraph(std::string name);
    ~SkewConstraintGraph();

    // getter.
    int get_sequential_vertexes_size() const { return _sequential_vertexes.size(); }
    int get_sequential_edges_size() const { return _sequential_edges.size(); }

    SequentialVertex* get_existent_vertex(std::string name);  // the return value need to modify.

    bool isExistentEdge(std::string src_name, std::string sink_name);

    // setter.
    void add_sequential_vertex(SequentialVertex* v);
    void add_sequential_edge(SequentialEdge* e);

private:
    std::string _name;
    Logger* _log;
    std::shared_ptr<Parameter> _parameter;

    // SeqeuntialVertex find and delete operations are required.
    std::unordered_map<std::string, SequentialVertex*> _sequential_vertexes;
    std::unordered_map<std::string, SequentialEdge*> _sequential_edges;
    std::vector<SequentialVertex*> _start_vertexes;
    std::vector<SequentialVertex*> _end_vertexes;
};
inline void SkewConstraintGraph::add_sequential_vertex(SequentialVertex* v) {
    if (v->isStart()) {
        _start_vertexes.push_back(v);
    } else if (v->isEnd()) {
        _end_vertexes.push_back(v);
    }
    _sequential_vertexes.emplace(v->get_name(), v);
}
inline void SkewConstraintGraph::add_sequential_edge(SequentialEdge* e) { _sequential_edges.emplace(e->get_name(), e); }

}  // namespace itdp

#endif
