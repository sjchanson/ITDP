#include "sequentialGraph.h"

sequentialArc::sequentialArc() : _idx(UINT_MAX), _src(nullptr), _sink(nullptr) {}

sequentialArc::sequentialArc(sequentialVertex* src, sequentialVertex* sink) : sequentialArc() {
    _src = src;
    _sink = sink;
}

sequentialArc::~sequentialArc() {
    _src = nullptr;
    _sink = nullptr;
    for (int i = 0; i < _logic_cells.size(); i++) {
        delete _logic_cells[i];
    }
}

sequentialVertex::sequentialVertex() : _idx(UINT_MAX), _node(nullptr), _is_start(0), _is_end(0), _is_const(0) {}

sequentialVertex::sequentialVertex(sequentialElement* node) : sequentialVertex() { _node = node; }

sequentialVertex::~sequentialVertex() {
    _node = nullptr;
    _is_start = 0;
    _is_end = 0;
    _is_const = 0;
    for (int i = 0; i < _src_edges.size(); i++) {
        delete _src_edges[i];
    }
    for (int i = 0; i < _sink_edges.size(); i++) {
        delete _sink_edges[i];
    }
}

sequentialGraph::sequentialGraph() = default;

sequentialGraph::~sequentialGraph() {
    for (int i = 0; i < _vertexes.size(); i++) {
        delete _vertexes[i];
    }
    for (int i = 0; i < _arcs.size(); i++) {
        delete _arcs[i];
    }
}

void sequentialGraph::add_vertex(sequentialVertex* v) {
    if (v->get_vertex()->isPi()) {
        v->set_start();
        add_start_vertex(v);
    } else if (v->get_vertex()->isPo()) {
        v->set_end();
        add_end_vertex(v);
    } else {
        v->set_const();
        add_const_vertex(v);
    }
    _vertexes.push_back(v);
}

void sequentialGraph::add_edge(sequentialArc* e) { _arcs.push_back(e); }

void sequentialGraph::add_start_vertex(sequentialVertex* start_vertex) { _start_vertexes.insert(start_vertex); }

void sequentialGraph::add_end_vertex(sequentialVertex* end_vertex) { _end_vertexes.insert(end_vertex); }

void sequentialGraph::add_const_vertex(sequentialVertex* const_vertex) { _const_vertexes.insert(const_vertex); }