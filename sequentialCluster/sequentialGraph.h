
/**
 * @file sequentialGraph.h
 * @author SJchan (13560469332@163.com)
 * @brief
 * @version 0.1
 * @date 2021-04-14
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <memory>
#include <set>
#include <stack>
#include <vector>

// #include "../evaluate.h"
#include "../include/utility.h"
#include "sequentialElement.h"

class sequentialElement;
class sequentialVertex;
class sequentialArc;

class sequentialArc {
public:
    sequentialArc();
    sequentialArc(sequentialVertex* src, sequentialVertex* sink);

    // copy constructor
    sequentialArc(const sequentialArc& obj);

    ~sequentialArc();

    void set_idx(uint id) { _idx = id; }
    void add_logic_cells(cell* c) { _logic_cells.push_back(c); }
    std::vector<cell*>& get_logic_cells() { return _logic_cells; }

    uint get_idx() const { return _idx; }
    sequentialVertex* get_src() const { return _src; }
    sequentialVertex* get_sink() const { return _sink; }

private:
    uint _idx;
    sequentialVertex* _src;
    sequentialVertex* _sink;
    std::vector<cell*> _logic_cells;
};

class sequentialVertex {
public:
    sequentialVertex();
    sequentialVertex(sequentialElement* node);

    // copy constructor
    sequentialVertex(const sequentialVertex& obj);

    ~sequentialVertex();

    uint get_idx() const { return _idx; }
    std::vector<uint> get_connect_vertexes() const { return _connect_vertexes; }

    sequentialElement* get_vertex() const { return _node; }

    void add_connect_vertexes(uint id) { _connect_vertexes.push_back(id); }

    void add_src_edges(sequentialArc* src_edge) { _src_edges.push_back(src_edge); }
    void add_sink_edges(sequentialArc* sink_edge) { _sink_edges.push_back(sink_edge); }

    void add_from_vertexes(uint from) { _from_vertexes.insert(from); }
    void add_to_vertexes(uint to) { _to_vertexes.insert(to); }

    void set_idx(uint id) { _idx = id; }
    void set_start() { _is_start = 1; }
    void set_end() { _is_end = 1; }
    void set_const() { _is_const = 1; }

    std::vector<sequentialArc*>& get_src_edges() { return _src_edges; }
    std::vector<sequentialArc*>& get_sink_edges() { return _sink_edges; }

    std::unordered_set<uint>& get_from_vertexes() { return _from_vertexes; }
    std::unordered_set<uint>& get_to_vertexes() { return _to_vertexes; }

    std::unordered_set<uint>& get_connect_vertexes() { return _to_vertexes; }

    unsigned isStart() const { return _is_start; }
    unsigned isEnd() const { return _is_end; }

private:
    uint _idx;
    sequentialElement* _node;
    unsigned _is_start : 1;                   // The vertex is start sequentialElement.
    unsigned _is_end : 1;                     // The vertex is end sequentialElement.
    unsigned _is_const : 1;                   // The vertex is const.
    std::vector<sequentialArc*> _src_edges;   // The sequentialArc sourced from the vertex.
    std::vector<sequentialArc*> _sink_edges;  // The sequentialArc sinked to the vertex.

    std::vector<uint> _connect_vertexes;  // In order to judge the vertex direct connection

    std::unordered_set<uint> _from_vertexes;  // Record the from vertexes.
    std::unordered_set<uint> _to_vertexes;    // Record the to vertexes.
};

class sequentialGraph {
public:
    sequentialGraph();

    // copy constructor
    sequentialGraph(const sequentialGraph& obj);

    ~sequentialGraph();

    void add_vertex(sequentialVertex* v);
    void add_edge(sequentialArc* e);

    void add_start_vertex(sequentialVertex* start_vertex);
    void add_end_vertex(sequentialVertex* end_vertex);
    void add_const_vertex(sequentialVertex* const_vertex);

    void updateHop();
    void hopForwardDFS(std::stack<sequentialVertex*>& stack);
    void hopBackwardDFS(std::stack<sequentialVertex*>& stack);

    void initDegree();

    bool findRing(sequentialVertex* vertex_1, sequentialVertex* vertex_2,
                  std::unordered_map<uint, int> vertex_to_degree);

    void updateCoordMapping();

    sequentialVertex* findLeastCommonAncestor(sequentialVertex* vertex_1, sequentialVertex* vertex_2);
    uint leastCommonAncestorDFS(std::stack<sequentialVertex*>& stack, std::unordered_set<uint>& search_set);

    std::vector<sequentialVertex*> get_vertexes() const { return _vertexes; }
    std::vector<sequentialArc*> get_edges() const { return _arcs; }
    std::set<sequentialVertex*> get_start_vertexes() const { return _start_vertexes; }
    std::set<sequentialVertex*> get_end_vertexes() const { return _end_vertexes; }
    std::set<sequentialVertex*> get_const_vertexes() const { return _const_vertexes; }

private:
    std::vector<sequentialVertex*> _vertexes;
    std::vector<sequentialArc*> _arcs;

    std::set<sequentialVertex*> _start_vertexes;
    std::set<sequentialVertex*> _end_vertexes;
    std::set<sequentialVertex*> _const_vertexes;

    // for coordinate mapping
    std::vector<int> _x_coords;
    std::vector<int> _y_coords;
    std::unordered_multimap<int, uint> _x_to_vertexs;
    std::unordered_multimap<int, uint> _y_to_vertexs;

    // for topological sorting
    std::unordered_map<uint, int> _vertex_to_degree;
};
