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
    if (v->get_vertex()->isPi() || v->get_vertex()->isFFPi()) {
        v->set_start();
        add_start_vertex(v);
    } else if (v->get_vertex()->isPo() || v->get_vertex()->isFFPo()) {
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

void sequentialGraph::updateHop() {
    stack<sequentialVertex*> vertex_stack;
    double begin, end;
    begin = microtime();
    for (auto& pi : _start_vertexes) {
        if (!vertex_stack.empty()) {
            std::cout << "ERROR in Stack" << std::endl;
            exit(1);
        }
        vertex_stack.push(pi);
        hopForwardDFS(vertex_stack);
    }
    end = microtime();
    std::cout << "DFS Forward Timing : " << end - begin << "'s" << std::endl;

    begin = microtime();
    for (auto& po : _end_vertexes) {
        if (!vertex_stack.empty()) {
            std::cout << "ERROR in Stack" << std::endl;
            exit(1);
        }
        vertex_stack.push(po);
        hopBackwardDFS(vertex_stack);
    }
    end = microtime();
    std::cout << "DFS Backward Timing : " << end - begin << "'s" << std::endl;
}

void sequentialGraph::hopForwardDFS(std::stack<sequentialVertex*>& stack) {
    auto cur_v = stack.top();

    if (cur_v->isEnd()) {
        stack.pop();
        return;
    }

    if (!cur_v->get_to_vertexes().empty()) {
        stack.pop();
        return;
    }

    for (int i = 0; i < cur_v->get_sink_edges().size(); i++) {
        auto next_v = cur_v->get_sink_edges()[i]->get_sink();
        stack.push(next_v);
        hopForwardDFS(stack);

        // Add the vertex hop
        std::unordered_set<uint>::iterator iter;
        for (iter = next_v->get_to_vertexes().begin(); iter != next_v->get_to_vertexes().end(); iter++) {
            cur_v->add_to_vertexes(*iter);
        }
        cur_v->add_to_vertexes(next_v->get_idx());
    }
    stack.pop();
}

void sequentialGraph::hopBackwardDFS(std::stack<sequentialVertex*>& stack) {
    auto cur_v = stack.top();

    if (cur_v->isStart()) {
        stack.pop();
        return;
    }

    if (!cur_v->get_from_vertexes().empty()) {
        stack.pop();
        return;
    }

    for (int i = 0; i < cur_v->get_src_edges().size(); i++) {
        auto front_v = cur_v->get_src_edges()[i]->get_src();
        stack.push(front_v);
        hopBackwardDFS(stack);

        // Add the vertex hop
        std::unordered_set<uint>::iterator iter;
        for (iter = front_v->get_from_vertexes().begin(); iter != front_v->get_from_vertexes().end(); iter++) {
            cur_v->add_from_vertexes(*iter);
        }
        cur_v->add_from_vertexes(front_v->get_idx());
    }
    stack.pop();
}

bool sequentialGraph::findRing(sequentialVertex* vertex_1, sequentialVertex* vertex_2) {
    if (vertex_1->get_connect_vertexes().find(vertex_2->get_idx()) != vertex_1->get_connect_vertexes().end()) {
        return false;
    }

    if (vertex_1->get_to_vertexes().find(vertex_2->get_idx()) != vertex_1->get_to_vertexes().end()) {
        return true;
    }

    if (vertex_1->get_from_vertexes().find(vertex_2->get_idx()) != vertex_1->get_from_vertexes().end()) {
        return true;
    }

    return false;
}

sequentialVertex* sequentialGraph::findLeastCommonAncestor(sequentialVertex* vertex_1, sequentialVertex* vertex_2) {
    std::stack<sequentialVertex*> stack;
    uint ancestor_id = UINT_MAX;

    stack.push(vertex_1);
    ancestor_id = leastCommonAncestorDFS(stack, vertex_2->get_from_vertexes());

    if (ancestor_id != UINT_MAX) {
        return _vertexes[ancestor_id];
    }

    return nullptr;
}

uint sequentialGraph::leastCommonAncestorDFS(std::stack<sequentialVertex*>& stack,
                                             std::unordered_set<uint>& search_set) {
    sequentialVertex* vertex = stack.top();
    uint record = UINT_MAX;

    if (vertex->isStart()) {
        stack.pop();
        return record;
    }

    if (search_set.find(vertex->get_idx()) != search_set.end()) {
        stack.pop();
        return vertex->get_idx();
    }

    for (int i = 0; i < vertex->get_src_edges().size(); i++) {
        auto front_vertex = vertex->get_src_edges()[i]->get_src();
        stack.push(front_vertex);
        record = leastCommonAncestorDFS(stack, search_set);
        if (record != UINT_MAX) {
            break;
        }
    }
    stack.pop();
    return record;
}

void sequentialGraph::updateCoordMapping() {
    for (auto& vertex : _vertexes) {
        int x_coord = vertex->get_vertex()->get_coord().x;
        int y_coord = vertex->get_vertex()->get_coord().y;

        _x_coords.push_back(x_coord);
        _y_coords.push_back(y_coord);
        _x_to_vertexs.emplace(x_coord, vertex);
        _y_to_vertexs.emplace(y_coord, vertex);
    }
    std::sort(_x_coords.begin(), _y_coords.end());
    std::sort(_y_coords.begin(), _y_coords.end());
}



