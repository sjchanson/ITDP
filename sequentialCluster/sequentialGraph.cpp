#include "sequentialGraph.h"

sequentialArc::sequentialArc() : _idx(UINT_MAX), _src(nullptr), _sink(nullptr) {}

sequentialArc::sequentialArc(sequentialVertex* src, sequentialVertex* sink) : sequentialArc() {
    _src = src;
    _sink = sink;
}

// sequentialArc::sequentialArc(const sequentialArc& obj) {
//     _idx = obj._idx;
//     _logic_cells = obj._logic_cells;
//     _src = new sequentialVertex(*obj._src);
//     _sink = new sequentialVertex(*obj._sink);
// }

sequentialArc::~sequentialArc() {
    _src = nullptr;
    _sink = nullptr;
    for (int i = 0; i < _logic_cells.size(); i++) {
        delete _logic_cells[i];
    }
}

sequentialVertex::sequentialVertex() : _idx(UINT_MAX), _node(nullptr), _is_start(0), _is_end(0), _is_const(0) {}

sequentialVertex::sequentialVertex(sequentialBase* node) : sequentialVertex() { _node = node; }

// sequentialVertex::sequentialVertex(const sequentialVertex& obj) {
//     _idx = obj._idx;
//     _node = obj._node;
//     _is_start = obj._is_start;
//     _is_end = obj._is_end;
//     _is_const = obj._is_const;
//     _connect_vertexes = obj._connect_vertexes;
//     _from_vertexes = obj._from_vertexes;
//     _to_vertexes = obj._to_vertexes;

//     for (auto src_edge : obj._src_edges) {
//         sequentialArc* edge = new sequentialArc(*src_edge);
//         _src_edges.push_back(edge);
//     }
//     for (auto sink_edge : obj._sink_edges) {
//         sequentialArc* edge = new sequentialArc(*sink_edge);
//         _sink_edges.push_back(edge);
//     }
// }

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

bool sequentialVertex::operator==(const sequentialVertex*& v) {
    if (this->get_vertex()->get_name() == v->get_vertex()->get_name()) {
        return true;
    }
    return false;
}

sequentialPair::sequentialPair() : _vertex_1(nullptr), _vertex_2(nullptr), _distance(DBL_MAX) {}

sequentialPair::sequentialPair(sequentialVertex* v1, sequentialVertex* v2) : sequentialPair() {
    _vertex_1 = v1;
    _vertex_2 = v2;
}

bool sequentialPair::operator==(sequentialPair* p) {
    std::string v1_name = this->_vertex_1->get_vertex()->get_name();
    std::string v2_name = this->_vertex_2->get_vertex()->get_name();
    std::string p1_name = p->_vertex_1->get_vertex()->get_name();
    std::string p2_name = p->_vertex_2->get_vertex()->get_name();

    if (v1_name == p2_name && v2_name == p1_name) {
        return true;
    }

    if (v1_name == p1_name && v2_name == p2_name) {
        return true;
    }
    return false;
}

/**
 * @brief find whether is the looking for vertex in pair, if true, return the other vertex.
 *
 * @param vertex_1
 * @param another_vertex
 * @return true
 * @return false
 */
bool sequentialPair::findAnotherVetex(sequentialVertex* vertex_1, sequentialVertex*& another_vertex) {
    if (vertex_1 == _vertex_1) {
        another_vertex = _vertex_2;
        return true;
    }

    if (vertex_1 == _vertex_2) {
        another_vertex = _vertex_1;
        return true;
    }

    another_vertex = nullptr;
    return false;
}

sequentialGraph::sequentialGraph(Logger* log, int side_length) {
    _log = log;
    _side_length = side_length;
}

// sequentialGraph::sequentialGraph(const sequentialGraph& obj) {
//     _x_coords = obj._x_coords;
//     _y_coords = obj._y_coords;
//     _x_to_vertexs = obj._x_to_vertexs;
//     _y_to_vertexs = obj._y_to_vertexs;

//     for(auto v : obj._vertexes) {

//     }
// }

sequentialGraph::~sequentialGraph() {
    for (int i = 0; i < _vertexes.size(); i++) {
        delete _vertexes[i];
    }
    for (int i = 0; i < _arcs.size(); i++) {
        delete _arcs[i];
    }
}

void sequentialGraph::add_vertex(sequentialVertex* v) {
    // should distinguish different type of vertexes, start/end/const
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

void sequentialGraph::initDegree() {
    for (auto vertex : _vertexes) {
        _vertex_to_degree[vertex->get_vertex()->get_name()] = vertex->get_src_edges().size();
    }
}

bool sequentialGraph::reduceDegree(sequentialVertex* v, std::unordered_map<std::string, int>& vertex_to_degree) {
    auto iter = vertex_to_degree.find(v->get_vertex()->get_name());
    if (iter == vertex_to_degree.end()) {
        _log->error("Error occur in reduce degree", 1, 1);
    } else {
        iter->second = iter->second - 1;
        if (iter->second == 0) {
            return true;
        }
    }
    return false;
}

/**
 * @brief init the searching pair, and arrange this distance.
 *
 */
void sequentialGraph::initSeqentialPair() {
    for (auto vertex : _vertexes) {
        auto vertex_base = vertex->get_vertex();
        if (vertex_base->get_type() == 3) {
            int low_x = vertex_base->get_coord().x - _side_length / 2;
            int low_y = vertex_base->get_coord().y - _side_length / 2;
            int top_x = vertex_base->get_coord().x + _side_length / 2;
            int top_y = vertex_base->get_coord().y + _side_length / 2;

            std::set<sequentialVertex*> region_vertexes;
            region_vertexes = this->getRegionVertexes(low_x, low_y, top_x, top_y);

            for (auto region_vertex : region_vertexes) {
                if (vertex == region_vertex) {
                    continue;
                }

                // ring case
                if (this->findRing(vertex, region_vertex, _vertex_to_degree)) {
                    continue;
                }

                sequentialPair* pair = new sequentialPair(vertex, region_vertex);
                _sequential_pairs.insert(pair);

                // direct connecting case
                for (auto v : region_vertex->get_connect_vertexes()) {
                    if (v == vertex->get_idx()) {
                        // shoule make normalization.
                        pair->set_distance(this->getDirectConnectingSkew(vertex, region_vertex));
                        continue;
                    }
                }
            }
        }
    }
}

/**
 * @brief giving an region , return vertexes inside the region.
 *
 * @param lx
 * @param ly
 * @param ux
 * @param uy
 * @return std::set<sequentialVertex*>
 */
std::set<sequentialVertex*> sequentialGraph::getRegionVertexes(int lx, int ly, int ux, int uy) {
    std::set<sequentialVertex*> region_vertexes;

    int low_subscript = 0;
    int high_subscript = 0;
    for (int i = 0; i < _x_coords.size(); i++) {
        if (_x_coords[i] <= lx) {
            low_subscript = i;
        } else if (_x_coords[i] > lx && _x_coords[i] <= uy) {
            high_subscript = i;
        } else {
            break;
        }
    }

    for (int i = low_subscript; i < high_subscript + 1; i++) {
        // find vertex* with giving x coord.
        std::set<sequentialVertex*> x_vertexes;
        x_vertexes = this->getXCoordVertexes(_x_coords[i]);
        for (auto vertex : x_vertexes) {
            int vertex_y = vertex->get_vertex()->get_coord().y;
            if (vertex_y >= ly && vertex_y <= uy) {
                region_vertexes.insert(vertex);
            }
        }
    }
    return region_vertexes;
}

/**
 * @brief giving a coord , return the vertex vector. If not found, return the empty vertexes.
 *
 * @param x
 * @return std::vector<sequentialVertex*>
 */
std::set<sequentialVertex*> sequentialGraph::getXCoordVertexes(int x) {
    std::set<sequentialVertex*> x_vertexes;
    auto key_num = _x_to_vertexs.count(x);
    if (key_num == 1) {
        uint vertex_id = _x_to_vertexs.find(x)->second;
        x_vertexes.insert(_vertexes[vertex_id]);
    } else if (key_num > 1) {
        auto range = _x_to_vertexs.equal_range(key_num);
        while (range.first != range.second) {
            uint vertex_id = range.first->second;
            x_vertexes.insert(_vertexes[vertex_id]);
            ++range.first;
        }
    }
    return x_vertexes;
}

double sequentialGraph::getDirectConnectingSkew(sequentialVertex* v1, sequentialVertex* v2) {
    for (auto edge : v2->get_src_edges()) {
        if (edge->get_src() == v1) {
            return v2->get_vertex()->get_skew();
        }
    }
    return v1->get_vertex()->get_skew();
}

void sequentialGraph::updatePairSkew(sequentialVertex* root) {
    _record_path.push_back(root);

    // Termination conditions
    if (root->get_sink_edges().empty()) {
        return;
    }

    for (auto edge : root->get_sink_edges()) {
        auto cur_vertex = edge->get_sink();
        updatePairSkew(cur_vertex);

        for (auto pair : _sequential_pairs) {
            sequentialVertex* another_vertex;
            sequentialVertex* ancestor;
            if (pair->findAnotherVetex(cur_vertex, another_vertex) && _visited[another_vertex]) {
                ancestor = findAncestor(another_vertex);
            }
        }

        _visited[cur_vertex] = true;
        _vertex_to_ancestor[cur_vertex] = root;

        // record the second path.
        

        _record_path.pop_back();
    }
    _visited[root] = true;
}

sequentialVertex* sequentialGraph::findAncestor(sequentialVertex* vertex) {
    sequentialVertex* cur_v = vertex;

    while (!(cur_v == _vertex_to_ancestor[cur_v])) {
        // can record the path.

        cur_v = _vertex_to_ancestor[cur_v];
    }

    return cur_v;
}

bool sequentialGraph::findRing(sequentialVertex* vertex_1, sequentialVertex* vertex_2,
                               std::unordered_map<std::string, int> vertex_to_degree) {
    // trick for quick search ring.
    for (auto v : vertex_1->get_connect_vertexes()) {
        if (v == vertex_2->get_idx()) {
            return false;
        }
    }

    if (vertex_1->get_to_vertexes().find(vertex_2->get_idx()) != vertex_1->get_to_vertexes().end()) {
        return true;
    }
    if (vertex_1->get_from_vertexes().find(vertex_2->get_idx()) != vertex_1->get_from_vertexes().end()) {
        return true;
    }

    // topological sorting to find ring.
    std::queue<sequentialVertex*> cur_queue;
    for (auto pi : _start_vertexes) {
        cur_queue.push(pi);
    }

    while (!cur_queue.empty()) {
        sequentialVertex* cur_vertex = cur_queue.front();
        for (auto edge : cur_vertex->get_sink_edges()) {
            sequentialVertex* sink_vertex = edge->get_sink();
            if (reduceDegree(sink_vertex, _vertex_to_degree)) {
                cur_queue.push(sink_vertex);
            }
        }
        cur_queue.pop();
    }

    for (auto vertex : _end_vertexes) {
        if (vertex_to_degree[vertex->get_vertex()->get_name()] != 0) {
            return true;
        }
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
        _x_to_vertexs.emplace(x_coord, vertex->get_idx());
        _y_to_vertexs.emplace(y_coord, vertex->get_idx());
    }
    std::sort(_x_coords.begin(), _y_coords.end());
    std::sort(_y_coords.begin(), _y_coords.end());
}
