#include "sequentialGraph.h"

sequentialArc::sequentialArc() : _src(nullptr), _sink(nullptr) {}

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

sequentialVertex::sequentialVertex() : _name(""), _node(nullptr), _is_start(0), _is_end(0), _is_const(0) {}

sequentialVertex::sequentialVertex(sequentialBase* node) : sequentialVertex() {
    _name = node->get_name();
    _node = node;
}

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
    _name = "";
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

vertexPair::vertexPair() : _vertex_1(nullptr), _vertex_2(nullptr) {}

vertexPair::vertexPair(sequentialVertex* v1, sequentialVertex* v2) {
    _vertex_1 = v1;
    _vertex_2 = v2;
}

sequentialPair::sequentialPair() : _vertex_1(nullptr), _vertex_2(nullptr), _distance(DBL_MAX) {}

sequentialPair::sequentialPair(sequentialVertex* v1, sequentialVertex* v2) : sequentialPair() {
    _vertex_1 = v1;
    _vertex_2 = v2;
}

sequentialPair::sequentialPair(vertexPair* pair) : sequentialPair() {
    _vertex_1 = pair->get_vertex1();
    _vertex_2 = pair->get_vertex2();
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

// arrivalDistance::arrivalDistance(sequentialVertex* vertex, double dist) {
//     arrival_vertex = vertex;
//     distance = dist;
// }

sequentialGraph::sequentialGraph() : _log(nullptr), _virtual_root(nullptr) {}

sequentialGraph::sequentialGraph(Logger* log) : sequentialGraph() { _log = log; }

// sequentialGraph::sequentialGraph(const sequentialGraph& obj) {
//     _x_coords = obj._x_coords;
//     _y_coords = obj._y_coords;
//     _x_to_vertexs = obj._x_to_vertexs;
//     _y_to_vertexs = obj._y_to_vertexs;

//     for(auto v : obj._vertexes) {

//     }
// }

sequentialGraph::~sequentialGraph() {
    _vertexes.clear();
    _arcs.clear();
}

void sequentialGraph::removeVertex(sequentialVertex* vertex) {
    auto iter = _vertexes.find(vertex->get_name());
    if (iter != _vertexes.end()) {
        _vertexes.erase(iter);
    } else {
        _log->error("Error in remove vertex", 1, 1);
    }
    return;
}

void sequentialGraph::add_vertex(std::string name, sequentialVertex* v) {
    if (_vertexes.find(name) != _vertexes.end()) {
        _log->error("ERROR in add vertex.", 1, 1);
    }
    _vertexes[name] = v;
}

void sequentialGraph::add_edge(sequentialArc* e) { _arcs.insert(e); }

void sequentialGraph::add_start_vertex(sequentialVertex* start_vertex) { _start_vertexes.insert(start_vertex); }

void sequentialGraph::add_end_vertex(sequentialVertex* end_vertex) { _end_vertexes.insert(end_vertex); }

void sequentialGraph::add_const_vertex(sequentialVertex* const_vertex) { _const_vertexes.insert(const_vertex); }

void sequentialGraph::initHop() {
    std::stack<sequentialVertex*> vertex_stack;
    double begin, end;
    begin = microtime();
    for (auto& pi : _start_vertexes) {
        if (!vertex_stack.empty()) {
            _log->error("ERROR in Stack", 1, 1);
        }

        vertex_stack.push(pi);
        hopForwardDFS(vertex_stack);
    }
    end = microtime();
    std::cout << "DFS Forward Timing : " << end - begin << "'s" << std::endl;

    begin = microtime();
    for (auto& po : _end_vertexes) {
        if (!vertex_stack.empty()) {
            _log->error("ERROR in Stack", 1, 1);
        }
        vertex_stack.push(po);
        hopBackwardDFS(vertex_stack);
    }
    end = microtime();
    std::cout << "DFS Backward Timing : " << end - begin << "'s" << std::endl;
}

/**
 * @brief forward to traverse the graph, in order to update "to" vertexes.
 *
 * @param stack
 */
void sequentialGraph::hopForwardDFS(std::stack<sequentialVertex*>& stack) {
    auto cur_v = stack.top();

    // end case.
    if (cur_v->isEnd()) {
        stack.pop();
        return;
    }

    // if the descendants is not empty, it has been arrange.
    if (!cur_v->get_descendants().empty()) {
        stack.pop();
        return;
    }

    auto edges = cur_v->get_sink_edges();
    for (int i = 0; i < edges.size(); i++) {
        auto next_v = edges[i]->get_sink();
        stack.push(next_v);
        hopForwardDFS(stack);

        // Add the vertex hop.
        for (auto vertex : next_v->get_descendants()) {
            cur_v->add_descendants(vertex);
        }
        cur_v->add_descendants(next_v);
    }

    stack.pop();
}

/**
 * @brief backward to traverse the graph, in order to update "from" vertexes.
 *
 * @param stack
 */
void sequentialGraph::hopBackwardDFS(std::stack<sequentialVertex*>& stack) {
    auto cur_v = stack.top();

    if (cur_v->isStart()) {
        stack.pop();
        return;
    }

    if (!cur_v->get_ancestors().empty()) {
        stack.pop();
        return;
    }

    for (int i = 0; i < cur_v->get_src_edges().size(); i++) {
        auto front_v = cur_v->get_src_edges()[i]->get_src();
        stack.push(front_v);
        hopBackwardDFS(stack);

        // Add the vertex hop
        for (auto vertex : front_v->get_ancestors()) {
            cur_v->add_ancestors(vertex);
        }
        cur_v->add_ancestors(front_v);
    }
    stack.pop();
}

/**
 * @brief update the hop according to the relationship.
 *
 * @param vertex_1
 * @param vertex_2
 * @param fusion_vertex
 */
// void sequentialGraph::updateHop(sequentialVertex* vertex_1, sequentialVertex* vertex_2,
//                                 sequentialVertex* fusion_vertex) {
//     for (auto v : vertex_1->get_ancestors()) {
//         v->get_descendants().erase(vertex_1);
//         v->get_descendants().erase(vertex_2);
//         v->add_descendants(fusion_vertex);

//         std::set<sequentialVertex*, sequentialVertex::vertexCmp> union_descendants;
//         std::set_union(v->get_descendants().begin(), v->get_descendants().end(), vertex_2->get_descendants().begin(),
//                        vertex_2->get_descendants().end(), inserter(union_descendants, union_descendants.begin()));
//         v->get_descendants() = union_descendants;
//     }

//     for (auto v : vertex_1->get_descendants()) {
//         v->get_ancestors().erase(vertex_1);
//         v->get_ancestors().erase(vertex_2);
//         v->add_ancestors(fusion_vertex);

//         std::set<sequentialVertex*, sequentialVertex::vertexCmp> union_ancestors;
//         std::set_union(v->get_ancestors().begin(), v->get_ancestors().end(), vertex_2->get_ancestors().begin(),
//                        vertex_2->get_ancestors().end(), inserter(union_ancestors, union_ancestors.begin()));
//         v->get_descendants() = union_ancestors;
//     }

//     for (auto v : vertex_2->get_ancestors()) {
//         v->get_descendants().erase(vertex_2);
//         v->get_descendants().erase(vertex_1);
//         v->add_descendants(fusion_vertex);

//         std::set<sequentialVertex*, sequentialVertex::vertexCmp> union_descendants;
//         std::set_union(v->get_descendants().begin(), v->get_descendants().end(), vertex_1->get_descendants().begin(),
//                        vertex_1->get_descendants().end(), inserter(union_descendants, union_descendants.begin()));
//         v->get_descendants() = union_descendants;
//     }

//     for (auto v : vertex_2->get_descendants()) {
//         v->get_ancestors().erase(vertex_2);
//         v->get_ancestors().erase(vertex_1);
//         v->add_ancestors(fusion_vertex);

//         std::set<sequentialVertex*, sequentialVertex::vertexCmp> union_ancestors;
//         std::set_union(v->get_ancestors().begin(), v->get_ancestors().end(), vertex_1->get_ancestors().begin(),
//                        vertex_1->get_ancestors().end(), inserter(union_ancestors, union_ancestors.begin()));
//         v->get_descendants() = union_ancestors;
//     }
// }

/**
 * @brief init the all degree according to their src edge count.
 *
 */
void sequentialGraph::initDegree() {
    // travever the unordered_map.
    std::unordered_map<std::string, sequentialVertex*>::iterator iter;
    for (iter = _vertexes.begin(); iter != _vertexes.end(); iter++) {
        _vertex_to_degree[(*iter).first] = (*iter).second->get_src_edges().size();
    }
}

void sequentialGraph::getDirectHopSkew(sequentialVertex* v1, sequentialVertex* v2, double& skew) {
    for (auto edge : v1->get_src_edges()) {
        // can not compare the pointer.
        if (*(edge->get_src()) == *v2) {
            skew = v1->get_base()->get_avg_skew();
            return;
        }
    }

    for (auto edge : v2->get_src_edges()) {
        if (*(edge->get_src()) == *v1) {
            skew = v2->get_base()->get_avg_skew();
            return;
        }
    }

    _log->error("ERROR in get direct skew", 1, 1);
    return;
}

bool sequentialGraph::isDirectHop(sequentialVertex* v1, sequentialVertex* v2) {
    for (auto edge : v1->get_src_edges()) {
        // must take care the compare.
        if (*(edge->get_src()) == *v2) {
            return true;
        }
    }

    for (auto edge : v2->get_src_edges()) {
        if (*(edge->get_src()) == *v1) {
            return true;
        }
    }

    return false;
}

bool sequentialGraph::reduceDegree(sequentialVertex* v, std::unordered_map<std::string, int>& vertex_to_degree) {
    auto iter = vertex_to_degree.find(v->get_base()->get_name());
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
void sequentialGraph::initSeqentialPair(int side_length, int core_x, int core_y, double max_skew) {
    std::unordered_map<std::string, sequentialVertex*>::iterator iter;

    double begin, end;
    begin = microtime();
    for (iter = _vertexes.begin(); iter != _vertexes.end(); iter++) {
        auto cur_vertex = (*iter).second;

        // init the vistor and the ancestor.
        _visited[cur_vertex] = false;
        _vertex_to_ancestor[cur_vertex] = cur_vertex;

        // get region vertexes.
        auto vertex_base = cur_vertex->get_base();
        if (vertex_base->get_type() == 3) {
            int low_x = vertex_base->get_coord().x - side_length / 2;
            int low_y = vertex_base->get_coord().y - side_length / 2;
            int top_x = vertex_base->get_coord().x + side_length / 2;
            int top_y = vertex_base->get_coord().y + side_length / 2;

            std::set<sequentialVertex*> region_vertexes;
            region_vertexes = this->getRegionVertexes(low_x, low_y, top_x, top_y);
            _log->printInt(cur_vertex->get_name() + " search window's num", region_vertexes.size() - 1, 2);

            for (auto region_vertex : region_vertexes) {
                if (cur_vertex == region_vertex) {  // the vertex itself.
                    continue;
                }

                // ring case
                if (this->findRing(cur_vertex, region_vertex)) {
                    continue;
                }

                vertexPair* pair = new vertexPair(cur_vertex, region_vertex);

                // direct connecting case
                if (isDirectHop(cur_vertex, region_vertex)) {
                    sequentialPair* seq_pair = new sequentialPair(pair);
                    double skew;
                    getDirectHopSkew(cur_vertex, region_vertex, skew);
                    // Flipflop multi input case.

                    double distance = calDistance(pair, skew, core_x, core_y, max_skew);
                    seq_pair->set_distance(distance);  // setting direct distance.
                    _sequential_pairs.insert(seq_pair);

                    // update the arrivals.
                    updateArrival(cur_vertex, region_vertex, distance);

                } else {
                    updateArrival(cur_vertex, region_vertex, DBL_MAX);
                }
            }
        }
    }
    end = microtime();
    std::cout << "init the arrival Timing : " << end - begin << "'s" << std::endl;

    // construct arrivals.
    addVirtualRoot();

    begin = microtime();
    leastCommonAncestorDFS(_virtual_root, core_x, core_y, max_skew);
    end = microtime();
    std::cout << "init all vertexes distance : " << end - begin << "'s" << std::endl;
}

/**
 * @brief update the arrivals.
 *
 * @param v1
 * @param v2
 */
void sequentialGraph::updateArrival(sequentialVertex* v1, sequentialVertex* v2, double distance) {
    auto v1_iter_1 = _arrivals.find(v1);
    // if has not init the _arrival.
    if (v1_iter_1 == _arrivals.end()) {
        std::map<sequentialVertex*, double, vertexPtrLess> init_map;
        init_map[v2] = distance;
        _arrivals[v1] = init_map;
    } else {
        auto& v1_map = (*v1_iter_1).second;
        v1_map[v2] = distance;
    }

    auto v2_iter_1 = _arrivals.find(v2);
    // if has not init the _arrival.
    if (v2_iter_1 == _arrivals.end()) {
        std::map<sequentialVertex*, double, vertexPtrLess> init_map;
        init_map[v1] = distance;
        _arrivals[v2] = init_map;
    } else {
        auto& v2_map = (*v2_iter_1).second;
        v2_map[v1] = distance;
    }
}

void sequentialGraph::deleteArrival(sequentialVertex* src, sequentialVertex* arrival) {
    auto src_iter = _arrivals.find(src);
    if (src_iter == _arrivals.end()) {
        _log->error("Error in delete the arrival", 1, 1);
    }

    // find the arrival
    auto& src_map = (*src_iter).second;
    auto arrival_iter = src_map.find(arrival);
    if (arrival_iter == src_map.end()) {
        _log->error("Error in delete the arrival", 1, 1);
    }
    src_map.erase(arrival_iter);
}

/**
 * @brief calculate the distance.
 *
 * @param pair
 * @param skew
 * @param core_x
 * @param core_y
 * @param max_skew
 * @return double
 */
double sequentialGraph::calDistance(vertexPair* pair, double skew, int core_x, int core_y, double max_skew) {
    auto v1 = pair->get_vertex1()->get_base();
    auto v2 = pair->get_vertex2()->get_base();
    double normal_x = fabs(v1->get_coord().x - v2->get_coord().x) / core_x;
    double normal_y = fabs(v1->get_coord().y - v2->get_coord().y) / core_y;
    double normal_skew = 0 * skew / max_skew;

    return normal_x + normal_y + normal_skew;
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
        } else if (_x_coords[i] > lx && _x_coords[i] <= ux) {
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
            // select the flipflop case.
            if (vertex->get_base()->get_type() != 3) {
                continue;
            }
            int vertex_y = vertex->get_base()->get_coord().y;
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
        sequentialVertex* vertex = _x_to_vertexs.find(x)->second;
        x_vertexes.insert(vertex);
    } else if (key_num > 1) {
        auto range = _x_to_vertexs.equal_range(x);
        while (range.first != range.second) {
            sequentialVertex* vertex = range.first->second;
            x_vertexes.insert(vertex);
            ++range.first;
        }
    }
    return x_vertexes;
}

double sequentialGraph::getDirectConnectingSkew(sequentialVertex* v1, sequentialVertex* v2) {
    for (auto edge : v2->get_src_edges()) {
        if (edge->get_src() == v1) {
            return v2->get_base()->get_avg_skew();
        }
    }
    return v1->get_base()->get_avg_skew();
}

// void sequentialGraph::updatePairSkew(sequentialVertex* root) {
//     _record_path.push_back(root);

//     // Termination conditions
//     if (root->get_sink_edges().empty()) {
//         return;
//     }

//     for (auto edge : root->get_sink_edges()) {
//         auto cur_vertex = edge->get_sink();
//         updatePairSkew(cur_vertex);

//         for (auto pair : _sequential_pairs) {
//             sequentialVertex* another_vertex;
//             sequentialVertex* ancestor;
//             if (pair->findAnotherVetex(cur_vertex, another_vertex) && _visited[another_vertex]) {
//                 ancestor = findAncestor(another_vertex);
//             }
//         }

//         _visited[cur_vertex] = true;
//         _vertex_to_ancestor[cur_vertex] = root;

//         // record the second path.

//         _record_path.pop_back();
//     }
//     _visited[root] = true;
// }

/**
 * @brief get the skew sum from one path.
 *
 * @param vertex
 * @return double
 */
sequentialVertex* sequentialGraph::findAncestor(sequentialVertex* vertex, double& skew) {
    sequentialVertex* cur_v = vertex;
    double skew_sum = 0;
    while (!(*cur_v == *(_vertex_to_ancestor[cur_v]))) {
        skew_sum += cur_v->get_base()->get_avg_skew();

        cur_v = _vertex_to_ancestor[cur_v];
    }
    skew = skew_sum;
    return cur_v;
}

/**
 * @brief giving two vertex, return whether is ring. using a trick to quily find ring
 *
 * @param vertex_1
 * @param vertex_2
 * @param vertex_to_degree
 * @return true
 * @return false
 */
// bool sequentialGraph::findRing(sequentialVertex* vertex_1, sequentialVertex* vertex_2,
//                                std::unordered_map<std::string, int> vertex_to_degree) {
//     // direct connecting case.
//     if (isDirectHop(vertex_1, vertex_2)) {
//         return false;
//     }

//     // trick for quick search ring.
//     if (vertex_1->get_to_vertexes().find(vertex_2->get_name()) != vertex_1->get_to_vertexes().end()) {
//         return true;
//     }
//     if (vertex_1->get_from_vertexes().find(vertex_2->get_name()) != vertex_1->get_from_vertexes().end()) {
//         return true;
//     }

//     // topological sorting to find ring.
//     std::queue<sequentialVertex*> cur_queue;
//     for (auto pi : _start_vertexes) {
//         cur_queue.push(pi);
//     }

//     while (!cur_queue.empty()) {
//         sequentialVertex* cur_vertex = cur_queue.front();
//         for (auto edge : cur_vertex->get_sink_edges()) {
//             sequentialVertex* sink_vertex = edge->get_sink();

//             if (reduceDegree(sink_vertex, _vertex_to_degree)) {
//                 cur_queue.push(sink_vertex);
//             }
//         }
//         cur_queue.pop();
//     }

//     for (auto vertex : _end_vertexes) {
//         if (vertex_to_degree[vertex->get_base()->get_name()] != 0) {
//             return true;
//         }
//     }

//     return false;
// }

bool sequentialGraph::findRing(sequentialVertex* vertex_1, sequentialVertex* vertex_2) {
    // direct connect case.
    if (isDirectHop(vertex_1, vertex_2)) {
        return false;
    }

    for (auto vertex : vertex_1->get_ancestors()) {
        if (*vertex == *vertex_2) {
            return true;
        }
    }
    for (auto vertex : vertex_1->get_descendants()) {
        if (*vertex == *vertex_2) {
            return true;
        }
    }

    return false;
}

/**
 * @brief add a common virtual root, in order to make sure every vertex has ancestor.
 *
 */
void sequentialGraph::addVirtualRoot() {
    sequentialBase* base = new sequentialCluster("virtual_root");
    sequentialVertex* root = new sequentialVertex(base);

    for (auto v : _start_vertexes) {
        sequentialArc* arc = new sequentialArc(root, v);
        root->add_sink_edges(arc);
    }

    _virtual_root = root;

    // make the virtual root flag.
    _vertex_to_ancestor[_virtual_root] = _virtual_root;
    _visited[_virtual_root] = false;
}

/**
 * @brief looking for the LCA,and arrange the distance.
 *
 * @param stack
 * @param search_set
 * @return uint
 */
void sequentialGraph::leastCommonAncestorDFS(sequentialVertex* root, int core_x, int core_y, double max_skew) {
    _stack.push(root);

    if (root->get_sink_edges().empty()) {
        return;
    }

    for (auto edge : root->get_sink_edges()) {
        auto cur_v = edge->get_sink();

        // continue if cur_v has been visited.
        if (_visited[cur_v]) {
            continue;
        }

        leastCommonAncestorDFS(cur_v, core_x, core_y, max_skew);

        // vistied ?
        double skew_1 = UINT_MAX;
        double skew_2 = UINT_MAX;
        double distance = UINT_MAX;
        auto iter_map = _arrivals.find(cur_v);

        // 1. PI/PO not consider. 2. Flipflop not have search vertex.
        if (iter_map != _arrivals.end()) {
            // get the arrival map.
            auto& iter_1 = iter_map->second;

            std::map<sequentialVertex*, double, vertexPtrHash>::iterator iter;

            for (iter = iter_1.begin(); iter != iter_1.end(); iter++) {
                sequentialVertex* visit_v = (*iter).first;
                bool flag = _visited[visit_v];
                if (flag && (*iter).second == DBL_MAX) {  // has no arrange the distance.

                    sequentialVertex* ancestor = findAncestor(visit_v, skew_1);
                    skew_2 = findAncestorByStack(_stack, ancestor);

                    // calculate the skew disfference.
                    vertexPair* pair = new vertexPair(cur_v, visit_v);
                    distance = calDistance(pair, abs(skew_1 - skew_2), core_x, core_y, max_skew);

                    // update the sequential pairs.
                    sequentialPair* seq_pair = new sequentialPair(pair);
                    seq_pair->set_distance(distance);
                    _sequential_pairs.insert(seq_pair);

                    // update the metrix.
                    updateArrival(cur_v, visit_v, distance);
                }
            }
        }

        _visited[cur_v] = true;
        _vertex_to_ancestor[cur_v] = root;
        _stack.pop();
    }
    // _visited[root] = true;
}

double sequentialGraph::findAncestorByStack(std::stack<sequentialVertex*> stack, sequentialVertex* ancestor) {
    double skew = 0;
    while (!stack.empty()) {
        auto cur_vertex = stack.top();
        if (*cur_vertex == *ancestor) {
            break;
        }
        skew += cur_vertex->get_base()->get_avg_skew();
        stack.pop();
    }
    return skew;
}

/**
 * @brief update the coord mapping the vertexs.
 *
 */
void sequentialGraph::updateCoordMapping() {
    std::unordered_map<std::string, sequentialVertex*>::iterator iter;

    for (iter = _vertexes.begin(); iter != _vertexes.end(); iter++) {
        auto cur_base = (*iter).second->get_base();
        // flipflop case.
        if (cur_base->get_type() == 3) {
            int x_coord = cur_base->get_coord().x;
            int y_coord = cur_base->get_coord().y;

            _x_coords.push_back(x_coord);
            _y_coords.push_back(y_coord);
            _x_to_vertexs.emplace(x_coord, (*iter).second);
            _y_to_vertexs.emplace(y_coord, (*iter).second);
        }
    }

    std::sort(_x_coords.begin(), _x_coords.end());
    std::sort(_y_coords.begin(), _y_coords.end());
}

void sequentialGraph::makeVertexFusion(sequentialVertex* vertex_1, sequentialVertex* vertex_2,
                                       sequentialVertex* new_vertex, double extra_dist) {
    // add src.
    for (auto edge : vertex_1->get_src_edges()) {
        // direct connect case.
        if (*vertex_2 == *(edge->get_src())) {
            continue;
        }

        edge->updateSink(new_vertex);
        new_vertex->add_src_edges(edge);
    }

    // should take care of repeat add.
    for (auto edge : vertex_2->get_src_edges()) {
        // direct connect case.
        if (*vertex_1 == *(edge->get_src())) {
            continue;
        }

        bool is_repeat = false;
        for (auto e : new_vertex->get_src_edges()) {
            if (*(e->get_src()) == *(edge->get_src())) {
                is_repeat = true;
            }
        }

        if (!is_repeat) {
            edge->updateSink(new_vertex);
            new_vertex->add_src_edges(edge);
        }
    }

    // add sink.
    for (auto edge : vertex_1->get_sink_edges()) {
        // direct connect case.
        if (*vertex_2 == *(edge->get_sink())) {
            continue;
        }

        edge->updateSrc(new_vertex);
        new_vertex->add_sink_edges(edge);
    }
    // should take care of repeat add.
    for (auto edge : vertex_2->get_sink_edges()) {
        // direct connect case.
        if (*vertex_1 == *(edge->get_sink())) {
            continue;
        }

        bool is_repeat = false;
        for (auto e : new_vertex->get_sink_edges()) {
            if (e->get_sink() == edge->get_sink()) {
                is_repeat = true;
            }
        }

        if (!is_repeat) {
            edge->updateSrc(new_vertex);
            new_vertex->add_sink_edges(edge);
        }
    }

    // modify the ancestor and descestor, and update the arrival、sequential_pairs.
    auto tmp_ancestors = vertex_1->get_ancestors();

    for (auto cur_v : tmp_ancestors) {
        // direct connect case.
        if (*cur_v == *vertex_2) {
            continue;
        }

        auto& descendants = cur_v->get_descendants();
        auto iter_1 = descendants.find(vertex_1);
        // must take care of judging.
        if (iter_1 != descendants.end()) {
            descendants.erase(iter_1);
        } else {
            // Error case in init the ancestors or descendants.
            // _log->error("Error in init the ancestors or descendants", 1, 1);
        }

        auto iter_2 = descendants.find(vertex_2);
        if (iter_2 != descendants.end()) {
            // make no sure can find.
            descendants.erase(iter_2);
        }

        cur_v->add_descendants(new_vertex);
        std::set<sequentialVertex*, sequentialVertex::vertexCmp> union_descendants;
        std::set_union(descendants.begin(), descendants.end(), vertex_2->get_descendants().begin(),
                       vertex_2->get_descendants().end(), inserter(union_descendants, union_descendants.begin()));
        descendants = union_descendants;

        // add info to the new_vertex.
        new_vertex->add_ancestors(cur_v);
    }

    for (auto cur_v : vertex_2->get_ancestors()) {
        // direct connect case.
        if (*cur_v == *vertex_1) {
            continue;
        }

        auto& descendants = cur_v->get_descendants();

        const auto& iter_2 = descendants.find(vertex_2);
        if (iter_2 != descendants.end()) {
            descendants.erase(iter_2);
        } else {
            // Error case in init the ancestors or descendants.
            // However, the v1 and v2 ancestors may be the same.
            // _log->error("Error in init the ancestors or descendants", 1, 1);
        }

        auto iter_1 = descendants.find(vertex_1);
        if (iter_1 != descendants.end()) {
            // make no sure can find.
            descendants.erase(iter_1);
        }

        cur_v->add_descendants(new_vertex);

        std::set<sequentialVertex*, sequentialVertex::vertexCmp> union_descendants;
        std::set_union(descendants.begin(), descendants.end(), vertex_1->get_descendants().begin(),
                       vertex_1->get_descendants().end(), inserter(union_descendants, union_descendants.begin()));
        descendants = union_descendants;

        // add info to the new_vertex.
        new_vertex->add_ancestors(cur_v);
    }

    for (auto cur_v : vertex_1->get_descendants()) {
        // direct connect case.
        if (*cur_v == *vertex_2) {
            continue;
        }

        auto& ancestors = cur_v->get_ancestors();
        auto iter_1 = ancestors.find(vertex_1);
        if (iter_1 != ancestors.end()) {
            ancestors.erase(iter_1);
        } else {
            // Error case in init the ancestors or descendants.
            // _log->error("Error in init the ancestors or descendants", 1, 1);
        }

        auto iter_2 = ancestors.find(vertex_2);
        if (iter_2 != ancestors.end()) {
            // make no sure can find.
            ancestors.erase(iter_2);
        }

        cur_v->add_ancestors(new_vertex);
        std::set<sequentialVertex*, sequentialVertex::vertexCmp> union_ancestors;
        std::set_union(ancestors.begin(), ancestors.end(), vertex_2->get_ancestors().begin(),
                       vertex_2->get_ancestors().end(), inserter(union_ancestors, union_ancestors.begin()));

        ancestors = union_ancestors;

        // add info to the new_vertex.
        new_vertex->add_descendants(cur_v);
    }

    for (auto cur_v : vertex_2->get_descendants()) {
        // direct connect case.
        if (*cur_v == *vertex_1) {
            continue;
        }

        auto& ancestors = cur_v->get_ancestors();

        auto iter_2 = ancestors.find(vertex_2);
        if (iter_2 != ancestors.end()) {
            ancestors.erase(iter_2);
        } else {
            // Error case in init the ancestors or descendants.
            // _log->error("Error in init the ancestors or descendants", 1, 1);
        }

        auto iter_1 = ancestors.find(vertex_1);
        if (iter_1 != ancestors.end()) {
            // make no sure can find.
            ancestors.erase(iter_1);
        }

        cur_v->add_ancestors(new_vertex);
        std::set<sequentialVertex*, sequentialVertex::vertexCmp> union_ancestors;
        std::set_union(ancestors.begin(), ancestors.end(), vertex_1->get_ancestors().begin(),
                       vertex_1->get_ancestors().end(), inserter(union_ancestors, union_ancestors.begin()));

        ancestors = union_ancestors;

        // add info to the new_vertex.
        new_vertex->add_descendants(cur_v);
    }

    // update the arrival and the sequential pairs.

    std::map<sequentialVertex*, double, vertexPtrLess> new_map;

    auto& map_1 = _arrivals.find(vertex_1)->second;
    auto& map_2 = _arrivals.find(vertex_2)->second;

    std::map<sequentialVertex*, double, vertexPtrLess>::iterator iter_1;
    std::map<sequentialVertex*, double, vertexPtrLess>::iterator iter_2;

    for (iter_1 = map_1.begin(); iter_1 != map_1.end(); iter_1++) {
        // delete the sequential pair.
        sequentialPair tmp(vertex_1, iter_1->first);
        for (auto it = _sequential_pairs.begin(); it != _sequential_pairs.end(); it++) {
            if (*(*it) == tmp) {
                _sequential_pairs.erase(it);
                break;
            }
        }

        bool is_existed = false;
        auto v_1 = iter_1->first;
        for (iter_2 = map_2.begin(); iter_2 != map_2.end(); iter_2++) {
            auto v_2 = iter_2->first;

            // if the arrival is the same.
            if (*v_1 == *v_2) {
                is_existed = true;
                updateArrival(new_vertex, v_1, (iter_1->second + iter_2->second) / 2);
                // only need to delete the v_1 to vertex_1 arrival.
                deleteArrival(v_1, vertex_1);

                sequentialPair* new_pair = new sequentialPair(new_vertex, v_1);
                new_pair->set_distance((iter_1->second + iter_2->second) / 2);
                _sequential_pairs.insert(new_pair);

                break;
            }

            // if two vertexes been the fusion vertex.
            if (*v_1 == *vertex_2) {
                is_existed = true;
                break;
            }
        }
        if (!is_existed) {
            updateArrival(new_vertex, v_1, (iter_1->second + extra_dist) / 2);
            // only need to delete the v_1 to vertex_1 arrival.
            deleteArrival(v_1, vertex_1);

            sequentialPair* new_pair = new sequentialPair(new_vertex, v_1);
            new_pair->set_distance((iter_1->second + extra_dist) / 2);
            _sequential_pairs.insert(new_pair);
        }
    }

    for (iter_2 = map_2.begin(); iter_2 != map_2.end(); iter_2++) {
        // delete the sequential pair.
        sequentialPair tmp2(vertex_2, iter_2->first);
        for (auto it = _sequential_pairs.begin(); it != _sequential_pairs.end(); it++) {
            if (*(*it) == tmp2) {
                _sequential_pairs.erase(it);
                break;
            }
        }

        bool is_existed = false;
        auto v_2 = iter_2->first;
        for (iter_1 = map_1.begin(); iter_1 != map_1.end(); iter_1++) {
            auto v_1 = iter_1->first;
            if (*v_1 == *v_2) {
                deleteArrival(v_2, vertex_2);
                is_existed = true;
                break;
            }

            if (*v_2 == *vertex_1) {
                is_existed = true;
                break;
            }
        }
        if (!is_existed) {
            updateArrival(new_vertex, v_2, (iter_2->second + extra_dist) / 2);
            // only need to delete the v_2 to vertex_2 arrival.
            deleteArrival(v_2, vertex_2);

            sequentialPair* new_pair = new sequentialPair(new_vertex, v_2);
            new_pair->set_distance((iter_2->second + extra_dist) / 2);
            _sequential_pairs.insert(new_pair);
        }
    }

    // delete arrival vertex_1, vertex_2.
    auto v1_iter = _arrivals.find(vertex_1);
    _arrivals.erase(v1_iter);
    auto v2_iter = _arrivals.find(vertex_2);
    _arrivals.erase(v2_iter);

    // delete vertex_1, vertex_2.
    this->removeVertex(vertex_1);
    this->removeVertex(vertex_2);
    this->add_vertex(new_vertex->get_name(), new_vertex);
}

bool sequentialGraph::isVertexExist(std::string vertex_name) {
    auto v = _vertexes.find(vertex_name);
    if (v == _vertexes.end()) {
        return false;
    } else {
        return true;
    }
}

sequentialVertex* sequentialGraph::get_vertex(std::string vertex_name) {
    if (isVertexExist(vertex_name)) {
        return _vertexes[vertex_name];
    } else {
        return nullptr;
    }
}