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

void sequentialVertex::add_batch_ancestors(std::list<sequentialVertex*> batch) {
    std::stack<sequentialVertex*> tmp_stack;  // for keep the order of ancestors.
    for (auto iter = batch.rbegin(); iter != batch.rend(); iter++) {
        auto cur_v = *iter;

        bool is_exsited = std::find_if(_ancestors.begin(), _ancestors.end(),
                                       [cur_v](const sequentialVertex* v) { return *cur_v == *v; }) != _ancestors.end();
        if (is_exsited) {
            break;
        } else {
            tmp_stack.push(cur_v);
        }
    }
    while (!tmp_stack.empty()) {
        add_ancestor(tmp_stack.top());
        tmp_stack.pop();
    }
}

void sequentialVertex::add_batch_descendants(std::list<sequentialVertex*> batch) {
    std::stack<sequentialVertex*> tmp_stack;  // for keep the order of descendants.
    for (auto iter = batch.rbegin(); iter != batch.rend(); iter++) {
        auto cur_v = *iter;

        bool is_exsited = std::find_if(_descendants.begin(), _descendants.end(), [cur_v](const sequentialVertex* v) {
                              return *cur_v == *v;
                          }) != _descendants.end();

        if (is_exsited) {
            break;
        } else {
            tmp_stack.push(cur_v);
        }
    }
    while (!tmp_stack.empty()) {
        add_descendant(tmp_stack.top());
        tmp_stack.pop();
    }
}

void sequentialVertex::updateAncestors(std::list<sequentialVertex*> extra_ancestors) {
    _ancestors.sort(compareVertexPtrLess);
    _ancestors.unique(compareVertexPtrEqual);
    extra_ancestors.sort(compareVertexPtrLess);
    extra_ancestors.unique(compareVertexPtrEqual);

    std::list<sequentialVertex*> tmp;
    std::set_difference(extra_ancestors.begin(), extra_ancestors.end(), _ancestors.begin(), _ancestors.end(),
                        back_inserter(tmp), compareVertexPtrLess);
    _ancestors.merge(tmp);
}

void sequentialVertex::updateDescendants(std::list<sequentialVertex*> extra_descendants) {
    _descendants.sort(compareVertexPtrLess);
    _descendants.unique(compareVertexPtrEqual);
    extra_descendants.sort(compareVertexPtrLess);
    extra_descendants.unique(compareVertexPtrEqual);

    std::list<sequentialVertex*> tmp;
    std::set_difference(extra_descendants.begin(), extra_descendants.end(), _descendants.begin(), _descendants.end(),
                        back_inserter(tmp), compareVertexPtrLess);
    _descendants.merge(tmp);
}

void sequentialVertex::removeAncestor(sequentialVertex* v) {
    _ancestors.remove_if([v](const sequentialVertex* ancestor) { return *v == *ancestor; });
}

void sequentialVertex::removeDescendant(sequentialVertex* v) {
    _descendants.remove_if([v](const sequentialVertex* descendant) { return *v == *descendant; });
}

void sequentialVertex::duplicateRemoveAncestors() {
    _ancestors.sort(compareVertexPtrLess);
    _ancestors.unique(compareVertexPtrEqual);
}

void sequentialVertex::duplicateRemoveDescendants() {
    _descendants.sort(compareVertexPtrLess);
    _descendants.unique(compareVertexPtrEqual);
}

/**
 * @brief get the direct connect vertex.
 *
 * @param cur_vertex
 * @return std::vector<sequentialVertex*>
 */
std::vector<sequentialVertex*> sequentialVertex::get_direct_vertexes() {
    std::vector<sequentialVertex*> direct_vertex_vec;
    // get sink.
    for (auto edge : _sink_edges) {
        auto sink = edge->get_sink();
        direct_vertex_vec.push_back(sink);
    }

    // get source.
    for (auto edge : _src_edges) {
        auto src = edge->get_src();
        direct_vertex_vec.push_back(src);
    }

    return direct_vertex_vec;
}

std::set<sequentialVertex*, vertexPtrLess> sequentialVertex::get_src_vertexes() {
    std::set<sequentialVertex*, vertexPtrLess> src_set;
    for (auto edge : _src_edges) {
        auto src_v = edge->get_src();
        src_set.emplace(src_v);
    }
    return src_set;
}

std::set<sequentialVertex*, vertexPtrLess> sequentialVertex::get_sink_vertexes() {
    std::set<sequentialVertex*, vertexPtrLess> sink_set;
    for (auto edge : _sink_edges) {
        auto sink_v = edge->get_sink();
        sink_set.emplace(sink_v);
    }
    return sink_set;
}

vertexPair::vertexPair() : _vertex_1(nullptr), _vertex_2(nullptr) {}

vertexPair::vertexPair(sequentialVertex* v1, sequentialVertex* v2) {
    _vertex_1 = v1;
    _vertex_2 = v2;
}

sequentialPair::sequentialPair() : _name(""), _vertex_1(nullptr), _vertex_2(nullptr), _distance(DBL_MAX) {}

sequentialPair::sequentialPair(sequentialVertex* v1, sequentialVertex* v2) : sequentialPair() {
    _vertex_1 = v1;
    _vertex_2 = v2;

    if (*v1 == *v2) {
        cout << "Error in create sequentialPair." << endl;
    }

    string v1_name = v1->get_name();
    string v2_name = v2->get_name();

    if (v1_name < v2_name) {
        _name = v1_name + "-" + v2_name;
    } else {
        _name = v2_name + "-" + v1_name;
    }
}

sequentialPair::sequentialPair(vertexPair* pair) : sequentialPair(pair->get_vertex1(), pair->get_vertex2()) {}

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

sequentialGraph::sequentialGraph()
    : _name("")
    , _log(nullptr)
    , _virtual_root(nullptr)
    , _modify_arrival_cost(0.0)
    , _modify_relative_cost(0.0)
    , _modify_topo_cost(0.0) {}

sequentialGraph::sequentialGraph(string name, Logger* log, parameter* para) : sequentialGraph() {
    _name = name;
    _log = log;
    _para = para;
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
    _log = nullptr;
    _para = nullptr;
    _virtual_root = nullptr;

    // delete the vertex pointer.
    for (auto it = _vertexes.begin(); it != _vertexes.end(); it++) {
        delete it->second;
    }
    _vertexes.clear();

    // delete the arc pointer.
    for (auto it = _arcs.begin(); it != _arcs.end(); it++) {
        delete *it;
    }
    _arcs.clear();

    // delete the sequential pair
    for (auto it = _sequential_pairs.begin(); it != _sequential_pairs.end(); it++) {
        delete *it;
    }

    for (auto pair : _precluster_pair_vec) {
        delete pair;
    }
    _precluster_pair_vec.clear();

    for (auto sub_graph : _sub_graphs) {
        delete sub_graph;
    }
    _sub_graphs.clear();

    _x_to_vertexs.clear();
    _y_to_vertexs.clear();
    _arrivals.clear();
    _vertex_to_ancestor.clear();
    _visited.clear();
    _stack.~stack();
    _precluster_pair_set.clear();
    _pre_clusters_visited.clear();
    _pre_clusters_multimap.clear();
    _vertex_to_cluster_name.clear();
}

void sequentialGraph::initPreCluster() {
    std::stack<sequentialVertex*> vertex_stack;
    double begin, end;
    begin = microtime();
    for (auto pi : _start_vertexes) {
        if (!vertex_stack.empty()) {
            _log->error("ERROR in Stack", 1, 1);
        }

        vertex_stack.push(pi);
        calPreClusterPair(vertex_stack);
    }
    std::sort(_precluster_pair_vec.begin(), _precluster_pair_vec.end(), preClusterPairCMP);
    end = microtime();
    _log->printTime("[main-initPreCluster] init pre Cluster Cost", end - begin, 1);
}

void sequentialGraph::calPreClusterPair(std::stack<sequentialVertex*>& stack) {
    auto cur_v = stack.top();

    // end case.
    if (cur_v->isEnd()) {
        stack.pop();
        return;
    }

    // if the current vertex has been visited.
    if (_pre_clusters_visited.find(cur_v) != _pre_clusters_visited.end()) {
        stack.pop();
        return;
    }

    _pre_clusters_visited[cur_v] = true;

    auto edges = cur_v->get_sink_edges();
    for (int i = 0; i < edges.size(); i++) {
        auto next_v = edges[i]->get_sink();
        stack.push(next_v);
        calPreClusterPair(stack);

        // only choose both cur_v and next_v are flipflops.
        if (cur_v->get_base()->get_type() == 3 && next_v->get_base()->get_type() == 3) {
            // cal the weighting distance.
            vertexPair tmp(cur_v, next_v);
            double skew = next_v->get_base()->get_skew(cur_v->get_base());
            double distance =
                calDistance(&tmp, skew, _para->get_core_x(), _para->get_core_y(), _para->get_max_required_skew());

            // setting pre cluster pair.
            sequentialPair* seq_pair = new sequentialPair(&tmp);
            seq_pair->set_distance(distance);
            _precluster_pair_vec.push_back(seq_pair);
            _precluster_pair_set.emplace(seq_pair);
        }
    }
    stack.pop();
}

/**
 * @brief Solve the preCluster step.
 *
 */
void sequentialGraph::preClusterSolve() {
    string cluster_name;
    bool existed_flag;
    int cluster_id;
    // step 0 : construct preClusters from pair.
    for (auto pair : _precluster_pair_vec) {
        auto vertex_1 = pair->get_vertex1();
        auto vertex_2 = pair->get_vertex2();
        auto iter_1 = _vertex_to_cluster_name.find(vertex_1);
        auto iter_2 = _vertex_to_cluster_name.find(vertex_2);
        if (iter_1 == _vertex_to_cluster_name.end() && iter_2 == _vertex_to_cluster_name.end()) {
            cluster_name = "subCluster_" + std::to_string(cluster_id++);
            _pre_clusters_multimap.emplace(cluster_name, vertex_1);
            _pre_clusters_multimap.emplace(cluster_name, vertex_2);

            _vertex_to_cluster_name[vertex_1] = cluster_name;
            _vertex_to_cluster_name[vertex_2] = cluster_name;
        } else if (iter_1 != _vertex_to_cluster_name.end() && iter_2 == _vertex_to_cluster_name.end()) {
            cluster_name = iter_1->second;
            existed_flag = isExtraDistanceConnection(cluster_name, vertex_1, vertex_2, pair->get_distance());
            if (!existed_flag) {
                _pre_clusters_multimap.emplace(cluster_name, vertex_2);
                _vertex_to_cluster_name[vertex_2] = cluster_name;
            }
        } else if (iter_1 == _vertex_to_cluster_name.end() && iter_2 != _vertex_to_cluster_name.end()) {
            cluster_name = iter_2->second;

            existed_flag = isExtraDistanceConnection(cluster_name, vertex_2, vertex_1, pair->get_distance());
            if (!existed_flag) {
                _pre_clusters_multimap.emplace(cluster_name, vertex_1);
                _vertex_to_cluster_name[vertex_1] = cluster_name;
            }
        } else {
            std::vector<sequentialVertex*> tmp_vec;
            string cluster1_name = iter_1->second;
            string cluster2_name = iter_2->second;
            // vertex_1 and vertex_2 both in clusters.
            if (cluster1_name == cluster2_name) {
                continue;
            }
            // add cluster to another cluster.
            auto key_num1 = _pre_clusters_multimap.count(cluster1_name);
            auto key_num2 = _pre_clusters_multimap.count(cluster2_name);
            if (key_num1 + key_num2 > _para->pre_clus_size) {
                continue;
            }
            auto range = _pre_clusters_multimap.equal_range(cluster1_name);
            while (range.first != range.second) {
                auto vertex = range.first->second;
                existed_flag = isExtraDistanceConnection(cluster2_name, vertex_2, vertex_1, pair->get_distance());
                if (existed_flag) {
                    tmp_vec.clear();
                    break;
                }
                tmp_vec.push_back(vertex);
                ++range.first;
            }
            if (!existed_flag) {
                // easy edition, for quickly achieve.
                for (auto v : tmp_vec) {
                    _pre_clusters_multimap.emplace(cluster2_name, v);
                    _vertex_to_cluster_name[v] = cluster2_name;
                }
                _pre_clusters_multimap.erase(cluster1_name);
            }
        }
    }

    // step 1 : complete the sub clusters.
    string prefix_name = "single_";
    completePreCluster(prefix_name);

    // step 2 : acoording to pre clusters, construct the sub graph.
    _sub_graphs.clear();
    int graph_id = 0;
    for (auto it = _pre_clusters_multimap.begin(); it != _pre_clusters_multimap.end();) {
        string cluster_name = it->first;
        sequentialGraph* sub_graph = new sequentialGraph(std::to_string(graph_id++), _log, _para);
        constructSubGraphs(sub_graph, cluster_name);
        _sub_graphs.push_back(sub_graph);

        // traverse the key in unordered_multimap.
        while (++it != _pre_clusters_multimap.end() && it->first == cluster_name)
            ;
    }
}

bool sequentialGraph::isExtraDistanceConnection(std::string cluster_name, sequentialVertex* class_vertex,
                                                sequentialVertex* join_vertex, double distance) {
    auto join_link_vertexes = join_vertex->get_direct_vertexes();
    auto key_num = _pre_clusters_multimap.count(cluster_name);
    // preCluster size control.
    if (key_num + 1 > _para->pre_clus_size) {
        return true;
    }

    if (key_num == 1) {
        return false;
    } else if (key_num > 1) {
        auto range = _pre_clusters_multimap.equal_range(cluster_name);
        while (range.first != range.second) {
            auto vertex = range.first->second;
            if (*vertex == *class_vertex) {
                ++range.first;
            } else {
                for (auto link_vertex : join_link_vertexes) {
                    if (*link_vertex == *vertex) {
                        sequentialPair tmp(join_vertex, link_vertex);
                        // using hash, but the hash may be repeated, should replace later.
                        auto it = _precluster_pair_set.find(&tmp);
                        if (it != _precluster_pair_set.end()) {
                            auto extra_distance = (*it)->get_distance();
                            if (extra_distance > distance) {
                                return true;
                            }
                        } else {
                            _log->error("ERROR in finding pair", 1, 1);
                        }
                    }
                }
                ++range.first;
            }
        }
    } else {
        _log->error("ERROR in finding preCluster", 1, 1);
    }
    return false;
}

/**
 * @brief Add the remaining single flipflop to the preCluster.
 *
 * @param prefix_name
 */
void sequentialGraph::completePreCluster(std::string prefix_name) {
    string cluster_name;
    for (auto iter : _vertexes) {
        cluster_name = prefix_name + iter.first;
        auto vertex = iter.second;
        if (vertex->get_base()->get_type() != 3) {
            continue;
        }
        if (_vertex_to_cluster_name.find(vertex) == _vertex_to_cluster_name.end()) {
            _pre_clusters_multimap.emplace(cluster_name, vertex);
        }
    }
}

/**
 * @brief construct the subGraphs.
 *
 *
 * @param graph
 * @param cluster_name
 */
void sequentialGraph::constructSubGraphs(sequentialGraph* graph, string cluster_name) {
    std::set<sequentialVertex*, vertexPtrLess> curCluster_vertexes;
    auto bucket_num = _pre_clusters_multimap.count(cluster_name);
    if (bucket_num == 1) {
        auto cur_v = _pre_clusters_multimap.find(cluster_name)->second;
        sequentialVertex* sub_vertex = new sequentialVertex(cur_v->get_base());
        graph->add_start_vertex(sub_vertex);
        graph->add_vertex(cur_v->get_name(), sub_vertex);
        return;
    } else if (bucket_num > 1) {
        auto range = _pre_clusters_multimap.equal_range(cluster_name);
        while (range.first != range.second) {
            curCluster_vertexes.emplace(range.first->second);
            ++range.first;
        }
    } else {
        _log->error("ERROR in construct subGraphs finding cluster", 1, 1);
    }

    // get the PI,PO of the subGraph.
    std::vector<sequentialVertex*> PIs;
    std::set<sequentialVertex*, vertexPtrLess> POs;
    std::vector<sequentialVertex*> tmp;  // tmp store the intersection of two set.
    std::stack<sequentialVertex*> stack;
    std::unordered_set<sequentialVertex*, vertexPtrHash, vertexPtrEqual> visited_vertexes;

    for (auto vertex : curCluster_vertexes) {
        tmp.clear();
        // determine vertex is PI of the subGraph.
        auto src_set = vertex->get_src_vertexes();
        std::set_intersection(src_set.begin(), src_set.end(), curCluster_vertexes.begin(), curCluster_vertexes.end(),
                              std::back_inserter(tmp), compareVertexPtrLess);
        if (tmp.empty()) {
            sequentialVertex* pi_vertex = new sequentialVertex(vertex->get_base());
            graph->add_start_vertex(pi_vertex);
            graph->add_vertex(pi_vertex->get_name(), pi_vertex);
            PIs.push_back(vertex);
        }

        tmp.clear();
        // determine vertex is PO of the subGraph.
        auto sink_set = vertex->get_sink_vertexes();
        std::set_intersection(sink_set.begin(), sink_set.end(), curCluster_vertexes.begin(), curCluster_vertexes.end(),
                              std::back_inserter(tmp), compareVertexPtrLess);
        if (tmp.empty()) {
            sequentialVertex* po_vertex = new sequentialVertex(vertex->get_base());
            graph->add_end_vertex(po_vertex);
            graph->add_vertex(po_vertex->get_name(), po_vertex);
            POs.emplace(vertex);
        }
    }

    // construct the sub Graph.

    for (auto pi : graph->get_start_vertexes()) {
        auto origin_vertex = _vertexes[pi->get_name()];
        if (!stack.empty()) {
            _log->error("ERROR In Stack Init", 1, 1);
        }
        stack.push(origin_vertex);
        DFSSubGraphs(graph, stack, POs, curCluster_vertexes, visited_vertexes);
    }
}

/**
 * @brief DFS for construct the subGraph.
 *
 * @param graph
 * @param stack
 * @param POs
 * @param cluster_vertexes
 * @param visited_vertexes
 */
void sequentialGraph::DFSSubGraphs(
    sequentialGraph* graph, std::stack<sequentialVertex*>& stack, std::set<sequentialVertex*, vertexPtrLess> POs,
    std::set<sequentialVertex*, vertexPtrLess> cluster_vertexes,
    std::unordered_set<sequentialVertex*, vertexPtrHash, vertexPtrEqual>& visited_vertexes) {
    auto cur_vertex = stack.top();

    // PO case.
    if (POs.find(cur_vertex) != POs.end()) {
        stack.pop();
        return;
    }

    // visited.
    if (visited_vertexes.find(cur_vertex) != visited_vertexes.end()) {
        stack.pop();
        return;
    }
    visited_vertexes.emplace(cur_vertex);

    for (auto edge : cur_vertex->get_sink_edges()) {
        auto next_vertex = edge->get_sink();

        if (cluster_vertexes.find(next_vertex) == cluster_vertexes.end()) {
            continue;
        } else {
            stack.push(next_vertex);
            DFSSubGraphs(graph, stack, POs, cluster_vertexes, visited_vertexes);

            // check if the vertex is in the graph.
            sequentialVertex* graph_src_v = graph->get_vertex(cur_vertex->get_name());
            if (!graph_src_v) {
                graph_src_v = new sequentialVertex(cur_vertex->get_base());
                graph->add_const_vertex(graph_src_v);
                graph->add_vertex(cur_vertex->get_name(), graph_src_v);
            }
            sequentialVertex* graph_sink_v = graph->get_vertex(next_vertex->get_name());
            if (!graph_sink_v) {
                graph_sink_v = new sequentialVertex(next_vertex->get_base());
                graph->add_const_vertex(graph_sink_v);
                graph->add_vertex(next_vertex->get_name(), graph_sink_v);
            }

            // add the src.
            sequentialArc* new_arc = new sequentialArc(graph_src_v, graph_sink_v);
            graph->add_edge(new_arc);

            // vertex add the src.
            graph_src_v->add_sink_edges(new_arc);
            graph_sink_v->add_src_edges(new_arc);
        }
    }
    stack.pop();
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
    _log->printTime("[main-init-initHop] DFS Forward Timing", end - begin, 2);

    begin = microtime();
    for (auto& po : _end_vertexes) {
        if (!vertex_stack.empty()) {
            _log->error("ERROR in Stack", 1, 1);
        }
        vertex_stack.push(po);
        hopBackwardDFS(vertex_stack);
    }
    end = microtime();
    _log->printTime("[main-init-initHop] DFS Backward Timing", end - begin, 2);

    for (auto cur_v : _vertexes) {
        cur_v.second->duplicateRemoveDescendants();
        cur_v.second->duplicateRemoveAncestors();
    }
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
        cur_v->add_batch_descendants(next_v->get_descendants());
        cur_v->add_descendant(next_v);
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

    auto edges = cur_v->get_src_edges();
    for (int i = 0; i < edges.size(); i++) {
        auto front_v = edges[i]->get_src();
        stack.push(front_v);
        hopBackwardDFS(stack);

        // Add the vertex hop
        cur_v->add_batch_ancestors(front_v->get_ancestors());
        cur_v->add_ancestor(front_v);
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
//         v->add_descendant(fusion_vertex);

//         std::set<sequentialVertex*, sequentialVertex::vertexCmp> union_descendants;
//         std::set_union(v->get_descendants().begin(), v->get_descendants().end(), vertex_2->get_descendants().begin(),
//                        vertex_2->get_descendants().end(), inserter(union_descendants, union_descendants.begin()));
//         v->get_descendants() = union_descendants;
//     }

//     for (auto v : vertex_1->get_descendants()) {
//         v->get_ancestors().erase(vertex_1);
//         v->get_ancestors().erase(vertex_2);
//         v->add_ancestor(fusion_vertex);

//         std::set<sequentialVertex*, sequentialVertex::vertexCmp> union_ancestors;
//         std::set_union(v->get_ancestors().begin(), v->get_ancestors().end(), vertex_2->get_ancestors().begin(),
//                        vertex_2->get_ancestors().end(), inserter(union_ancestors, union_ancestors.begin()));
//         v->get_descendants() = union_ancestors;
//     }

//     for (auto v : vertex_2->get_ancestors()) {
//         v->get_descendants().erase(vertex_2);
//         v->get_descendants().erase(vertex_1);
//         v->add_descendant(fusion_vertex);

//         std::set<sequentialVertex*, sequentialVertex::vertexCmp> union_descendants;
//         std::set_union(v->get_descendants().begin(), v->get_descendants().end(), vertex_1->get_descendants().begin(),
//                        vertex_1->get_descendants().end(), inserter(union_descendants, union_descendants.begin()));
//         v->get_descendants() = union_descendants;
//     }

//     for (auto v : vertex_2->get_descendants()) {
//         v->get_ancestors().erase(vertex_2);
//         v->get_ancestors().erase(vertex_1);
//         v->add_ancestor(fusion_vertex);

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
    int flipflop_vertex_cnt = 0;
    begin = microtime();

    // record the search windows' size.
    int window_avg_size = 0;
    int window_max_size = 0;
    int window_single_cnt = 0;
    for (iter = _vertexes.begin(); iter != _vertexes.end(); iter++) {
        auto cur_vertex = (*iter).second;

        // init the vistor and the ancestor.
        _visited[cur_vertex] = false;
        _vertex_to_ancestor[cur_vertex] = cur_vertex;

        // get region vertexes.
        auto vertex_base = cur_vertex->get_base();
        if (vertex_base->get_type() == 3) {
            flipflop_vertex_cnt++;
            int low_x = vertex_base->get_coord().x - side_length / 2;
            int low_y = vertex_base->get_coord().y - side_length / 2;
            int top_x = vertex_base->get_coord().x + side_length / 2;
            int top_y = vertex_base->get_coord().y + side_length / 2;

            std::set<sequentialVertex*> region_vertexes;
            region_vertexes = this->getRegionVertexes(low_x, low_y, top_x, top_y);

            if (region_vertexes.size() == 1) {
                window_single_cnt++;
            } else {
                region_vertexes.size() - 1 > window_max_size ? window_max_size = region_vertexes.size() - 1
                                                             : window_max_size;
            }

            window_avg_size += region_vertexes.size() - 1;
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
    _log->printTime("[main-initSequentialPair] Init The Arrival Timing", end - begin, 2);
    _log->printDouble("[main-initSequentialPair] Search Windows Average Size",
                      (double)window_avg_size / (double)flipflop_vertex_cnt, 2);
    _log->printInt("[main-initSequentialPair] Search Windows Max Size", window_max_size, 2);
    _log->printInt("[main-initSequentialPair] Search Windows Single Flipflop Count", window_single_cnt, 2);

    // construct arrivals.
    addVirtualRoot();

    begin = microtime();
    leastCommonAncestorDFS(_virtual_root, core_x, core_y, max_skew);
    end = microtime();
    _log->printTime("[main-initSequentialPair] Init All Vertexes Distance", end - begin, 2);
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

bool sequentialGraph::deleteArrival(sequentialVertex* src, sequentialVertex* arrival) {
    auto src_iter = _arrivals.find(src);
    if (src_iter == _arrivals.end()) {
        string info = "Warning in delete the arrival : " + src->get_name() + " & " + arrival->get_name();
        _log->warn(info, 1, 1);
        return false;
    }

    // find the arrival
    auto& src_map = (*src_iter).second;
    auto arrival_iter = src_map.find(arrival);
    if (arrival_iter == src_map.end()) {
        string info = "Warning in delete the arrival : " + src->get_name() + " & " + arrival->get_name();
        _log->warn(info, 1, 1);
        return false;
    }
    src_map.erase(arrival_iter);
    return true;
}

void sequentialGraph::deleteSequentialPair(sequentialVertex* vertex_1, sequentialVertex* vertex_2) {
    // debug.
    int i = 0;
    sequentialPair tmp(vertex_1, vertex_2);
    for (auto it = _sequential_pairs.begin(); it != _sequential_pairs.end(); it++) {
        i++;
        if (*(*it) == tmp) {
            _sequential_pairs.erase(it);
            return;
        }
    }
    string info = "Error in delete the SequentailPair : " + vertex_1->get_name() + " & " + vertex_2->get_name();
    _log->error(info, 1, 1);
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
    // only geometry coordate.
    double normal_skew = skew / max_skew;

    return normal_x + normal_y + (_para->lamda * normal_skew);
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

    auto low_iter = std::lower_bound(_x_coords.begin(), _x_coords.end(), lx);
    auto upper_iter = std::upper_bound(_x_coords.begin(), _x_coords.end(), ux);

    for (auto it = low_iter; it != upper_iter; it++) {
        std::set<sequentialVertex*> x_vertexes;
        x_vertexes = this->getXCoordVertexes(*it);
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

bool sequentialGraph::makeVertexFusion(sequentialVertex* vertex_1, sequentialVertex* vertex_2, sequentialBase* new_base,
                                       double extra_dist) {
    sequentialVertex* new_vertex = new sequentialVertex(new_base);
    double begin, end;
    // special case.
    if (_arrivals.find(vertex_1) == _arrivals.end() || _arrivals.find(vertex_2) == _arrivals.end()) {
        string info = "Warning in makeVertxFusion : " + vertex_1->get_name() + " & " + vertex_2->get_name();
        _log->warn(info, 1, 1);
        // modify arrival
        begin = microtime();
        deleteSequentialPair(vertex_1, vertex_2);
        end = microtime();
        _modify_arrival_cost += (end - begin);
        return false;
    }

    begin = microtime();
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
    end = microtime();
    _modify_topo_cost = (end - begin);

    // modify the ancestor and descestor, and update the arrival、sequential_pairs.
    // printAncestors();
    begin = microtime();
    modifyVertexPairAncestors(vertex_1, vertex_2, new_vertex);
    end = microtime();
    _modify_relative_cost += (end - begin);
    // printDescendants();

    // printDescendants();
    begin = microtime();
    modifyVertexPairDescendants(vertex_1, vertex_2, new_vertex);
    end = microtime();
    _modify_relative_cost += (end - begin);

    // printAncestors();

    // update the arrival and the sequential pairs.
    begin = microtime();
    std::map<sequentialVertex*, double, vertexPtrLess> new_map;

    auto map_1 = _arrivals.find(vertex_1)->second;
    auto map_2 = _arrivals.find(vertex_2)->second;

    std::map<sequentialVertex*, double, vertexPtrLess>::iterator iter_1;
    std::map<sequentialVertex*, double, vertexPtrLess>::iterator iter_2;

    for (iter_1 = map_1.begin(); iter_1 != map_1.end(); iter_1++) {
        // delete the sequential pair.
        // printSequentialPair();  // debug
        // printArrival(vertex_1);
        // printSequentialPair(vertex_1);
        if (vertex_1->get_name() == "A1_B1_C6_D7_E5_F1_G1_H1_I1_o418759") {  // debug.
            printSequentialPair(vertex_1);
        }
        deleteSequentialPair(vertex_1, iter_1->first);
        if (vertex_1->get_name() == "A1_B1_C6_D7_E5_F1_G1_H1_I1_o418759") {  // debug.
            printSequentialPair(vertex_1);
        }
        // printSequentialPair();  // debug

        // printArrival();  // debug

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
        // printArrival();  // debug
        // printSequentialPair();  // debug
    }

    for (iter_2 = map_2.begin(); iter_2 != map_2.end(); iter_2++) {
        // already be deleted.
        if (*(iter_2->first) == *vertex_1) {
            continue;
        }
        // printSequentialPair();  // debug
        // delete the sequential pair.
        deleteSequentialPair(vertex_2, iter_2->first);
        // printSequentialPair();  // debug

        // printArrival();  // debug

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
        // printArrival();         // debug
        // printSequentialPair();  // debug
    }

    // delete arrival vertex_1, vertex_2.
    auto v1_iter = _arrivals.find(vertex_1);
    _arrivals.erase(v1_iter);
    auto v2_iter = _arrivals.find(vertex_2);
    _arrivals.erase(v2_iter);
    end = microtime();
    _modify_arrival_cost += (end - begin);

    // delete vertex_1, vertex_2.
    begin = microtime();
    this->removeVertex(vertex_1);
    this->removeVertex(vertex_2);
    this->add_vertex(new_vertex->get_name(), new_vertex);
    end = microtime();
    _modify_topo_cost = (end - begin);
    return true;
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

/**
 * @brief update all ancestors of the vertex_1 and the vertex_2, modifying their descentants. Add fusion_vertex to the
 * list.
 *
 * @param vertex_1
 * @param vertex_2
 * @param fusion_vertex
 */
void sequentialGraph::modifyVertexPairAncestors(sequentialVertex* vertex_1, sequentialVertex* vertex_2,
                                                sequentialVertex* fusion_vertex) {
    std::list<sequentialVertex*> v1_ancestors_list = vertex_1->get_ancestors();
    for (auto iter = v1_ancestors_list.begin(); iter != v1_ancestors_list.end(); iter++) {
        sequentialVertex* v1_ancestor = *iter;

        // vertex_2 is vertex_1 ancestor case.
        if (*v1_ancestor == *vertex_2) {
            continue;
        }

        // add to the fusion_vertex.
        fusion_vertex->add_ancestor(v1_ancestor);

        // remove vertex_1, vertex_2 inside v1_ancestor's descendants_list.
        v1_ancestor->removeDescendant(vertex_1);
        v1_ancestor->removeDescendant(vertex_2);

        // add the fusion vertex in v1_ancestor's descendants_list.
        v1_ancestor->add_descendant(fusion_vertex);

        // add vertex_2 descendant to v1_ancestor.
        v1_ancestor->updateDescendants(vertex_2->get_descendants());
    }

    std::list<sequentialVertex*> v2_ancestors_list = vertex_2->get_ancestors();
    for (auto iter = v2_ancestors_list.begin(); iter != v2_ancestors_list.end(); iter++) {
        sequentialVertex* v2_ancestor = *iter;
        // vertex_1 is vertex_2 ancestor case.
        if (*v2_ancestor == *vertex_1) {
            continue;
        }

        // add to the fusion_vertex.
        fusion_vertex->add_ancestor(v2_ancestor);

        // remove vertex_1, vertex_2 inside v2_ancestor's descendants_list.
        v2_ancestor->removeDescendant(vertex_1);
        v2_ancestor->removeDescendant(vertex_2);

        // add the fusion vertex in v2_ancestor's descendants_list.
        v2_ancestor->add_descendant(fusion_vertex);

        // add vertex_1 descendant to v2_ancestor.
        v2_ancestor->updateDescendants(vertex_1->get_descendants());
    }

    // duplicate the repeat element.
    fusion_vertex->duplicateRemoveAncestors();
}

/**
 * @brief update all descentants of the vertex_1 and the vertex_2, modifying their ancestors. Add fusion_vertex to the
 * list.
 *
 * @param vertex_1
 * @param vertex_2
 * @param fusion_vertex
 */
void sequentialGraph::modifyVertexPairDescendants(sequentialVertex* vertex_1, sequentialVertex* vertex_2,
                                                  sequentialVertex* fusion_vertex) {
    std::list<sequentialVertex*> v1_descendants_list = vertex_1->get_descendants();
    for (auto iter = v1_descendants_list.begin(); iter != v1_descendants_list.end(); iter++) {
        sequentialVertex* v1_descendant = *iter;
        // vertex_2 is vertex_1 descendant case.
        if (*v1_descendant == *vertex_2) {
            continue;
        }

        // add to the fusion_vertex.
        fusion_vertex->add_descendant(v1_descendant);

        // remove vertex_1, vertex_2 inside v1_descendant's anscentors_list.
        v1_descendant->removeAncestor(vertex_1);
        v1_descendant->removeAncestor(vertex_2);

        // add the fusion vertex in v1_descendant's anscentors_list.
        v1_descendant->add_ancestor(fusion_vertex);

        // add vertex_2 anscentors to v1_descendant.
        v1_descendant->updateAncestors(vertex_2->get_ancestors());
    }

    std::list<sequentialVertex*> v2_descendants_list = vertex_2->get_descendants();
    for (auto iter = v2_descendants_list.begin(); iter != v2_descendants_list.end(); iter++) {
        sequentialVertex* v2_descendant = *iter;
        // vertex_1 is vertex_2 descendant case.
        if (*v2_descendant == *vertex_1) {
            continue;
        }

        // add to the fusion_vertex.
        fusion_vertex->add_descendant(v2_descendant);

        // remove vertex_1, vertex_2 inside v2_descendant's anscentors_list.
        v2_descendant->removeAncestor(vertex_1);
        v2_descendant->removeAncestor(vertex_2);

        // add the fusion vertex in v2_descendant's anscentors_list.
        v2_descendant->add_ancestor(fusion_vertex);

        // add vertex_1 descendant to v2_ancestor.
        v2_descendant->updateAncestors(vertex_1->get_ancestors());
    }

    // duplicate the repeat element.
    fusion_vertex->duplicateRemoveDescendants();
}

bool sequentialGraph::isSrcInsidePreCluster(
    sequentialVertex* vertex, std::unordered_set<sequentialVertex*, vertexPtrHash, vertexPtrEqual> cluster) {
    for (auto edge : vertex->get_src_edges()) {
        sequentialVertex* src_v = edge->get_src();
        auto iter = cluster.find(src_v);
        if (iter != cluster.end()) {
            return true;
        }
    }
    return false;
}

bool sequentialGraph::isSinkInsidePreCluster(
    sequentialVertex* vertex, std::unordered_set<sequentialVertex*, vertexPtrHash, vertexPtrEqual> cluster) {
    for (auto edge : vertex->get_sink_edges()) {
        sequentialVertex* sink_v = edge->get_sink();
        auto iter = cluster.find(sink_v);
        if (iter != cluster.end()) {
            return true;
        }
    }
    return false;
}

void sequentialGraph::printArrival() {
    auto arrival_sheet = _arrivals;
    _log->printItself("Print the arrival sheet", 1);
    for (auto head : arrival_sheet) {
        auto head_vertex = head.first;
        cout << "[" << head_vertex->get_name() << "] :";
        for (auto arrival : head.second) {
            auto arrival_vertex = arrival.first;
            cout << " " << arrival_vertex->get_name() << " ";
        }
        cout << endl;
    }
}

void sequentialGraph::printSequentialPair() {
    auto pair_gather = _sequential_pairs;
    _log->printItself("Print the sequential pair", 1);
    for (auto it = pair_gather.begin(); it != pair_gather.end(); it++) {
        auto vertex_1 = (*it)->get_vertex1();
        auto vertex_2 = (*it)->get_vertex2();
        cout << "(" << vertex_1->get_name() << "," << vertex_2->get_name() << ") ";
    }
    cout << endl;
}

void sequentialGraph::printAncestors() {
    _log->printItself("Print the Ancestors", 1);
    for (auto vertex_map : _vertexes) {
        auto vertex = vertex_map.second;
        if (vertex->get_base()->get_type() == 3 || vertex->get_base()->get_type() == 4) {
            cout << "[" << vertex->get_name() << "] :";
            for (auto ancestor : vertex->get_ancestors()) {
                cout << " " << ancestor->get_name() << " ";
            }
            cout << endl;
        }
    }
}

void sequentialGraph::printDescendants() {
    _log->printItself("Print the Descendants", 1);
    for (auto vertex_map : _vertexes) {
        auto vertex = vertex_map.second;
        if (vertex->get_base()->get_type() == 3 || vertex->get_base()->get_type() == 4) {
            cout << "[" << vertex->get_name() << "] :";
            for (auto descendant : vertex->get_descendants()) {
                cout << " " << descendant->get_name() << " ";
            }
            cout << endl;
        }
    }
}

void sequentialGraph::printArrival(sequentialVertex* vertex) {
    string info = "Print the Vertex [" + vertex->get_name() + "] arrival";
    _log->printItself(info, 1);
    for (auto arrival : _arrivals[vertex]) {
        auto arrival_vertex = arrival.first;
        cout << " " << arrival_vertex->get_name() << " ";
    }
}

void sequentialGraph::printSequentialPair(sequentialVertex* vertex) {
    string info = "Print the Vertex [" + vertex->get_name() + "] sequential pair";
    _log->printItself(info, 1);
    for (auto arrival : _arrivals[vertex]) {
        auto arrival_vertex = arrival.first;
        sequentialPair tmp(vertex, arrival_vertex);
        // debug.
        int i = 0;
        for (auto it = _sequential_pairs.begin(); it != _sequential_pairs.end(); it++) {
            i++;
            if (*(*it) == tmp) {
                cout << "(" << vertex->get_name() << "," << arrival_vertex->get_name() << ") ";
                break;
            }
        }
    }
    _log->printItself("", 1);
}