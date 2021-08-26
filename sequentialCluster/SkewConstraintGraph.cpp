/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-04 15:29:55
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-26 14:56:27
 * @Description:
 */

#include "SkewConstraintGraph.h"

#include <limits.h>

#include "omp.h"

struct SubCluster {
    SubCluster(int id) { idx = id; }
    void add_sub_vertex(itdp::SequentialVertex* v) { sub_vertexes.push_back(v); }

    int idx;
    std::vector<itdp::SequentialVertex*> sub_vertexes;
};

struct SequentialVertexPtrLess {
    bool operator()(const itdp::SequentialVertex* v1, const itdp::SequentialVertex* v2) {
        return v1->get_name() < v2->get_name();
    }
};

bool compareSequentialVertexPtrLess(const itdp::SequentialVertex* v1, const itdp::SequentialVertex* v2) {
    return v1->get_name() < v2->get_name();
}

bool compareSequentialVertexPtrEqual(const itdp::SequentialVertex* v1, const itdp::SequentialVertex* v2) {
    return v1->get_name() == v2->get_name();
}

bool sequentialEdgeCMP(const itdp::SequentialEdge* e1, const itdp::SequentialEdge* e2) {
    return e1->get_setup_skew() < e2->get_setup_skew();
}

namespace itdp {

SequentialVertex::SequentialVertex(SequentialElement* node) : _is_start(0), _is_end(0) {
    _name = node->get_name();
    _node = node;
    if (node->isPi()) {
        set_start();
    }
    if (node->isPo()) {
        set_end();
    }
}

/**
 * @description: add the descendant, prevent to repeat add.
 * @param {SequentialVertex*} v
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialVertex::add_descendant(SequentialVertex* v) {
    bool is_exist = std::find_if(_descendants.begin(), _descendants.end(), [v](const SequentialVertex* descendant) {
                        return *v == *descendant;
                    }) != _descendants.end();
    if (is_exist) {
        return;
    } else {
        _descendants.push_back(v);
    }
}

/**
 * @description: add the batch descndants.
 * @param {list<SequentialVertex*>} batch
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialVertex::add_batch_descendants(std::list<SequentialVertex*> batch) {
    // sort by address.
    _descendants.sort(compareSequentialVertexPtrLess);
    batch.sort(compareSequentialVertexPtrLess);

    std::list<SequentialVertex*> new_list;
    std::set_difference(batch.begin(), batch.end(), _descendants.begin(), _descendants.end(),
                        std::back_inserter(new_list), compareSequentialVertexPtrLess);

    _descendants.merge(new_list);
}

/**
 * @description: add the ancestor, prevent to repeat add.
 * @param {SequentialVertex*} v
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialVertex::add_ancestor(SequentialVertex* v) {
    bool is_exist = std::find_if(_ancestors.begin(), _ancestors.end(),
                                 [v](const SequentialVertex* ancestor) { return *v == *ancestor; }) != _ancestors.end();
    if (is_exist) {
        return;
    } else {
        _ancestors.push_back(v);
    }
}

/**
 * @description: add the batch ancestors in the special order.
 * @param {list<SequentialVertex*>} batch
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialVertex::add_batch_ancestors(std::list<SequentialVertex*> batch) {
    // sort by address.
    _ancestors.sort(compareSequentialVertexPtrLess);
    batch.sort(compareSequentialVertexPtrLess);

    std::list<SequentialVertex*> new_list;
    std::set_difference(batch.begin(), batch.end(), _ancestors.begin(), _ancestors.end(), std::back_inserter(new_list),
                        compareSequentialVertexPtrLess);

    _ancestors.merge(new_list);
}

SequentialEdge* SequentialVertex::get_src_edge(SequentialVertex* source) const {
    for (auto src_e : _src_edges) {
        if (src_e->get_src_vertex() == source) {
            return src_e;
        }
    }
    return nullptr;
}

SequentialEdge* SequentialVertex::get_sink_edge(SequentialVertex* destination) const {
    for (auto sink_e : _sink_edges) {
        if (sink_e->get_sink_vertex() == destination) {
            return sink_e;
        }
    }
    return nullptr;
}

void SequentialVertex::removeAncestor(SequentialVertex* ancestor) {
    _ancestors.remove_if([ancestor](SequentialVertex* v) { return *v == *ancestor; });
}

void SequentialVertex::removeDescendant(SequentialVertex* descendant) {
    _descendants.remove_if([descendant](SequentialVertex* v) { return *v == *descendant; });
}

/**
 * @description: Sort and Duplicate the ancestors.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialVertex::sortDuplicateAncestors() {
    _ancestors.sort(compareSequentialVertexPtrLess);
    _ancestors.unique(compareSequentialVertexPtrEqual);
}

/**
 * @description: Sort and Duplicate the descendants.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialVertex::sortDuplicateDescendants() {
    _descendants.sort(compareSequentialVertexPtrLess);
    _descendants.unique(compareSequentialVertexPtrEqual);
}

/**
 * @description: update the ancestors.
 * @param {list<SequentialVertex*>} extra_ancestors
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialVertex::updateAncestors(std::list<SequentialVertex*> extra_ancestors) {
    _ancestors.sort(compareSequentialVertexPtrLess);
    _ancestors.unique(compareSequentialVertexPtrEqual);
    extra_ancestors.sort(compareSequentialVertexPtrLess);
    extra_ancestors.unique(compareSequentialVertexPtrEqual);

    std::list<SequentialVertex*> difference_list;
    std::set_difference(extra_ancestors.begin(), extra_ancestors.end(), _ancestors.begin(), _ancestors.end(),
                        std::back_inserter(difference_list), compareSequentialVertexPtrLess);
    _ancestors.merge(difference_list);
}

/**
 * @description: update the descendants.
 * @param {list<SequentialVertex*>} extra_descendants
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialVertex::updateDescendants(std::list<SequentialVertex*> extra_descendants) {
    // repeat or not is unimportant for find ring.
    _descendants.sort(compareSequentialVertexPtrLess);
    _descendants.unique(compareSequentialVertexPtrEqual);
    extra_descendants.sort(compareSequentialVertexPtrLess);
    extra_descendants.unique(compareSequentialVertexPtrEqual);

    std::list<SequentialVertex*> difference_list;
    std::set_difference(extra_descendants.begin(), extra_descendants.end(), _descendants.begin(), _descendants.end(),
                        std::back_inserter(difference_list), compareSequentialVertexPtrLess);
    _descendants.merge(difference_list);
}

/**
 * @description: Remove the source edge.
 * @param {SequentialEdge*} e
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialVertex::removeSrcEdge(SequentialEdge* e) {
    auto it =
        std::find_if(_src_edges.begin(), _src_edges.end(), [e](const SequentialEdge* edge) { return *e == *edge; });
    if (it != _src_edges.end()) {
        _src_edges.erase(it);
    } else {
        Logger::get_logger_obj("xxx", 0)->error(e->get_name() + " doesn't in Vertex: " + _name, 1, 0);
    }
}

/**
 * @description: Remove the sink edge.
 * @param {SequentialEdge*} e
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialVertex::removeSinkEdge(SequentialEdge* e) {
    auto it =
        std::find_if(_sink_edges.begin(), _sink_edges.end(), [e](const SequentialEdge* edge) { return *e == *edge; });
    if (it != _sink_edges.end()) {
        _sink_edges.erase(it);
    } else {
        Logger::get_logger_obj("xxx", 0)->error(e->get_name() + " doesn't in Vertex: " + _name, 1, 0);
    }
}

SequentialEdge::SequentialEdge(SequentialVertex* src, SequentialVertex* sink) {
    std::string src_name = src->get_name();
    std::string sink_name = sink->get_name();
    const SequentialElement* sink_node = sink->get_node();
    src_name > sink_name ? _name = src_name + "-" + sink_name : _name = sink_name + "-" + src_name;
    _src = src;
    _sink = sink;
    _setup_skew = sink_node->get_skew(src_name);
}

void SequentialEdge::updateSink(SequentialVertex* sink) { _sink = sink; }

SkewConstraintGraph::SkewConstraintGraph(std::string name) {
    _name = name;
    _log = Logger::get_logger_obj("faker", 0);
    _parameter = Parameter::get_parameter_pointer();
}

SkewConstraintGraph::~SkewConstraintGraph() {
    for (auto vertex : _sequential_vertexes) {
        delete vertex.second;
    }
    for (auto edge : _sequential_edges) {
        delete edge.second;
    }
}

void SkewConstraintGraph::add_sequential_vertex(SequentialVertex* v) {
    if (v->isStart()) {
        auto it = _start_vertexes.find(v->get_name());
        if (it == _start_vertexes.end()) {
            _start_vertexes.emplace(v->get_name(), v);
        }

    } else if (v->isEnd()) {
        auto it = _end_vertexes.find(v->get_name());
        if (it == _end_vertexes.end()) {
            _end_vertexes.emplace(v->get_name(), v);
        }
    }
    _sequential_vertexes.emplace(v->get_name(), v);
}

void SkewConstraintGraph::add_sequential_edge(SequentialEdge* e) { _sequential_edges.emplace(e->get_name(), e); }

std::vector<SequentialVertex*> SkewConstraintGraph::get_start_vertexes() const {
    std::vector<SequentialVertex*> vertexes;
    for (auto v : _start_vertexes) {
        vertexes.push_back(v.second);
    }
    return vertexes;
}

bool SkewConstraintGraph::isStartVertex(std::string name) const {
    return _start_vertexes.find(name) != _start_vertexes.end();
}

bool SkewConstraintGraph::isEndVertex(std::string name) const {
    return _end_vertexes.find(name) != _end_vertexes.end();
}

std::vector<SequentialVertex*> SkewConstraintGraph::get_end_vertexes() const {
    std::vector<SequentialVertex*> vertexes;
    for (auto v : _end_vertexes) {
        vertexes.push_back(v.second);
    }
    return vertexes;
}

void SkewConstraintGraph::add_x_vertex_mapping(int x, SequentialVertex* v) {
    auto it = _x_to_vertexes.find(x);
    if (it == _x_to_vertexes.end()) {
        std::vector<SequentialVertex*> vertexes{v};
        _x_to_vertexes.emplace(x, vertexes);
    } else {
        (*it).second.push_back(v);
    }
}

void SkewConstraintGraph::deleteSequentialEdge(std::string name) {
    auto it = _sequential_edges.find(name);
    if (it != _sequential_edges.end()) {
        delete (*it).second;
        _sequential_edges.erase(it);
    } else {
        // _log->warn("Wanna to delete an unexist sequential edge :" + name, 1, 0);
    }
}

void SkewConstraintGraph::deleteSequentialVertex(std::string name) {
    auto it = _sequential_vertexes.find(name);
    if (it != _sequential_vertexes.end()) {
        delete (*it).second;
        _sequential_vertexes.erase(it);
    } else {
        _log->error("Wanna to delete an unexist sequential vertex", 1, 0);
    }
}

/**
 * @description: Determine whether the vertex already exists.
 * @param {string} name
 * @return {*}
 * @author: ShiJian Chen
 */
SequentialVertex* SkewConstraintGraph::get_existent_vertex(std::string name) {
    auto iter = _sequential_vertexes.find(name);
    if (iter == _sequential_vertexes.end()) {
        return nullptr;
    } else {
        return (*iter).second;
    }
}

/**
 * @description: Determine whether the edge already exists.
 * @param {string} src_name
 * @param {string} sink_name
 * @return {*}
 * @author: ShiJian Chen
 */
bool SkewConstraintGraph::isExistentEdge(std::string src_name, std::string sink_name) {
    std::string edge_name;
    src_name > sink_name ? edge_name = src_name + "-" + sink_name : edge_name = sink_name + "-" + src_name;
    auto iter = _sequential_edges.find(edge_name);
    if (iter == _sequential_edges.end()) {
        return false;
    } else {
        return true;
    }
}

/**
 * @description: Control the subgraph partition.
 * @param {int} max_subgraph_size
 * @return {*}
 * @author: ShiJian Chen
 */
std::map<std::string, std::vector<const SequentialElement*>> SkewConstraintGraph::subgraphPartition(
    int max_subgraph_size) {
    std::map<int, SubCluster*> sub_clusters;
    std::vector<SkewConstraintGraph*> sub_graphs;
    std::vector<SequentialVertex*> flipflop_vertexes;

    // obtain the flipflop vertexes.
    for (auto pair : _sequential_vertexes) {
        if (pair.second->get_node()->isFlipFlop()) {
            flipflop_vertexes.push_back(pair.second);
        }
    }

    std::vector<SequentialEdge*> ascend_skew_edges = sortEdgeBySkew();
    // classification.
    std::vector<SequentialVertex*> remain_vertexes;
    remain_vertexes = subClusterClassification(ascend_skew_edges, sub_clusters, max_subgraph_size, flipflop_vertexes);
    // Move the single cell into cluster.
    subClusterCompletion(sub_clusters, remain_vertexes);
    // balance the sub clusters.
    subClusterBalance(sub_clusters, max_subgraph_size);

    // bulid the subGraph.
    int subgraph_id = 0;
    int subgraph_vertexes_cnt = 0;
    int subgraph_min_size = INT_MAX;
    int subgraph_max_size = INT_MIN;
    for (auto it = sub_clusters.begin(); it != sub_clusters.end(); it++) {
        SkewConstraintGraph* sub_graph = new SkewConstraintGraph(std::to_string(subgraph_id));
        constructSubGraph(sub_graph, (*it).second);
        sub_graphs.push_back(sub_graph);
        sub_graph->get_sequential_vertexes_size() < subgraph_min_size
            ? subgraph_min_size = sub_graph->get_sequential_vertexes_size()
            : subgraph_min_size;
        sub_graph->get_sequential_vertexes_size() > subgraph_max_size
            ? subgraph_max_size = sub_graph->get_sequential_vertexes_size()
            : subgraph_max_size;
        subgraph_vertexes_cnt += sub_graph->get_sequential_vertexes_size();
        ++subgraph_id;
    }

    // print info.
    _log->printInt("SubGraph Count", sub_graphs.size(), 1);
    double subgraph_avg_size = static_cast<double>(subgraph_vertexes_cnt) / static_cast<double>(subgraph_id + 1);
    _log->printDouble("SubGraph Average Size", subgraph_avg_size, 1);
    _log->printInt("SubGraph Max Size", subgraph_max_size, 1);
    _log->printInt("SubGraph Min Size", subgraph_min_size, 1);

    // parall to do sequential cluster.
    std::map<std::string, std::vector<const SequentialElement*>> clusters;

    //     omp_set_num_threads(1);
    // #pragma omp parallel for
    for (size_t i = 0; i < sub_graphs.size(); i++) {
        // debug.
        // int j = static_cast<int>(i);
        // if (j == 58) {
        //     _log->printItself(std::to_string(j), 1);
        // }

        auto sub_graph = sub_graphs[i];
        sub_graph->initDistanceMatrix();
        auto subs = sub_graph->makeVertexesFusion();
        clusters.insert(subs.begin(), subs.end());
        _log->printInt("Finish SubGraph", i, 1);
    }

    // delete
    for (auto pair : sub_clusters) {
        delete pair.second;
    }
    for (auto graph : sub_graphs) {
        delete graph;
    }

    return clusters;
}

/**
 * @description: Complete the subCluster.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::subClusterCompletion(std::map<int, SubCluster*>& sub_clusters,
                                               std::vector<SequentialVertex*> remain_vertexes) {
    size_t max_idx = sub_clusters.rbegin()->first;

    for (auto vertex : remain_vertexes) {
        SubCluster* sub_cluster = new SubCluster(++max_idx);
        sub_cluster->add_sub_vertex(vertex);
        sub_clusters.emplace(sub_cluster->idx, sub_cluster);
    }
}

/**
 * @description: Balance the subCluster.
 * @param {int} max_subgraph_size
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::subClusterBalance(std::map<int, SubCluster*>& sub_clusters, int max_subgraph_size) {
    size_t max_idx = sub_clusters.rbegin()->first;

    std::vector<SequentialVertex*> reunion_vertexes;
    std::vector<SubCluster*> delete_clusters;
    for (auto pair : sub_clusters) {
        auto sub_cluster = pair.second;
        if (sub_cluster->sub_vertexes.size() < static_cast<size_t>(max_subgraph_size)) {
            delete_clusters.push_back(sub_cluster);
            for (auto sub_v : sub_cluster->sub_vertexes) {
                reunion_vertexes.push_back(sub_v);
            }
        }
    }
    for (auto cluster : delete_clusters) {
        sub_clusters.erase(cluster->idx);
        delete cluster;
    }

    int grid_size = std::sqrt(reunion_vertexes.size() / max_subgraph_size);
    int x_interval = reunion_vertexes.size() / (grid_size + 1);
    int y_interval = max_subgraph_size;

    // sort the reunion_vertexes.
    std::unordered_set<int> x_coords;
    std::vector<int> x_coords_vec;
    std::map<int, std::vector<SequentialVertex*>> x_to_vertexes;
    for (auto reunion_v : reunion_vertexes) {
        auto current_element = reunion_v->get_node();
        int x_coord = current_element->get_coord().get_x();
        x_coords.emplace(x_coord);
        // add to the vector.
        auto it = x_to_vertexes.find(x_coord);
        if (it == x_to_vertexes.end()) {
            std::vector<SequentialVertex*> vertexes{reunion_v};
            x_to_vertexes.emplace(x_coord, vertexes);
        } else {
            it->second.push_back(reunion_v);
        }
    }
    x_coords_vec.assign(x_coords.begin(), x_coords.end());
    std::sort(x_coords_vec.begin(), x_coords_vec.end());

    // rearrangement.
    std::vector<std::vector<SequentialVertex*>> x_clusters;
    std::vector<SequentialVertex*> tmp_vec;
    for (auto x_coord : x_coords_vec) {
        auto vertex_vec = x_to_vertexes[x_coord];
        if (tmp_vec.size() + vertex_vec.size() > static_cast<size_t>(x_interval)) {
            std::vector<SequentialVertex*> x_cluster = tmp_vec;
            x_clusters.push_back(x_cluster);
            tmp_vec.clear();
        }
        tmp_vec.insert(tmp_vec.end(), vertex_vec.begin(), vertex_vec.end());
    }
    std::vector<SequentialVertex*> x_cluster = tmp_vec;
    x_clusters.push_back(x_cluster);
    tmp_vec.clear();

    for (auto x_cluster : x_clusters) {
        std::unordered_set<int> y_coords;
        std::vector<int> y_coords_vec;
        std::map<int, std::vector<SequentialVertex*>> y_to_vertexes;
        for (auto current_v : x_cluster) {
            auto current_element = current_v->get_node();
            int y_coord = current_element->get_coord().get_y();
            y_coords.emplace(y_coord);
            // add to the vector.
            auto it = y_to_vertexes.find(y_coord);
            if (it == y_to_vertexes.end()) {
                std::vector<SequentialVertex*> vertexes{current_v};
                y_to_vertexes.emplace(y_coord, vertexes);
            } else {
                it->second.push_back(current_v);
            }
        }
        y_coords_vec.assign(y_coords.begin(), y_coords.end());
        std::sort(y_coords_vec.begin(), y_coords_vec.end());

        // rearrangement.
        std::vector<SequentialVertex*> tmp_vec;
        for (auto y_coord : y_coords_vec) {
            auto vertex_vec = y_to_vertexes[y_coord];
            if (tmp_vec.size() + vertex_vec.size() > static_cast<size_t>(y_interval)) {
                SubCluster* sub_cluster = new SubCluster(++max_idx);
                sub_cluster->sub_vertexes = tmp_vec;
                sub_clusters.emplace(sub_cluster->idx, sub_cluster);
                tmp_vec.clear();
            }
            tmp_vec.insert(tmp_vec.end(), vertex_vec.begin(), vertex_vec.end());
        }
        SubCluster* sub_cluster = new SubCluster(++max_idx);
        sub_cluster->sub_vertexes = tmp_vec;
        sub_clusters.emplace(sub_cluster->idx, sub_cluster);
        tmp_vec.clear();
    }
}

/**
 * @description: Sort the edges by the skew.
 * @param {*}
 * @return {std::vector<SequentialEdge*>}
 * @author: ShiJian Chen
 */
std::vector<SequentialEdge*> SkewConstraintGraph::sortEdgeBySkew() {
    std::vector<SequentialEdge*> edges;
    for (auto edge : _sequential_edges) {
        edges.push_back(edge.second);
    }
    std::sort(edges.begin(), edges.end(), sequentialEdgeCMP);
    return edges;
}

/**
 * @description: Main function of sub cluster partition, return the remain vertexes.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
std::vector<SequentialVertex*> SkewConstraintGraph::subClusterClassification(
    std::vector<SequentialEdge*> edges, std::map<int, SubCluster*>& sub_clusters, int max_subgraph_size,
    std::vector<SequentialVertex*> flipflop_vertexes) {
    int subcluster_id = 0;
    std::vector<SequentialVertex*> remain_vertexes;
    std::map<std::string, int> vertex_to_subcluster;
    for (auto edge : edges) {
        auto vertex_1 = edge->get_src_vertex();
        auto vertex_2 = edge->get_sink_vertex();

        auto iter_1 = vertex_to_subcluster.find(vertex_1->get_name());
        auto iter_2 = vertex_to_subcluster.find(vertex_2->get_name());
        bool flag_1, flag_2;
        iter_1 == vertex_to_subcluster.end() ? flag_1 = false : flag_1 = true;
        iter_2 == vertex_to_subcluster.end() ? flag_2 = false : flag_2 = true;
        if (!flag_1 && !flag_2) {
            SubCluster* sub_cluster = new SubCluster(subcluster_id);
            sub_cluster->add_sub_vertex(vertex_1);
            sub_cluster->add_sub_vertex(vertex_2);
            vertex_to_subcluster[vertex_1->get_name()] = subcluster_id;
            vertex_to_subcluster[vertex_2->get_name()] = subcluster_id;
            sub_clusters.emplace(subcluster_id, sub_cluster);
            ++subcluster_id;
        } else if (flag_1 && !flag_2) {
            int idx = (*iter_1).second;
            if (sub_clusters[idx]->sub_vertexes.size() >= static_cast<size_t>(max_subgraph_size)) {
                continue;
            }
            bool exist_flag = isExtraGreaterSkew(vertex_2, vertex_1, sub_clusters[idx], edge->get_setup_skew());
            if (!exist_flag) {
                sub_clusters[idx]->add_sub_vertex(vertex_2);
                vertex_to_subcluster[vertex_2->get_name()] = idx;
            }
        } else if (!flag_1 && flag_2) {
            int idx = (*iter_2).second;
            if (sub_clusters[idx]->sub_vertexes.size() >= static_cast<size_t>(max_subgraph_size)) {
                continue;
            }
            bool exist_flag = isExtraGreaterSkew(vertex_1, vertex_2, sub_clusters[idx], edge->get_setup_skew());
            if (!exist_flag) {
                sub_clusters[idx]->add_sub_vertex(vertex_1);
                vertex_to_subcluster[vertex_1->get_name()] = idx;
            }
        } else {
            int idx_1 = vertex_to_subcluster[vertex_1->get_name()];
            int idx_2 = vertex_to_subcluster[vertex_2->get_name()];
            auto vertex1_vec = sub_clusters[idx_1]->sub_vertexes;
            auto vertex2_vec = sub_clusters[idx_2]->sub_vertexes;
            // vertex_1 and vertex_2 both in same subcluster.
            if (idx_1 == idx_2) {
                continue;
            }
            if (vertex1_vec.size() + vertex2_vec.size() >= static_cast<size_t>(max_subgraph_size)) {
                continue;
            }
            bool exist_flag = false;
            for (auto sub_v1 : vertex1_vec) {
                for (auto sub_v2 : vertex2_vec) {
                    exist_flag = isExtraGreaterSkew(sub_v1, sub_v2, sub_clusters[idx_2], edge->get_setup_skew());
                }
            }
            if (!exist_flag) {
                for (auto sub_v2 : vertex2_vec) {
                    sub_clusters[idx_1]->add_sub_vertex(sub_v2);
                    vertex_to_subcluster[sub_v2->get_name()] = idx_1;
                }
                // delete the sub_clusters[idx_2].
                auto iter = sub_clusters.find(idx_2);
                delete (*iter).second;
                sub_clusters.erase(iter);
            }
        }
    }
    for (auto v : flipflop_vertexes) {
        // the vertex has not belonged to a subcluster.
        if (vertex_to_subcluster.find(v->get_name()) == vertex_to_subcluster.end()) {
            remain_vertexes.push_back(v);
        }
    }
    return remain_vertexes;
}

/**
 * @description: Determine whether there is an additional connection skew
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
bool SkewConstraintGraph::isExtraGreaterSkew(SequentialVertex* vertex, SequentialVertex* inner_vertex,
                                             SubCluster* sub_cluster, double current_skew) {
    std::vector<SequentialVertex*> src_vertexes;
    std::vector<SequentialVertex*> sink_vertexes;
    for (auto src_e : vertex->get_src_edges()) {
        src_vertexes.push_back(src_e->get_src_vertex());
    }
    for (auto sink_e : vertex->get_sink_edges()) {
        sink_vertexes.push_back(sink_e->get_sink_vertex());
    }

    auto sub_vertexes = sub_cluster->sub_vertexes;
    for (auto sub_v : sub_vertexes) {
        if (*sub_v == *inner_vertex) {
            continue;
        }
        for (auto src_v : src_vertexes) {
            if (*src_v == *sub_v) {
                double extra_skew = vertex->get_src_edge(src_v)->get_setup_skew();
                if (extra_skew > current_skew) {
                    return true;
                }
            }
        }
        for (auto sink_v : sink_vertexes) {
            if (*sink_v == *sub_v) {
                double extra_skew = vertex->get_sink_edge(sink_v)->get_setup_skew();
                if (extra_skew > current_skew) {
                    return true;
                }
            }
        }
    }
    return false;
}

/**
 * @description: According to the sub_clusters to construct the sub_graph.
 * @param {SkewConstraintGraph*} sub_graph
 * @param {SubCluster*} sub_cluster
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::constructSubGraph(SkewConstraintGraph* sub_graph, SubCluster* sub_cluster) {
    std::set<std::string> cluster_vertexes;
    std::stack<SequentialVertex*> stack;
    std::set<std::string> visited_vertexes;

    auto sub_vertexes = sub_cluster->sub_vertexes;
    if (sub_vertexes.size() == 1) {
        // make a single vertex for this cluster.
        SequentialVertex* sub_vertex = new SequentialVertex(sub_vertexes[0]->get_node());
        sub_vertex->set_start();
        sub_graph->add_sequential_vertex(sub_vertex);
        return;
    }

    for (auto vertex : sub_vertexes) {
        cluster_vertexes.emplace(vertex->get_name());
    }

    for (auto vertex : sub_vertexes) {
        addSubGraphInputOutput(sub_graph, vertex, cluster_vertexes);
    }

    // DFS to construct the sub graph.
    for (auto pi : sub_graph->get_start_vertexes()) {
        auto origin_v = get_existent_vertex(pi->get_name());
        if (!stack.empty()) {
            _log->error("Stack is not empty", 1, 0);
        }
        stack.push(origin_v);
        DFSGenerateSubGraph(sub_graph, cluster_vertexes, stack, visited_vertexes);
    }
}

/**
 * @description: Compare vertex's source/sink vertexes with the cluster vertexes, find the PIs/POs.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::addSubGraphInputOutput(SkewConstraintGraph* sub_graph, SequentialVertex* vertex,
                                                 std::set<std::string> cluster_vertexes) {
    std::set<std::string> src_vertexes;
    std::set<std::string> sink_vertexes;
    std::vector<std::string> tmp;  // tmp store the intersection of two set.
    tmp.clear();
    // construct the src_vertexes.
    for (auto src_e : vertex->get_src_edges()) {
        src_vertexes.emplace(src_e->get_src_vertex()->get_name());
    }
    std::set_intersection(src_vertexes.begin(), src_vertexes.end(), cluster_vertexes.begin(), cluster_vertexes.end(),
                          std::back_inserter(tmp));
    if (tmp.empty()) {
        // PI case.
        SequentialVertex* pi_vertex = new SequentialVertex(vertex->get_node());
        pi_vertex->set_start();
        sub_graph->add_sequential_vertex(pi_vertex);
    }

    tmp.clear();
    // construct the sink_vertexes.
    for (auto sink_e : vertex->get_sink_edges()) {
        sink_vertexes.emplace(sink_e->get_sink_vertex()->get_name());
    }
    std::set_intersection(sink_vertexes.begin(), sink_vertexes.end(), cluster_vertexes.begin(), cluster_vertexes.end(),
                          std::back_inserter(tmp));
    if (tmp.empty()) {
        // PO case.
        SequentialVertex* po_vertex = new SequentialVertex(vertex->get_node());
        po_vertex->set_end();
        sub_graph->add_sequential_vertex(po_vertex);
    }
}

/**
 * @description: Use dfs to traverse origin graph to complete the sub graph.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::DFSGenerateSubGraph(SkewConstraintGraph* sub_graph, std::set<std::string> cluster_vertexes,
                                              std::stack<SequentialVertex*>& stack,
                                              std::set<std::string>& visited_vertexes) {
    auto current_vertex = stack.top();
    // Terminate condition.
    if (sub_graph->isEndVertex(current_vertex->get_name())) {
        stack.pop();
        return;
    }
    // visited.
    if (visited_vertexes.find(current_vertex->get_name()) != visited_vertexes.end()) {
        stack.pop();
        return;
    }
    visited_vertexes.emplace(current_vertex->get_name());

    for (auto edge : current_vertex->get_sink_edges()) {
        auto next_vertex = edge->get_sink_vertex();
        if (cluster_vertexes.find(next_vertex->get_name()) == cluster_vertexes.end()) {
            continue;
        } else {
            stack.push(next_vertex);
            DFSGenerateSubGraph(sub_graph, cluster_vertexes, stack, visited_vertexes);

            // check if the vertex is in the graph.
            SequentialVertex* subgraph_src_v = sub_graph->get_existent_vertex(current_vertex->get_name());
            if (!subgraph_src_v) {
                subgraph_src_v = new SequentialVertex(current_vertex->get_node());
                sub_graph->add_sequential_vertex(subgraph_src_v);
            }
            SequentialVertex* subgraph_sink_v = sub_graph->get_existent_vertex(next_vertex->get_name());
            if (!subgraph_sink_v) {
                subgraph_sink_v = new SequentialVertex(next_vertex->get_node());
                sub_graph->add_sequential_vertex(subgraph_sink_v);
            }
            // add the edge.
            SequentialEdge* new_edge = new SequentialEdge(subgraph_src_v, subgraph_sink_v);
            sub_graph->add_sequential_edge(new_edge);
            // vertex add the edge.
            subgraph_src_v->add_sink_edge(new_edge);
            subgraph_sink_v->add_src_edge(new_edge);
        }
    }
    stack.pop();
}

/**
 * @description: Initialize all the reachable vertexes.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::initReachableVertexes() {
    std::stack<SequentialVertex*> stack;
    for (auto pair : _start_vertexes) {
        auto pi = pair.second;
        if (!stack.empty()) {
            _log->error("Stack is not empty during initReachableVertexes.", 1, 0);
        }
        stack.push(pi);
        hopForwardDFS(stack);
    }

    for (auto pair : _end_vertexes) {
        auto po = pair.second;
        if (!stack.empty()) {
            _log->error("Stack is not empty during initReachableVertexes.", 1, 0);
        }
        stack.push(po);
        hopBackwardDFS(stack);
    }

    // add the sort duplicate operation.
    for (auto pair : _sequential_vertexes) {
        auto vertex = pair.second;
        vertex->sortDuplicateAncestors();
        vertex->sortDuplicateDescendants();
    }
}

/**
 * @description: DFS forward to complete the descendants.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::hopForwardDFS(std::stack<SequentialVertex*>& stack) {
    auto current_vertex = stack.top();

    // Termination condition.
    if (current_vertex->isEnd()) {
        stack.pop();
        return;
    }
    // if the descendants is not empty, it has been arrange.
    if (!current_vertex->get_descendants().empty()) {
        stack.pop();
        return;
    }

    const auto& edges = current_vertex->get_sink_edges();
    for (size_t i = 0; i < edges.size(); i++) {
        auto next_vertex = edges[i]->get_sink_vertex();
        stack.push(next_vertex);
        hopForwardDFS(stack);

        // add the vertex hop.
        current_vertex->add_batch_descendants(next_vertex->get_descendants());
        current_vertex->add_descendant(next_vertex);
    }
    stack.pop();
}

/**
 * @description: DFS backward to complete the ancestors.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::hopBackwardDFS(std::stack<SequentialVertex*>& stack) {
    auto current_vertex = stack.top();

    // Termination condition.
    if (current_vertex->isStart()) {
        stack.pop();
        return;
    }
    // if the ancestors is not empty, it has been arrange.
    if (!current_vertex->get_ancestors().empty()) {
        stack.pop();
        return;
    }

    auto edges = current_vertex->get_src_edges();
    for (size_t i = 0; i < edges.size(); i++) {
        auto front_vertex = edges[i]->get_src_vertex();
        stack.push(front_vertex);
        // checkoutStack(stack);
        hopBackwardDFS(stack);

        // add the vertex hop.
        current_vertex->add_batch_ancestors(front_vertex->get_ancestors());
        current_vertex->add_ancestor(front_vertex);
    }
    stack.pop();
}

/**
 * @description: Debug.
 * @param {stack<SequentialVertex*>} stack
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::checkoutStack(std::stack<SequentialVertex*> stack) {
    int i = stack.size();
    auto current_vertex = stack.top();
    stack.pop();
    while (!stack.empty()) {
        auto search_vertex = stack.top();
        if (*current_vertex == *search_vertex) {
            _log->warn(current_vertex->get_name() + " has already been add", 1, 0);
            _log->printInt("Current Stack depth", i, 1);
        }
    }
}

/**
 * @description: Init the coordinates.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::initRegion() {
    std::unordered_set<int> x_coords;

    for (auto it = _sequential_vertexes.begin(); it != _sequential_vertexes.end(); it++) {
        auto current_element = (*it).second->get_node();
        // only consider the flipflop.
        if (current_element->isFlipFlop() || current_element->isBuffer()) {
            int x_coord = current_element->get_coord().get_x();
            x_coords.emplace(x_coord);
            add_x_vertex_mapping(x_coord, (*it).second);
        }
    }

    // sort and duplicate removal.
    _x_coordinates.assign(x_coords.begin(), x_coords.end());
    std::sort(_x_coordinates.begin(), _x_coordinates.end());
}

/**
 * @description: According to the region, return inside vertexes.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
std::vector<SequentialVertex*> SkewConstraintGraph::obtainRegionVertexes(std::pair<Point<DBU>, Point<DBU>> region) {
    std::vector<SequentialVertex*> region_vertexes;
    auto lower_point = region.first;
    auto upper_point = region.second;
    auto lower_iter = std::lower_bound(_x_coordinates.begin(), _x_coordinates.end(), lower_point.get_x());
    auto upper_iter = std::upper_bound(_x_coordinates.begin(), _x_coordinates.end(), upper_point.get_x());

    for (auto it = lower_iter; it != upper_iter; it++) {
        std::vector<SequentialVertex*> x_vertexes = _x_to_vertexes[*it];
        for (auto vertex : x_vertexes) {
            int y_coord = vertex->get_node()->get_coord().get_y();
            if (y_coord >= lower_point.get_y() && y_coord <= upper_point.get_y()) {
                region_vertexes.push_back(vertex);
            }
        }
    }
    return region_vertexes;
}

/**
 * @description: Determine whether is a ring between two vertexes.
 * thinking : is there can be a map for searching?
 * @param {SequentialVertex*} vertex_1
 * @param {SequentialVertex*} vertex_2
 * @return {*}
 * @author: ShiJian Chen
 */
bool SkewConstraintGraph::findRing(SequentialVertex* vertex_1, SequentialVertex* vertex_2) {
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
 * @description: Determine the two vertexes have direct connection.
 * @param {SequentialVertex*} vertex_1
 * @param {SequentialVertex*} vertex_2
 * @return {*}
 * @author: ShiJian Chen
 */
bool SkewConstraintGraph::isDirectHop(SequentialVertex* vertex_1, SequentialVertex* vertex_2) {
    auto vertex_1_edge = vertex_1->get_src_edge(vertex_2);
    if (vertex_1_edge) {
        return true;
    }
    auto vertex_2_edge = vertex_2->get_src_edge(vertex_1);
    if (vertex_2_edge) {
        return true;
    }
    return false;
}

/**
 * @description:
 * @param {SequentialVertex*} vertex_1
 * @param {SequentialVertex*} vertex_2
 * @return {*}
 * @author: ShiJian Chen
 */
double SkewConstraintGraph::get_direct_hop_skew(SequentialVertex* vertex_1, SequentialVertex* vertex_2) {
    double skew = DBL_MAX;
    auto vertex_1_edge = vertex_1->get_src_edge(vertex_2);
    if (vertex_1_edge) {
        skew = vertex_1_edge->get_setup_skew();
    }
    auto vertex_2_edge = vertex_2->get_src_edge(vertex_1);
    if (vertex_2_edge) {
        skew = vertex_2_edge->get_setup_skew();
    }
    if (skew == DBL_MAX) {
        _log->error(vertex_1->get_name() + " with " + vertex_2->get_name() + " are not the direct hop", 1, 0);
    }
    return skew;
}

/**
 * @description: Init the distance matrix(Graph).
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::initDistanceMatrix() {
    _x_coordinates.clear();
    _x_to_vertexes.clear();
    _distance_matrix.clear();

    initReachableVertexes();
    initRegion();

    // define the search window.
    std::pair<Point<DBU>, Point<DBU>> search_region;
    std::vector<SequentialVertex*> region_vertexes;

    // record the search windows average size.
    int region_vertexes_cnt = 0;
    int region_cnt = 0;

    // generate candidate pair
    for (auto pair : _sequential_vertexes) {
        auto current_vertex = pair.second;
        // only the flipflop should be considered.
        if (current_vertex->get_node()->isFlipFlop() || current_vertex->get_node()->isBuffer()) {
            search_region = obtainSearchRegion(current_vertex);
            region_vertexes = obtainRegionVertexes(search_region);
            ++region_cnt;
            region_vertexes_cnt += region_vertexes.size();
            for (auto region_v : region_vertexes) {
                if (findRing(current_vertex, region_v)) {
                    double skew = _parameter->get_max_required_skew();
                    double distance = calculateDistance(current_vertex, region_v, skew);
                    updateDistanceMatrix(current_vertex->get_name(), region_v->get_name(), distance);
                    continue;
                }
                if (isDirectHop(current_vertex, region_v)) {
                    double skew = get_direct_hop_skew(current_vertex, region_v);
                    double distance = calculateDistance(current_vertex, region_v, skew);
                    updateDistanceMatrix(current_vertex->get_name(), region_v->get_name(), distance);
                } else {
                    updateDistanceMatrix(current_vertex->get_name(), region_v->get_name(), DBL_MAX);
                }
            }
        }
    }

    // use least common ancestor to complete the distance matrix.
    obtainIndirectHopDistance();

    // print info.
    if (_parameter->get_row_height() != 0) {
        _log->printDouble("Search Window Edge / Row Height",
                          _parameter->get_side_length() / _parameter->get_row_height(), 1);
    }
    if (region_cnt != 0) {
        _log->printDouble("Search Window Average Vertexes(include the element itself)",
                          region_vertexes_cnt / region_cnt, 1);
    }
    _log->printPair("Distance Matrix", _distance_matrix.size(), _distance_matrix.size(), 1);
}

/**
 * @description: Modify the distance matrix of vertex_1, vertex_2, new_vertex.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::modifyDidstanceMatrix(SequentialVertex* vertex_1, SequentialVertex* vertex_2,
                                                SequentialVertex* new_vertex) {
    std::map<std::string, double> new_map = obtainDistanceMap(vertex_1, vertex_2);
    new_map.emplace(new_vertex->get_name(), DBL_MAX);
    deleteVertexDistanceMatrix(vertex_1);
    deleteVertexDistanceMatrix(vertex_2);
    addVertexDistanceMatrix(new_vertex->get_name(), new_map);
}

/**
 * @description: delete the information of vertex in _distance_matrix.
 * @param {SequentialVertex*} vertex
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::deleteVertexDistanceMatrix(SequentialVertex* vertex) {
    auto map_iter = _distance_matrix.find(vertex->get_name());
    std::map<std::string, double> vertex_map;
    if (map_iter != _distance_matrix.end()) {
        vertex_map = map_iter->second;

        for (auto vertex_iter : vertex_map) {
            std::string name = vertex_iter.first;
            // delete the two vertexes.
            deleteDistanceMatrix(name, vertex->get_name());
        }
    } else {
        _log->error(vertex->get_name() + " doesn't int _distance_matrix", 1, 0);
    }
    _distance_matrix.erase(map_iter);
}

/**
 * @description: delete delete_vertex in modify_vertex 's map /
 * @param {SequentialVertex*} modify_vertex
 * @param {SequentialVertex*} delete_vertex
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::deleteDistanceMatrix(std::string modify_name, std::string delete_name) {
    auto it = _distance_matrix.find(modify_name);
    if (it != _distance_matrix.end()) {
        auto& modify_map = (*it).second;
        auto delete_iter = modify_map.find(delete_name);
        if (delete_iter != modify_map.end()) {
            modify_map.erase(delete_iter);
        } else {
            _log->error(delete_name + " doesn't in the " + modify_name + "'s map", 1, 0);
        }
    } else {
        _log->error(modify_name + " doesn't in the _distance_matrix", 1, 0);
    }
}

/**
 * @description: Add the new vertex to the distance map.
 * @param {string} vertex_name
 * @param {map<std::string, double>} vertex_map
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::addVertexDistanceMatrix(std::string vertex_name, std::map<std::string, double> vertex_map) {
    for (auto pair : vertex_map) {
        std::string v_name = pair.first;
        double distance = pair.second;
        updateDistanceMatrix(vertex_name, v_name, distance);
    }
}

/**
 * @description: return the fusion distance map.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
std::map<std::string, double> SkewConstraintGraph::obtainDistanceMap(SequentialVertex* vertex_1,
                                                                     SequentialVertex* vertex_2) {
    std::map<std::string, double> new_map;
    auto map_iter1 = _distance_matrix.find(vertex_1->get_name());
    auto map_iter2 = _distance_matrix.find(vertex_2->get_name());
    std::map<std::string, double> map_1, map_2;
    if (map_iter1 != _distance_matrix.end()) {
        map_1 = map_iter1->second;
    } else {
        _log->error(vertex_1->get_name() + " does not in _distance_matrix", 1, 0);
    }
    if (map_iter2 != _distance_matrix.end()) {
        map_2 = map_iter2->second;
    } else {
        _log->error(vertex_2->get_name() + " does not in _distance_matrix", 1, 0);
    }
    for (auto vertex_iter1 : map_1) {
        std::string v1_name = vertex_iter1.first;
        if (v1_name == vertex_1->get_name()) {
            continue;
        }
        if (v1_name == vertex_2->get_name()) {
            continue;
        }
        bool is_exist = false;
        double v2_distance = 0.0;
        for (auto vertex_iter2 : map_2) {
            std::string v2_name = vertex_iter2.first;
            if (v2_name == vertex_2->get_name()) {
                continue;
            }
            if (v2_name == v1_name) {
                is_exist = true;
                v2_distance = vertex_iter2.second;
                break;
            }
        }
        if (is_exist) {
            new_map.emplace(v1_name, (vertex_iter1.second + v2_distance) / 2);
        } else {
            new_map.emplace(v1_name, vertex_iter1.second);
        }
    }
    for (auto vertex_iter2 : map_2) {
        std::string v2_name = vertex_iter2.first;
        if (v2_name == vertex_2->get_name()) {
            continue;
        }
        if (v2_name == vertex_1->get_name()) {
            continue;
        }
        bool is_exist = false;
        for (auto vertex_iter1 : map_1) {
            std::string v1_name = vertex_iter1.first;
            if (v1_name == vertex_1->get_name()) {
                continue;
            }
            if (v1_name == v2_name) {
                is_exist = true;
                break;
            }
        }
        if (!is_exist) {
            new_map.emplace(v2_name, vertex_iter2.second);
        }
    }
    return new_map;
}

/**
 * @description: Setting the region of a vertex.
 * @param {SequentialVertex*} vertex
 * @return {*}
 * @author: ShiJian Chen
 */
std::pair<Point<DBU>, Point<DBU>> SkewConstraintGraph::obtainSearchRegion(SequentialVertex* vertex) {
    // debug.
    // if (vertex->get_node()->isBuffer()) {
    //     DBU side_length = 160000;
    //     Point<DBU> vertex_coord = vertex->get_node()->get_coord();
    //     Point<DBU> lower_point(vertex_coord.get_x() - side_length / 2, vertex_coord.get_y() - side_length / 2);
    //     Point<DBU> upper_point(vertex_coord.get_x() + side_length / 2, vertex_coord.get_y() + side_length / 2);
    //     std::pair<Point<DBU>, Point<DBU>> region(lower_point, upper_point);
    //     return region;
    // }

    DBU side_length = _parameter->get_side_length();
    Point<DBU> vertex_coord = vertex->get_node()->get_coord();
    Point<DBU> lower_point(vertex_coord.get_x() - side_length / 2, vertex_coord.get_y() - side_length / 2);
    Point<DBU> upper_point(vertex_coord.get_x() + side_length / 2, vertex_coord.get_y() + side_length / 2);
    std::pair<Point<DBU>, Point<DBU>> region(lower_point, upper_point);
    return region;
}

/**
 * @description: Calculate the planning distance between two vertexes.
 * @param {SequentialVertex*} vertex_1
 * @param {SequentialVertex*} vertex_2
 * @param {double} skew
 * @return {*}
 * @author: ShiJian Chen
 */
double SkewConstraintGraph::calculateDistance(SequentialVertex* vertex_1, SequentialVertex* vertex_2, double skew) {
    int normal_x = 0;
    int normal_y = 0;
    double normal_skew = 0.0;
    SequentialElement* element_1 = nullptr;
    SequentialElement* element_2 = nullptr;

    if (vertex_1 && vertex_2) {
        element_1 = vertex_1->get_node();
        element_2 = vertex_2->get_node();
    } else {
        _log->error("Vertex is not existed", 1, 0);
    }

    if (element_1 && element_2) {
        normal_x = fabs(element_1->get_coord().get_x() - element_2->get_coord().get_x()) / _parameter->get_core_x();
        normal_y = fabs(element_1->get_coord().get_y() - element_2->get_coord().get_y()) / _parameter->get_core_y();
    } else {
        _log->error("Element is not existed", 1, 0);
    }
    normal_skew = skew / _parameter->get_max_required_skew();

    double distance = normal_x + normal_y + (_parameter->get_lamda() * normal_skew);
    return distance;
}

/**
 * @description: Update the distance matrix(Graph).
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::updateDistanceMatrix(std::string v1_name, std::string v2_name, double distance) {
    auto v1_iter = _distance_matrix.find(v1_name);
    if (v1_iter == _distance_matrix.end()) {
        std::map<std::string, double> current_map{std::make_pair(v2_name, distance)};
        _distance_matrix.emplace(v1_name, current_map);
    } else {
        auto& current_map = (*v1_iter).second;
        current_map[v2_name] = distance;
    }

    auto v2_iter = _distance_matrix.find(v2_name);
    if (v2_iter == _distance_matrix.end()) {
        std::map<std::string, double> current_map{std::make_pair(v1_name, distance)};
        _distance_matrix.emplace(v2_name, current_map);
    } else {
        auto& current_map = (*v2_iter).second;
        current_map[v1_name] = distance;
    }
}

/**
 * @description: Calculate the distance between two indirect vertexes.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::obtainIndirectHopDistance() {
    std::unordered_map<std::string, std::string> vertex_to_ancestor;
    std::unordered_map<std::string, bool> vertex_visited;
    std::stack<SequentialVertex*> stack;

    // init the vertex_to_ancestor and vertex_visited.
    for (auto pair : _sequential_vertexes) {
        std::string name = pair.first;
        vertex_to_ancestor[name] = name;
        vertex_visited[name] = false;
    }

    // add virtual root.
    SequentialCluster virtual_element("virtual_root");
    SequentialVertex virtual_root(&virtual_element);
    std::vector<SequentialEdge*> virtual_edges;
    for (auto pair : _start_vertexes) {
        SequentialEdge* virtual_edge = new SequentialEdge(&virtual_root, pair.second);
        virtual_edges.push_back(virtual_edge);
        virtual_root.add_sink_edge(virtual_edge);
    }
    vertex_to_ancestor[virtual_root.get_name()] = virtual_root.get_name();
    vertex_visited[virtual_root.get_name()] = false;

    // modify pi's skew source.
    for (auto pair : _sequential_vertexes) {
        auto start_element = pair.second->get_node();
        start_element->add_name_to_skew("virtual_root", 0);
    }

    // least common ancestor to arrange the distance.
    leastCommonAncestorDFS(&virtual_root, stack, vertex_to_ancestor, vertex_visited);

    // delete the virtual_edges.
    for (auto edge : virtual_edges) {
        delete edge;
    }
}

/**
 * @description: Use LCA to complete the distance.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::leastCommonAncestorDFS(SequentialVertex* root, std::stack<SequentialVertex*>& stack,
                                                 std::unordered_map<std::string, std::string>& vertex_to_ancestor,
                                                 std::unordered_map<std::string, bool>& vertex_visited) {
    // root vertex and it's sink should both in stack.
    stack.push(root);

    // Termination condition.
    if (root->get_sink_edges().empty()) {
        // stack.pop();
        return;
    }

    for (auto edge : root->get_sink_edges()) {
        auto current_vertex = edge->get_sink_vertex();
        // continue if current_vertex has been visited.
        if (vertex_visited[current_vertex->get_name()]) {
            continue;
        }

        leastCommonAncestorDFS(current_vertex, stack, vertex_to_ancestor, vertex_visited);

        // Backtracking.
        // not consider the pi po case.
        auto destination_iter = _distance_matrix.find(current_vertex->get_name());
        if (destination_iter != _distance_matrix.end()) {
            auto& destination_map = (*destination_iter).second;
            for (auto dest_pair : destination_map) {
                std::string dest_v_name = dest_pair.first;
                if (vertex_visited[dest_v_name] && dest_pair.second == DBL_MAX) {
                    double skew_1, skew_2;
                    std::string ancestor_name = findAncestor(dest_v_name, vertex_to_ancestor, skew_1);
                    skew_2 = skewAccumulation(stack, ancestor_name);
                    SequentialVertex* dest_vertex = get_existent_vertex(dest_v_name);
                    if (!dest_vertex) {
                        _log->error(dest_v_name + " is not a sequential vertex", 1, 0);
                    }
                    double distance = calculateDistance(current_vertex, dest_vertex, std::abs(skew_1 - skew_2));
                    updateDistanceMatrix(current_vertex->get_name(), dest_vertex->get_name(), distance);
                }
            }
        } else {
            // _log->warn("_distance_matrix has not the " + current_vertex->get_name(), 1, 0);
        }

        vertex_visited[current_vertex->get_name()] = true;
        vertex_to_ancestor[current_vertex->get_name()] = root->get_name();
        // here pop the sink.
        stack.pop();
    }
}

/**
 * @description: obtain sum skew from one path.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
std::string SkewConstraintGraph::findAncestor(std::string vertex_name,
                                              std::unordered_map<std::string, std::string> vertex_to_ancestor,
                                              double& skew) {
    std::string current_name = vertex_name;
    SequentialVertex* current_vertex;
    double sum_skew = 0.0;

    while (current_name != vertex_to_ancestor[current_name]) {
        current_vertex = get_existent_vertex(current_name);
        if (!current_vertex) {
            _log->error(current_name + " is not a sequential vertex", 1, 0);
        } else {
            std::string ancestor_name = vertex_to_ancestor[current_name];
            double current_skew = current_vertex->get_node()->get_skew(ancestor_name);
            if (current_skew == DBL_MAX) {
                _log->error(current_name + " has no skew from " + ancestor_name, 1, 0);
            }
            sum_skew += current_skew;
            current_name = ancestor_name;
        }
    }
    skew = sum_skew;
    return current_name;
}

/**
 * @description: obtain sum skew from another path.
 * @param {stack<std::string>} stack
 * @param {SequentialVertex*} ancestor_name
 * @return {*}
 * @author: ShiJian Chen
 */
double SkewConstraintGraph::skewAccumulation(std::stack<SequentialVertex*> stack, std::string ancestor_name) {
    double sum_skew = 0.0;
    while (!stack.empty()) {
        auto current_vertex = stack.top();
        std::string current_name = current_vertex->get_name();
        if (current_name == ancestor_name) {
            break;
        }

        // take the source_name.
        stack.pop();
        if (stack.empty()) {
            _log->error("Cannot find the ancestor", 1, 0);
        } else {
            auto source_vertex = stack.top();
            std::string source_name = source_vertex->get_name();

            double current_skew = current_vertex->get_node()->get_skew(source_name);
            if (current_skew == DBL_MAX) {
                _log->error(current_name + " has no skew from " + source_name, 1, 0);
            }
            sum_skew += current_skew;
        }
    }
    return sum_skew;
}

/**
 * @description: Execute the Sequential Cluster, return the clusters result.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
std::map<std::string, std::vector<const SequentialElement*>> SkewConstraintGraph::makeVertexesFusion() {
    std::list<SequentialCluster*> clusters;

    // TODO : loop condition should be considered.
    while (1) {
        std::pair<std::string, std::string> vertex_pair = obtainMinDistancePair();
        std::string v1_name = vertex_pair.first;
        std::string v2_name = vertex_pair.second;
        if (v1_name == "end" && v2_name == "end") {
            break;
        }
        SequentialVertex* vertex_1 = get_existent_vertex(v1_name);
        SequentialVertex* vertex_2 = get_existent_vertex(v2_name);
        if (!vertex_1 || !vertex_2) {
            _log->error("Vertex is not in the graph", 1, 0);
        }

        // ring case.
        if (findRing(vertex_1, vertex_2)) {
            // reset the vertex1, vertex_2 distance.
            deleteDistanceMatrix(vertex_1->get_name(), vertex_2->get_name());
            deleteDistanceMatrix(vertex_2->get_name(), vertex_1->get_name());
            continue;
        } else {
            SequentialElement* element_1 = vertex_1->get_node();
            SequentialElement* element_2 = vertex_2->get_node();
            if (!element_1 || !element_2) {
                _log->error("Element is not existed", 1, 0);
            } else {
                if ((element_1->isFlipFlop() || element_1->isBuffer()) &&
                    (element_2->isFlipFlop() || element_2->isBuffer())) {
                    SequentialCluster* cluster = new SequentialCluster("clus_" + element_1->get_name());

                    if (element_1->isFlipFlop()) {
                        SequentialFlipFlop* flipflop_1 = static_cast<SequentialFlipFlop*>(element_1);
                        flipflop_1->set_cluster(cluster);
                    }
                    if (element_2->isFlipFlop()) {
                        SequentialFlipFlop* flipflop_2 = static_cast<SequentialFlipFlop*>(element_2);
                        flipflop_2->set_cluster(cluster);
                    }

                    if (element_1->isBuffer()) {
                        SequentialBuffer* buffer_1 = static_cast<SequentialBuffer*>(element_1);
                        buffer_1->set_cluster(cluster);
                    }

                    if (element_2->isBuffer()) {
                        SequentialBuffer* buffer_2 = static_cast<SequentialBuffer*>(element_2);
                        buffer_2->set_cluster(cluster);
                    }

                    // add flipflop.
                    cluster->addSubElement(element_1);
                    cluster->addSubElement(element_2);
                    clusters.push_back(cluster);
                    SequentialVertex* cluster_vertex = new SequentialVertex(cluster);
                    updateTwoVertexesFusion(vertex_1, vertex_2, cluster_vertex);
                } else if ((element_1->isFlipFlop() || element_1->isBuffer()) && element_2->isCluster()) {
                    SequentialCluster* cluster = static_cast<SequentialCluster*>(element_2);
                    cluster->set_name(cluster->get_name() + "+");
                    // cluster size control.
                    if (cluster->get_sub_size() == _parameter->get_clus_size()) {
                        deleteDistanceMatrix(vertex_1->get_name(), vertex_2->get_name());
                        deleteDistanceMatrix(vertex_2->get_name(), vertex_1->get_name());
                        continue;
                    }
                    if (element_1->isFlipFlop()) {
                        SequentialFlipFlop* flipflop = static_cast<SequentialFlipFlop*>(element_1);
                        flipflop->set_cluster(cluster);
                    }
                    if (element_1->isBuffer()) {
                        SequentialBuffer* buffer = static_cast<SequentialBuffer*>(element_1);
                        buffer->set_cluster(cluster);
                    }

                    cluster->addSubElement(element_1);
                    SequentialVertex* cluster_vertex = new SequentialVertex(cluster);
                    updateTwoVertexesFusion(vertex_1, vertex_2, cluster_vertex);

                } else if (element_1->isCluster() && element_2->isFlipFlop()) {
                    SequentialCluster* cluster = static_cast<SequentialCluster*>(element_1);
                    cluster->set_name(cluster->get_name() + "+");
                    // cluster size control.
                    if (cluster->get_sub_size() == _parameter->get_clus_size()) {
                        deleteDistanceMatrix(vertex_1->get_name(), vertex_2->get_name());
                        deleteDistanceMatrix(vertex_2->get_name(), vertex_1->get_name());
                        continue;
                    }
                    if (element_2->isFlipFlop()) {
                        SequentialFlipFlop* flipflop = static_cast<SequentialFlipFlop*>(element_2);
                        flipflop->set_cluster(cluster);
                    }
                    if (element_2->isBuffer()) {
                        SequentialBuffer* buffer = static_cast<SequentialBuffer*>(element_2);
                        buffer->set_cluster(cluster);
                    }
                    cluster->addSubElement(element_2);
                    SequentialVertex* cluster_vertex = new SequentialVertex(cluster);
                    updateTwoVertexesFusion(vertex_2, vertex_1, cluster_vertex);
                } else if (element_1->isCluster() && element_2->isCluster()) {
                    SequentialCluster* cluster = static_cast<SequentialCluster*>(element_1);
                    cluster->set_name(cluster->get_name() + "+");
                    SequentialCluster* cluster_2 = static_cast<SequentialCluster*>(element_2);
                    if (cluster->get_sub_size() + cluster_2->get_sub_size() > _parameter->get_clus_size()) {
                        deleteDistanceMatrix(vertex_1->get_name(), vertex_2->get_name());
                        deleteDistanceMatrix(vertex_2->get_name(), vertex_1->get_name());
                        continue;
                    }
                    auto sub_elements = cluster_2->get_all_element_vec();
                    for (auto flipflop : sub_elements) {
                        cluster->addSubElement(flipflop);
                    }
                    // delete the cluster_2.
                    auto it =
                        std::find_if(clusters.begin(), clusters.end(), [cluster_2](const SequentialCluster* cluster) {
                            return cluster->get_name() == cluster_2->get_name();
                        });
                    if (it != clusters.end()) {
                        clusters.erase(it);
                    } else {
                        _log->error(cluster_2->get_name() + " doesn't in clusters", 1, 0);
                    }
                    SequentialVertex* cluster_vertex = new SequentialVertex(cluster);
                    updateTwoVertexesFusion(vertex_1, vertex_2, cluster_vertex);
                } else {
                    _log->error("Element type is incorrect during vertexes fusion", 1, 0);
                }
            }
        }
    }

    std::map<std::string, std::vector<const SequentialElement*>> final_clusters;
    for (auto cluster : clusters) {
        std::vector<const SequentialElement*> element_vec = cluster->get_all_element_vec();
        final_clusters.emplace(cluster->get_name(), element_vec);
    }

    // delete the clusters.
    for (auto cluster : clusters) {
        delete cluster;
    }
    clusters.clear();
    return final_clusters;
}

/**
 * @description: Take out the min distance's vertex pair.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
std::pair<std::string, std::string> SkewConstraintGraph::obtainMinDistancePair() {
    std::pair<std::string, std::string> vertex_pair{"end", "end"};
    double min_distance = DBL_MAX;
    for (auto pair_1 : _distance_matrix) {
        auto v1_name = pair_1.first;
        auto v1_map = pair_1.second;
        for (auto pair_2 : v1_map) {
            auto v2_name = pair_2.first;
            // Traversing only half a matrix
            if (v1_name == v2_name) {
                break;
            }
            double current_distance = pair_2.second;
            if (current_distance < min_distance) {
                min_distance = current_distance;
                vertex_pair = std::make_pair(v1_name, v2_name);
            }
        }
    }
    return vertex_pair;
}

/**
 * @description: Finish the two vertexes fusion.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::updateTwoVertexesFusion(SequentialVertex* vertex_1, SequentialVertex* vertex_2,
                                                  SequentialVertex* new_vertex) {
    // change the connection relationship of the graph.
    updateGraphConnection(vertex_1, vertex_2, new_vertex);

    // modify the ancestors and descendants.
    modifyVertexAncestors(vertex_1, vertex_2, new_vertex);
    modifyVertexDescendants(vertex_1, vertex_2, new_vertex);

    // update the distance matrix.
    modifyDidstanceMatrix(vertex_1, vertex_2, new_vertex);

    // delete the two fusion vertex
    deleteSequentialVertex(vertex_1->get_name());
    deleteSequentialVertex(vertex_2->get_name());

    // add the new_vertex.
    add_sequential_vertex(new_vertex);
}

/**
 * @description: Delete the direct hop connection.
 * @param {SequentialVertex*} vertex_1
 * @param {SequentialVertex*} vertex_2
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::deleteDirectHopConnection(SequentialVertex* vertex_1, SequentialVertex* vertex_2) {
    auto src_e1 = vertex_1->get_src_edge(vertex_2);
    if (src_e1) {
        vertex_1->removeSrcEdge(src_e1);
        vertex_2->removeSinkEdge(src_e1);
        deleteSequentialEdge(src_e1->get_name());
    }

    auto src_e2 = vertex_2->get_src_edge(vertex_1);
    if (src_e2) {
        vertex_2->removeSrcEdge(src_e2);
        vertex_1->removeSinkEdge(src_e2);
        deleteSequentialEdge(src_e2->get_name());
    }
}

/**
 * @description: Update the source connetion.
 * @param {SequentialVertex*} vertex
 * @param {SequentialVertex*} new_vertex
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::updateSourceConnection(SequentialVertex* vertex, SequentialVertex* new_vertex) {
    std::vector<SequentialEdge*> src_edges = vertex->get_src_edges();
    for (auto edge : src_edges) {
        auto src_v = edge->get_src_vertex();

        // avoid repeat add.
        auto exist_e1 = new_vertex->get_src_edge(src_v);
        if (exist_e1) {
            double merge_skew = (edge->get_setup_skew() + exist_e1->get_setup_skew()) / 2;
            exist_e1->set_skew(merge_skew);
            src_v->removeSinkEdge(edge);
            vertex->removeSrcEdge(edge);
            deleteSequentialEdge(edge->get_name());
            continue;
        }

        // avoid reverse add.
        auto exist_e2 = new_vertex->get_sink_edge(src_v);
        if (exist_e2) {
            double skew_1 = edge->get_setup_skew();
            double skew_2 = exist_e2->get_setup_skew();

            if (skew_1 > skew_2) {
                double merge_skew = skew_1 - skew_2;
                edge->set_skew(merge_skew);

                new_vertex->removeSinkEdge(exist_e2);
                src_v->removeSrcEdge(exist_e2);
                deleteSequentialEdge(exist_e2->get_name());
            } else {
                double merge_skew = skew_2 - skew_1;
                exist_e2->set_skew(merge_skew);

                src_v->removeSinkEdge(edge);
                vertex->removeSrcEdge(edge);
                deleteSequentialEdge(edge->get_name());
                continue;
            }
        }

        vertex->removeSrcEdge(edge);
        src_v->removeSinkEdge(edge);
        SequentialEdge* new_edge = new SequentialEdge(src_v, new_vertex);
        new_edge->set_skew(edge->get_setup_skew());
        src_v->add_sink_edge(new_edge);
        new_vertex->add_src_edge(new_edge);
        add_sequential_edge(new_edge);

        deleteSequentialEdge(edge->get_name());
    }
}

/**
 * @description: Update the sink connection.
 * @param {SequentialVertex*} vertex
 * @param {SequentialVertex*} new_vertex
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::updateSinkConnection(SequentialVertex* vertex, SequentialVertex* new_vertex) {
    std::vector<SequentialEdge*> sink_edges = vertex->get_sink_edges();
    for (auto edge : sink_edges) {
        auto sink_v = edge->get_sink_vertex();

        // avoid repeat add.
        auto exist_e1 = new_vertex->get_sink_edge(sink_v);
        if (exist_e1) {
            double merge_skew = (edge->get_setup_skew() + exist_e1->get_setup_skew()) / 2;
            exist_e1->set_skew(merge_skew);
            vertex->removeSinkEdge(edge);
            sink_v->removeSrcEdge(edge);
            deleteSequentialEdge(edge->get_name());
            continue;
        }

        // avoid reverse add.
        auto exist_e2 = new_vertex->get_src_edge(sink_v);
        if (exist_e2) {
            double skew_1 = edge->get_setup_skew();
            double skew_2 = exist_e2->get_setup_skew();

            if (skew_1 > skew_2) {
                double merge_skew = skew_1 - skew_2;
                edge->set_skew(merge_skew);

                new_vertex->removeSrcEdge(exist_e2);
                sink_v->removeSinkEdge(exist_e2);
                deleteSequentialEdge(exist_e2->get_name());
            } else {
                double merge_skew = skew_2 - skew_1;
                exist_e2->set_skew(merge_skew);

                vertex->removeSinkEdge(edge);
                sink_v->removeSrcEdge(edge);
                deleteSequentialEdge(edge->get_name());
                continue;
            }
        }

        vertex->removeSinkEdge(edge);
        sink_v->removeSrcEdge(edge);
        SequentialEdge* new_edge = new SequentialEdge(new_vertex, sink_v);
        new_edge->set_skew(edge->get_setup_skew());
        sink_v->add_src_edge(new_edge);
        new_vertex->add_sink_edge(new_edge);
        add_sequential_edge(new_edge);

        deleteSequentialEdge(edge->get_name());
    }
}

/**
 * @description: Upadate the graph connection.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::updateGraphConnection(SequentialVertex* vertex_1, SequentialVertex* vertex_2,
                                                SequentialVertex* new_vertex) {
    deleteDirectHopConnection(vertex_1, vertex_2);
    updateSourceConnection(vertex_1, new_vertex);
    updateSourceConnection(vertex_2, new_vertex);
    updateSinkConnection(vertex_1, new_vertex);
    updateSinkConnection(vertex_2, new_vertex);
}

/**
 * @description: Update the graph connection base on the cluster.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::updateGraphConnectionFromSubGraph(std::string cluster_name,
                                                            std::vector<const SequentialElement*> fusion_flipflops,
                                                            SequentialElement* buffer) {
    SequentialCluster* cluster = new SequentialCluster(cluster_name);

    SequentialVertex* flipflop_v1 = get_existent_vertex(fusion_flipflops[0]->get_name());
    SequentialVertex* flipflop_v2 = get_existent_vertex(fusion_flipflops[1]->get_name());
    SequentialVertex* cluster_vertex = new SequentialVertex(cluster);
    updateGraphConnection(flipflop_v1, flipflop_v2, cluster_vertex);
    deleteSequentialVertex(flipflop_v1->get_name());
    deleteSequentialVertex(flipflop_v2->get_name());
    add_sequential_vertex(cluster_vertex);

    for (size_t i = 2; i < fusion_flipflops.size(); ++i) {
        SequentialVertex* flipflop_vertex = get_existent_vertex(fusion_flipflops[i]->get_name());
        SequentialVertex* past_vertex = get_existent_vertex(cluster->get_name());
        cluster->set_name(cluster->get_name() + "+");
        SequentialVertex* cluster_vertex = new SequentialVertex(cluster);
        updateGraphConnection(flipflop_vertex, past_vertex, cluster_vertex);
        deleteSequentialVertex(past_vertex->get_name());
        deleteSequentialVertex(flipflop_vertex->get_name());
        add_sequential_vertex(cluster_vertex);
    }

    modifyVertexName(cluster->get_name(), buffer);

    delete cluster;
}

/**
 * @description: update all ancestors of the vertex_1 and the vertex_2, modifying their descentant. Add
 * fusion_vertex to the list.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::modifyVertexAncestors(SequentialVertex* vertex_1, SequentialVertex* vertex_2,
                                                SequentialVertex* new_vertex) {
    auto v1_ancestors = vertex_1->get_ancestors();
    for (auto ancestor : v1_ancestors) {
        // skip the vertex_2.
        if (*ancestor == *vertex_2) {
            continue;
        }
        // new_vertex add the ancestor.
        new_vertex->add_ancestor(ancestor);

        ancestor->add_descendant(new_vertex);
        ancestor->updateDescendants(vertex_2->get_descendants());

        // must be here.
        ancestor->removeDescendant(vertex_1);
        ancestor->removeDescendant(vertex_2);
    }
    auto v2_ancestors = vertex_2->get_ancestors();
    for (auto ancestor : v2_ancestors) {
        // skip the vertex_1.
        if (*ancestor == *vertex_1) {
            continue;
        }
        // new_vertex add the ancestor.
        new_vertex->add_ancestor(ancestor);

        ancestor->add_descendant(new_vertex);
        ancestor->updateDescendants(vertex_1->get_descendants());

        // must be here.
        ancestor->removeDescendant(vertex_1);
        ancestor->removeDescendant(vertex_2);
    }
}

/**
 * @description:  update all descentants of the vertex_1 and the vertex_2, modifying their ancestor. Add fusion_vertex
 * to the list.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::modifyVertexDescendants(SequentialVertex* vertex_1, SequentialVertex* vertex_2,
                                                  SequentialVertex* new_vertex) {
    auto v1_descendants = vertex_1->get_descendants();
    for (auto descendant : v1_descendants) {
        // skip the vertex_2.
        if (*descendant == *vertex_2) {
            continue;
        }
        // new_vertex add the descendant.
        new_vertex->add_descendant(descendant);

        descendant->add_ancestor(new_vertex);
        descendant->updateAncestors(vertex_2->get_ancestors());

        // must be here.
        descendant->removeAncestor(vertex_1);
        descendant->removeAncestor(vertex_2);
    }

    auto v2_descendants = vertex_2->get_descendants();
    for (auto descendant : v2_descendants) {
        // skip the vertex_1.
        if (*descendant == *vertex_1) {
            continue;
        }
        new_vertex->add_descendant(descendant);

        descendant->add_ancestor(new_vertex);
        descendant->updateAncestors(vertex_1->get_ancestors());

        // must be here.
        descendant->removeAncestor(vertex_1);
        descendant->removeAncestor(vertex_2);
    }
}

/**
 * @description: Modify the vertex name in graph.
 * @param {string} old_name
 * @param {SequentialElement*} buffer
 * @return {*}
 * @author: ShiJian Chen
 */
void SkewConstraintGraph::modifyVertexName(std::string old_name, SequentialElement* buffer) {
    SequentialVertex* old_vertex = get_existent_vertex(old_name);
    SequentialVertex* new_vertex = new SequentialVertex(buffer);
    updateSourceConnection(old_vertex, new_vertex);
    updateSinkConnection(old_vertex, new_vertex);

    deleteSequentialVertex(old_vertex->get_name());
    add_sequential_vertex(new_vertex);
}

}  // namespace itdp