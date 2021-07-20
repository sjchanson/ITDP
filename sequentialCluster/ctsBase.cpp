#include "ctsBase.h"

#include <bits/stdc++.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <iomanip>
#include <sstream>

#include "omp.h"

ClusterVertex::ClusterVertex() : _name(""), _level(INT_MAX), _point(nullptr), _skew(0) {}

ClusterVertex::ClusterVertex(sequentialFlipFlop *flipflop) : ClusterVertex() {
    _point = new Point<DBU>(flipflop->get_coord().x, flipflop->get_coord().y);
    _name = flipflop->get_name();
}

ClusterVertex::ClusterVertex(DBU x, DBU y, string name) : ClusterVertex() {
    _point = new Point<DBU>(x, y);
    _name = name;
}

ctsEdge::ctsEdge() : _src(nullptr), _sink(nullptr) {}

ctsEdge::ctsEdge(ClusterVertex *src, ClusterVertex *sink) : ctsEdge() {
    _src = src;
    _sink = sink;
}

ctsEdge::~ctsEdge() {
    _src = nullptr;
    _sink = nullptr;
}

ClusterVertexPair::ClusterVertexPair() : vertex_1(nullptr), vertex_2(nullptr), distance(DBL_MAX) {}

ClusterVertexPair::ClusterVertexPair(ClusterVertex *v1, ClusterVertex *v2) : ClusterVertexPair() {
    vertex_1 = v1;
    vertex_2 = v2;
    distance =
        abs(v1->get_point()->getX() - v2->get_point()->getX()) + abs(v1->get_point()->getY() - v2->get_point()->getY());
}

ctsSingleClus::ctsSingleClus() : _name(""), _transition_point_cnt(0), _top_level(0), _root_vertex(nullptr) {}

ctsSingleClus::ctsSingleClus(
    std::unordered_set<sequentialFlipFlop *, sequentialFlipFlop::basePtrHash, sequentialFlipFlop::basePtrEqual>
        flipflops,
    string name)
    : ctsSingleClus() {
    _name = name;
    _origin_flipflops = flipflops;
}

ctsSingleClus::~ctsSingleClus() {
    _binary_tree.clear();
    _origin_flipflops.clear();
}

void ctsSingleClus::analyseSinkRelationship(ofstream &sheet_stream) {
    // print the flipflop sheet.
    stringstream feed;
    feed.precision(5);

    for (auto cur_f : _origin_flipflops) {
        // Judge if the source flipflop is with the class.
        bool is_existed = false;
        for (auto src_f : cur_f->get_source()) {
            if (src_f->get_type() != 3) {
                continue;
            }
            auto it = _origin_flipflops.find(dynamic_cast<sequentialFlipFlop *>(src_f));
            if (it != _origin_flipflops.end()) {
                is_existed = true;
                break;
            }
        }
        if (is_existed) {
            _binary_flipflops.push_back(cur_f);
        } else {
            cur_f->resetSkew();  // cur vertex has no source in the sub graph.
            _binary_flipflops.push_back(cur_f);
        }
    }

    // sort _binary_flipflops by skew.
    _binary_flipflops.sort(
        [](sequentialFlipFlop *f1, sequentialFlipFlop *f2) { return f1->get_max_skew() > f2->get_max_skew(); });

    std::list<ClusterVertex *> level_up_list;
    std::list<ClusterVertex *> remain_list;
    std::list<ClusterVertex *> tmp_list;
    // pick the bigger skew pair to make vertex, and make the union vertex.
    while (!_binary_flipflops.empty()) {
        auto flipflop = *_binary_flipflops.begin();
        if (flipflop->get_max_skew() == 0.0) {
            break;
        }
        auto src_flipflop = flipflop->get_max_skew_flipflop_source();  // the src must exist because the max skew exist.
        auto it = std::find_if(_binary_flipflops.begin(), _binary_flipflops.end(),
                               [src_flipflop](const sequentialFlipFlop *f) { return *src_flipflop == *f; });
        if (it != _binary_flipflops.end()) {
            // print sheet.
            feed << "," << src_flipflop->get_name() << "," << flipflop->get_name() << ","
                 << flipflop->get_input_pin()->lateSlk << std::endl;

            _binary_flipflops.remove_if([src_flipflop](const sequentialFlipFlop *f) { return *src_flipflop == *f; });
            _binary_flipflops.remove_if([flipflop](const sequentialFlipFlop *f) { return *flipflop == *f; });
            // make the vertex.
            auto up_vertex = buildVertexes(src_flipflop, flipflop, _transition_point_cnt++);
            level_up_list.push_back(up_vertex);
            continue;
        }

        // print sheet.
        // find origin flipflop info.
        auto origin_it = _origin_flipflops.find(flipflop);
        feed << ","
             << "," << flipflop->get_name() << "," << (*origin_it)->get_input_pin()->lateSlk << std::endl;

        _binary_flipflops.remove_if([flipflop](const sequentialFlipFlop *f) { return *flipflop == *f; });
    }

    // remain _binary_flipflops become vertexes.
    for (auto remain_flipflop : _binary_flipflops) {
        // print sheet.
        // find origin flipflop info.
        auto origin_it = _origin_flipflops.find(remain_flipflop);
        feed << ","
             << "," << remain_flipflop->get_name() << "," << (*origin_it)->get_input_pin()->lateSlk << std::endl;

        ClusterVertex *cur_vertex = new ClusterVertex(remain_flipflop);

        remain_list.push_back(cur_vertex);
    }

    tmp_list = updateUpLevelVertexes(remain_list, 0);

    // level 0
    for (auto vertex : tmp_list) {
        level_up_list.push_back(vertex);
    }

    while (level_up_list.size() != 1) {
        _top_level++;
        level_up_list = updateUpLevelVertexes(level_up_list, _top_level);
    }
    _root_vertex = *level_up_list.begin();
    _root_vertex->set_level(++_top_level);
    _vertexes.push_back(_root_vertex);

    sheet_stream << feed.str();
    feed.clear();
}

ClusterVertex *ctsSingleClus::buildVertexes(sequentialFlipFlop *from, sequentialFlipFlop *to, int transition_cnt) {
    string transition_name = "transition_" + std::to_string(transition_cnt);

    ClusterVertex *src_vertex = new ClusterVertex(from);
    src_vertex->set_level(0);  // set level.
    // src_vertex->set_skew();  // src skew should add as it is sink.
    ClusterVertex *sink_vertex = new ClusterVertex(to);
    sink_vertex->set_level(0);
    sink_vertex->set_skew(to->get_skew(from));
    _vertexes.push_back(src_vertex);
    _vertexes.push_back(sink_vertex);

    // add transition vertex.
    DBU x_coord = (from->get_coord().x + to->get_coord().x) / 2;
    DBU y_coord = (from->get_coord().y + to->get_coord().y) / 2;
    ClusterVertex *transition_vertex = new ClusterVertex(x_coord, y_coord, transition_name);

    // make graph.
    ctsEdge *edge_1 = new ctsEdge(transition_vertex, src_vertex);
    ctsEdge *edge_2 = new ctsEdge(transition_vertex, sink_vertex);
    transition_vertex->add_sink_edges(edge_1);
    transition_vertex->add_sink_edges(edge_2);
    src_vertex->add_src_edges(edge_1);
    sink_vertex->add_src_edges(edge_2);

    _edges.push_back(edge_1);
    _edges.push_back(edge_2);

    return transition_vertex;
}

std::list<ClusterVertex *> ctsSingleClus::updateUpLevelVertexes(std::list<ClusterVertex *> cur_vertexes, int level) {
    std::list<ClusterVertex *> up_vertex_list;

    for (auto iter = cur_vertexes.begin(); iter != cur_vertexes.end(); ++iter) {
        ClusterVertex *vertex_i = *iter;
        auto tmp_iter = iter;
        for (auto iter_back = ++tmp_iter; iter_back != cur_vertexes.end(); iter_back++) {
            ClusterVertex *vertex_j = *iter_back;
            ClusterVertexPair pair(vertex_i, vertex_j);
            _binary_pair.push_back(pair);
        }
    }

    // sort the _binary_pair according to the distance.
    std::sort(_binary_pair.begin(), _binary_pair.end(),
              [](const ClusterVertexPair &p1, const ClusterVertexPair &p2) { return p1.distance < p2.distance; });

    for (auto pair : _binary_pair) {
        auto v1 = pair.vertex_1;
        auto v2 = pair.vertex_2;
        auto it_1 = std::find_if(cur_vertexes.begin(), cur_vertexes.end(),
                                 [v1](ClusterVertex *v) { return v1->get_name() == v->get_name(); });
        auto it_2 = std::find_if(cur_vertexes.begin(), cur_vertexes.end(),
                                 [v2](ClusterVertex *v) { return v2->get_name() == v->get_name(); });
        if (it_1 != cur_vertexes.end() && it_2 != cur_vertexes.end()) {
            string transition_name = "transition_" + std::to_string(_transition_point_cnt++);
            v1->set_level(level);  // set level.
            _vertexes.push_back(v1);
            v2->set_level(level);  // set level.
            _vertexes.push_back(v2);

            // add transition vertex.
            DBU x_coord = (v1->get_point()->getX() + v2->get_point()->getX()) / 2;
            DBU y_coord = (v1->get_point()->getY() + v2->get_point()->getY()) / 2;
            ClusterVertex *transition_vertex = new ClusterVertex(x_coord, y_coord, transition_name);

            // make graph.
            ctsEdge *edge_1 = new ctsEdge(transition_vertex, v1);
            ctsEdge *edge_2 = new ctsEdge(transition_vertex, v2);
            transition_vertex->add_sink_edges(edge_1);
            transition_vertex->add_sink_edges(edge_2);
            v1->add_src_edges(edge_1);
            v2->add_src_edges(edge_2);

            _edges.push_back(edge_1);
            _edges.push_back(edge_2);

            // add to the up_vertex_list.
            up_vertex_list.push_back(transition_vertex);

            // delete the v1, v2.
            cur_vertexes.erase(it_1);
            cur_vertexes.erase(it_2);
        }
    }
    // there must be one vertex left or no vertex left.
    if (!cur_vertexes.empty()) {
        if (cur_vertexes.size() == 1) {
            auto remain_vertex = *(cur_vertexes.begin());
            up_vertex_list.push_back(remain_vertex);
        } else {
            std::cout << "ERROR In updateUpLevelVertexes." << std::endl;
        }
    }

    return up_vertex_list;
}

void ctsSingleClus::constructPerfectBinaryTree() {
    // Complete perfect binary tree.
    for (auto vertex : _vertexes) {
        if (vertex->get_level() == 1 && vertex->get_sink_edges().empty()) {
            string prefix = "transition_";
            ClusterVertex *t_1 = new ClusterVertex(-1, -1, prefix + std::to_string(_transition_point_cnt++));
            ClusterVertex *t_2 = new ClusterVertex(-1, -1, prefix + std::to_string(_transition_point_cnt++));

            ctsEdge *edge_1 = new ctsEdge(vertex, t_1);
            ctsEdge *edge_2 = new ctsEdge(vertex, t_2);

            vertex->add_sink_edges(edge_1);
            vertex->add_sink_edges(edge_2);
            t_1->add_src_edges(edge_1);
            t_2->add_src_edges(edge_2);

            _vertexes.push_back(t_1);
            _vertexes.push_back(t_2);
            _edges.push_back(edge_1);
            _edges.push_back(edge_2);
        }
    }

    // Construct the prefect binary tree.
    std::queue<ClusterVertex *> queue;
    queue.push(_root_vertex);
    analyseBFS(queue);
}

void ctsSingleClus::analyseBFS(std::queue<ClusterVertex *> &queue) {
    while (!queue.empty()) {
        auto cur_v = queue.front();
        _binary_tree.push_back(cur_v);
        queue.pop();

        auto e_vec = cur_v->get_sink_edges();
        for (auto e : e_vec) {
            auto next_v = e->get_sink();
            queue.push(next_v);
        }
    }
}

ctsBase::ctsBase(std::unordered_set<sequentialCluster *, baseHash, baseEqual> clusters) {
    _clusters = clusters;
    init();
}

ctsBase::~ctsBase() {
    _sub_clusters.clear();
    _clusters.clear();
}

void ctsBase::init() {
    for (auto clus : _clusters) {
        auto sub_flipflops = clus->get_subordinate_flipflops();
        ctsSingleClus *sub_cluster = new ctsSingleClus(sub_flipflops, clus->get_name());
        _sub_clusters.push_back(sub_cluster);
    }

    string path = "sheet";
    string file_name = "simple";
    if (opendir(path.c_str()) == nullptr) {
        if (mkdir(path.c_str(), 0777) == -1) {
            std::cout << "ERROR In Create The Folder." << std::endl;
        }
    }
    ofstream sheet_stream(path + "/" + file_name + ".csv");
    sheet_stream << "CLUSTER_NAME,"
                 << "SOURCE_FLIPFLOP,"
                 << "SINK_FLIPFLOP,"
                 << "SLACK" << std::endl;

    for (auto sub_cluster : _sub_clusters) {
        sheet_stream << sub_cluster->get_name() << std::endl;
        sub_cluster->analyseSinkRelationship(sheet_stream);
        sub_cluster->constructPerfectBinaryTree();
    }
    sheet_stream.close();
}