#include "ctsBase.h"

ctsVertex::ctsVertex() : _id(-1), _coord(-1, -1), _skew(DBL_MAX) {}

ctsVertex::ctsVertex(DBU x, DBU y) : ctsVertex() { _coord = Point<DBU>(_coord.getX(), _coord.getY()); }

ctsSingleClus::ctsSingleClus() {}

ctsSingleClus::ctsSingleClus(std::unordered_set<sequentialFlipFlop *> flipflops) : ctsSingleClus() {
    _origin_flipflops = flipflops;
    init();
}

ctsSingleClus::~ctsSingleClus() {
    _binary_tree.clear();
    _origin_flipflops.clear();
}

void ctsSingleClus::init() {
    splitStructre();
    analyStructre();
}

void ctsSingleClus::splitStructre() {
    for (auto cur_f : _origin_flipflops) {
        // Judge if the source flipflop is with the class.
        bool flag = false;  // flag to identfy if current flipflop has connection.
        for (auto src_f : cur_f->get_source()) {
            if (src_f->get_type() != 3) {
                continue;
            }
            auto it = _origin_flipflops.find(dynamic_cast<sequentialFlipFlop *>(src_f));
            if (it != _origin_flipflops.end()) {
                flag = true;
                auto f1 = dynamic_cast<sequentialFlipFlop *>(src_f);
                auto f2 = dynamic_cast<sequentialFlipFlop *>(cur_f);
                // add topo.
                ctsVertex *src_v = new ctsVertex(f1->get_coord().x, f1->get_coord().y);
                ctsVertex *sink_v = new ctsVertex(f2->get_coord().x, f2->get_coord().y);
                sink_v->set_skew(f2->get_skew(f1));  // get the skew form f1->f2.
                ctsEdge *edge = new ctsEdge(src_v, sink_v);
                src_v->add_sink_edges(edge);
                sink_v->add_src_edges(edge);

                src_v->set_id(_vertexes.size());  // add id.
                _vertexes.push_back(src_v);
                sink_v->set_id(_vertexes.size());  // add id.
                _vertexes.push_back(sink_v);
                _edges.push_back(edge);
            }
        }
        if (flag) {
            continue;
        }
        // no connection case.
        _binary_flipflops.push_back(cur_f);
    }
}

void ctsSingleClus::analyStructre() {
    std::vector<ctsVertex *> start_vertexes;
    for (auto v : _vertexes) {
        if (v->get_src_edges().empty()) {
            start_vertexes.push_back(v);
        }
    }
    std::stack<ctsVertex *> stack;
    std::map<int, bool> is_visited;
    for (auto src_v : start_vertexes) {
        auto e_vec = src_v->get_sink_edges();
        if (e_vec.size() == 1) {  // common connect case.
            auto sink_v = e_vec[0]->get_sink();
            if (sink_v->get_sink_edges().empty()) {
                std::pair<ctsVertex *, ctsVertex *> cur_pair(src_v, sink_v);
                _pairs.emplace(cur_pair);
            } else {  // cascade case.
                _special_cascades.push_back(src_v);

                // DFS for make visited flag.
                stack.push(src_v);
                analyDFS(stack, is_visited);
            }
        } else {  // fork case.
            _special_forks.push_back(src_v);

            // DFS for make visited flag.
            stack.push(src_v);
            analyDFS(stack, is_visited);
        }
    }
}

void ctsSingleClus::analyDFS(std::stack<ctsVertex *> &stack, std::map<int, bool> &is_visited) {
    auto cur_v = stack.top();

    if (is_visited[cur_v->get_id()]) {  // merge case.
        _special_merges.push_back(cur_v);
        stack.pop();
        return;
    }

    is_visited[cur_v->get_id()] = true;

    if (cur_v->get_sink_edges().empty()) {
        stack.pop();
        return;
    }

    auto e_vec = cur_v->get_sink_edges();
    for (auto e : e_vec) {
        stack.push(e->get_sink());
        analyDFS(stack, is_visited);
    }
    stack.pop();
}

ctsBase::ctsBase(std::unordered_set<sequentialCluster *, baseHash, baseEqual> clusters) {
    _clusters = clusters;
    init();
}

ctsBase::~ctsBase() {
    _binary_tree.clear();
    _clusters.clear();
}

void ctsBase::init() {
    for (auto clus : _clusters) {
        auto flipflops = clus->get_subordinate_flipflops();
    }
}