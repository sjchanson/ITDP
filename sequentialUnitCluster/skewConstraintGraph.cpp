/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-04 15:29:55
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-07 11:11:50
 * @Description:
 */

#include "skewConstraintGraph.h"

namespace itdp {

SequentialVertex::SequentialVertex(SequentialElement* node) {
    _name = node->get_name();
    _node = node;
    if (node->isPi()) {
        set_start();
    }
    if (node->isPo()) {
        set_end();
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



}  // namespace itdp