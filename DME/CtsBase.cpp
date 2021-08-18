/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-16 15:42:38
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-18 11:42:05
 * @Description:
 */

#include "CtsBase.h"

struct CtsBase {
    /* data */
};

SingleCluster::SingleCluster(std::vector<const itdp::SequentialElement*> elements) { _origin_elements = elements; }

void SingleCluster::makeBinaryPair() {
    // pick up the connection of elements.
    std::vector<ClusterVertexPair> connection_pair;
    std::map<std::pair<itdp::SequentialElement*, itdp::SequentialElement*>, double> pair_to_skew;

    // identify the skew case.
    for (auto current_e : _origin_elements) {
        for (auto compare_e : _origin_elements) {
            if (current_e->get_name() == compare_e->get_name()) {
                continue;
            } else {
                double skew = current_e->get_skew(compare_e->get_name());
                // if the skew exist.
                if (skew != DBL_MAX) {
                    auto pair = std::make_pair(current_e, compare_e);
                    pair_to_skew _binary_pairs.push_back(pair);
                }
            }
        }
    }
    std::sort(
        _binary_pairs.begin(), _binary_pairs.end(),
        [](const ClusterVertexPair& pair1, const ClusterVertexPair& pair2) { return pair1.distance > pair2.distance; });

    // pick up the pair
    for (auto pair : _binary_pairs) {
        updateVertexFusion(pair.vertex_1, pair.vertex_2, pair.distance, 0);
    }
}

/**
 * @description: Make the cluster vertex.
 * @param {string} name
 * @param {Point<DBU>} point
 * @return {*}
 * @author: ShiJian Chen
 */
ClusterVertex* SingleCluster::makeClusterVertex(std::string name, Point<DBU> point) {
    auto it = _vertexes.find(name);
    if (it != _vertexes.end()) {
        return (*it).second;
    } else {
        ClusterVertex* vertex = new ClusterVertex(point, name);
        _vertexes.emplace(name, vertex);
        return vertex;
    }
}

/**
 * @description:
 * @param {ClusterVertex*} v1
 * @param {ClusterVertex*} v2
 * @param {double} skew
 * @return {*}
 * @author: ShiJian Chen
 */
void SingleCluster::updateVertexFusion(ClusterVertex* v1, ClusterVertex* v2, double skew, int level) {
    v2->set_skew(skew);

    ClusterNode* node_1 = makeClusterNode(v1);
    ClusterNode* node_2 = makeClusterNode(v2);

    DBU x_coord = (v1->get_point().get_x() + v2->get_point().get_x()) / 2;
    DBU y_coord = (v1->get_point().get_y() + v2->get_point().get_y()) / 2;

    ClusterVertex* fusion_vertex =
        makeClusterVertex("transition_" + std::to_string(_vertexes.size()), Point<DBU>(x_coord, y_coord));
    ClusterNode* new_node = makeClusterNode(fusion_vertex);

    node_1->parent = new_node;
    node_1->level = level;
    node_2->parent = new_node;
    node_2->level = level;
    new_node->left_child = node_1;
    new_node->right_child = node_2;
}

/**
 * @description: Make the Cluster node.
 * @param {ClusterVertex*} vertex
 * @return {*}
 * @author: ShiJian Chen
 */
ClusterNode* SingleCluster::makeClusterNode(ClusterVertex* vertex) {
    auto it = _nodes.find(vertex->get_name());
    if (it != _nodes.end()) {
        return (*it).second;
    } else {
        ClusterNode* node = new ClusterNode(vertex);
        _nodes.emplace(vertex->get_name(), node);
        return node;
    }
}