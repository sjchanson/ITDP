/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-16 15:42:38
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-16 16:27:09
 * @Description:
 */

#include "CtsBase.h"

SingleCluster::SingleCluster(std::vector<const itdp::SequentialElement*> elements) { _origin_elements = elements; }

void SingleCluster::makeBinaryPair() {
    // pick up the connection of elements.
    std::vector<ClusterVertexPair> connection_pair;

    for (auto current_e : _origin_elements) {
        for (auto compare_e : _origin_elements) {
            if (current_e->get_name() == compare_e->get_name()) {
                continue;
            }
        }
    }
}

ClusterVertex* SingleCluster::makeClusterVertex(std::string name, Point<DBU> point) {
    auto it = _vertexes.find(name);
    if (it != _vertexes.end()) {
        return (*it).second;
    } else {
        ClusterVertex* vertex = new ClusterVertex(&point, name);
    }
}