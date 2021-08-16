/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-16 15:42:30
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-16 16:25:38
 * @Description:
 */

#include <map>
#include <string>
#include <vector>

#include "../sequentialUnitCluster/sequentialElement.h"
#include "DME.h"

struct ClusterVertexPair {
    ClusterVertexPair(ClusterVertex* v1, ClusterVertex* v2, DBU dist);

    ClusterVertex* vertex_1;
    ClusterVertex* vertex_2;
    DBU distance;
};
inline ClusterVertexPair::ClusterVertexPair(ClusterVertex* v1, ClusterVertex* v2, DBU dist) {
    vertex_1 = v1;
    vertex_2 = v2;
    distance = dist;
}

class SingleCluster {
public:
    SingleCluster() = delete;
    SingleCluster(std::vector<const itdp::SequentialElement*> elements);
    ~SingleCluster();

    void makeBinaryPair();

    ClusterVertex* makeClusterVertex(std::string name, Point<DBU> point);

private:
    std::vector<const itdp::SequentialElement*> _origin_elements;
    std::vector<const itdp::SequentialElement*> _remain_elements;

    std::map<std::string, ClusterVertex*> _vertexes;
    std::vector<ClusterVertex*> _perfect_binary_trees;
    std::vector<ClusterVertexPair> _binary_pairs;
};

class CtsBase {
public:
    CtsBase() = default;
    ~CtsBase();

private:
    //
};