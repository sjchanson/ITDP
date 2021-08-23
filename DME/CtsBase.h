/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-16 15:42:30
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-19 13:18:55
 * @Description:
 */

#include <math.h>

#include <algorithm>
#include <map>
#include <queue>
#include <string>
#include <vector>

#include "../sequentialUnitCluster/sequentialElement.h"
#include "DME.h"

struct ClusterVertexPair {
    ClusterVertexPair(ClusterVertex* v1, ClusterVertex* v2);

    ClusterVertex* vertex_1;
    ClusterVertex* vertex_2;
    DBU distance;
};
inline ClusterVertexPair::ClusterVertexPair(ClusterVertex* v1, ClusterVertex* v2) {
    vertex_1 = v1;
    vertex_2 = v2;
    distance = std::fabs(v1->get_point().get_x() - v2->get_point().get_x()) +
               std::fabs(v1->get_point().get_y() - v2->get_point().get_y());
}

struct ClusterNode {
    ClusterNode(ClusterVertex* v);
    int level;
    ClusterVertex* vertex;
    ClusterNode* parent;
    ClusterNode* left_child;
    ClusterNode* right_child;
};
inline ClusterNode::ClusterNode(ClusterVertex* v)
    : level(__INT_MAX__), parent(nullptr), left_child(nullptr), right_child(nullptr) {
    vertex = v;
}

class SingleCluster {
public:
    SingleCluster() = delete;
    SingleCluster(std::string name, std::vector<const itdp::SequentialElement*> elements);
    ~SingleCluster();

    // getter.
    std::string get_name() const { return _name; }
    std::vector<ClusterVertex*> get_perfect_binary_tree() const { return _perfect_binary_trees; }

    void buildBinaryTree();

private:
    std::string _name;
    std::vector<const itdp::SequentialElement*> _origin_elements;
    std::map<std::string, ClusterVertex*> _vertexes;
    std::map<std::string, ClusterNode*> _nodes;
    std::vector<ClusterVertex*> _perfect_binary_trees;

    void buildPerfectBinaryTree(ClusterNode* root);
    void analyzeBFS(std::queue<ClusterNode*> queue);

    void makeInitBinaryVertexes(std::map<std::string, ClusterVertex*>& remain_vertexes);
    void initRemainElements(std::map<std::string, const itdp::SequentialElement*>& remain_elements);
    bool isElementRemain(std::map<std::string, const itdp::SequentialElement*> remain_elements,
                         const itdp::SequentialElement* element);
    void eraseElement(std::map<std::string, const itdp::SequentialElement*>& remain_elements,
                      const itdp::SequentialElement* element);
    void obtainElementPair(std::vector<itdp::SequentialElementPair>& element_pair);

    std::vector<ClusterVertexPair> makeNextBinaryPair(std::map<std::string, ClusterVertex*>& remain_vertexes);
    bool isClusterVertexRemain(std::map<std::string, ClusterVertex*> remain_vertexes, ClusterVertex* vertex);
    void eraseClusterVertex(std::map<std::string, ClusterVertex*>& remain_vertexes, ClusterVertex* vertex);

    std::map<std::string, ClusterVertex*> updateNextLevelTree(std::vector<ClusterVertexPair> vertex_pair, int level);

    ClusterVertex* makeClusterVertex(std::string name, Point<DBU> point);
    ClusterNode* makeClusterNode(ClusterVertex* vertex);
    ClusterVertex* updateVertexFusion(ClusterVertex* v1, ClusterVertex* v2, double skew, int level);
};

class CtsBase {
public:
    CtsBase() = delete;
    CtsBase(std::map<std::string, std::vector<const itdp::SequentialElement*>> clusters_map);
    ~CtsBase();

    std::vector<SingleCluster*> get_binary_clusters() const { return _binary_clusters; }

private:
    std::vector<SingleCluster*> _binary_clusters;
};