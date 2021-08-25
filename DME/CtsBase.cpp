/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-16 15:42:38
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-25 11:09:45
 * @Description:
 */

#include "CtsBase.h"

#include "omp.h"

SingleCluster::SingleCluster(std::string name, std::vector<const itdp::SequentialElement*> elements) {
    _name = name;
    _origin_elements = elements;
}

SingleCluster::~SingleCluster() {
    _origin_elements.clear();
    for (auto vertex : _vertexes) {
        delete vertex.second;
    }
    _vertexes.clear();
    for (auto node : _nodes) {
        delete node.second;
    }
    _nodes.clear();
    _perfect_binary_trees.clear();
}

/**
 * @description: Main function of building the perfect binary tree.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SingleCluster::buildBinaryTree() {
    std::map<std::string, ClusterVertex*> remain_vertexes;
    std::vector<ClusterVertexPair> pairs;

    makeInitBinaryVertexes(remain_vertexes);  // level = 0

    // if only a vertex.
    if (remain_vertexes.size() == 1) {
        _perfect_binary_trees.clear();
        std::string root_name = (*remain_vertexes.begin()).first;
        ClusterNode* root = _nodes[root_name];
        buildPerfectBinaryTree(root);
        return;
    }

    int level = 1;
    while (remain_vertexes.size() > 1) {
        pairs.clear();
        pairs = makeNextBinaryPair(remain_vertexes);
        auto transition_vertexes = updateNextLevelTree(pairs, level++);
        remain_vertexes.insert(transition_vertexes.begin(), transition_vertexes.end());
    }

    // tree root.
    std::string root_name = (*remain_vertexes.begin()).first;
    ClusterNode* root = _nodes[root_name];

    buildPerfectBinaryTree(root);
}

/**
 * @description: Build the perfect binary tree.
 * @param {ClusterNode*} root
 * @return {*}
 * @author: ShiJian Chen
 */
void SingleCluster::buildPerfectBinaryTree(ClusterNode* root) {
    _perfect_binary_trees.clear();
    std::queue<ClusterNode*> node_queue;
    node_queue.push(root);
    analyzeBFS(node_queue);
}

/**
 * @description: Level traversal.
 * @param {queue<ClusterNode*>} queue
 * @return {*}
 * @author: ShiJian Chen
 */
void SingleCluster::analyzeBFS(std::queue<ClusterNode*> queue) {
    while (!queue.empty()) {
        auto current_node = queue.front();
        _perfect_binary_trees.push_back(current_node->vertex);

        queue.pop();

        if (current_node->level == 0) {
            continue;
        }

        if (current_node->left_child == nullptr && current_node->level != 0) {
            auto left_v = makeClusterVertex("transition_" + std::to_string(_vertexes.size()), Point<DBU>(-1, -1));
            auto left_child = makeClusterNode(left_v);
            left_child->level = current_node->level - 1;
            auto right_v = makeClusterVertex("transition_" + std::to_string(_vertexes.size()), Point<DBU>(-1, -1));
            auto right_child = makeClusterNode(right_v);
            right_child->level = current_node->level - 1;
            current_node->left_child = left_child;
            current_node->right_child = right_child;
        }
        queue.push(current_node->left_child);
        queue.push(current_node->right_child);
    }
}

/**
 * @description: Init the binary pair, return the next level vertexes.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SingleCluster::makeInitBinaryVertexes(std::map<std::string, ClusterVertex*>& remain_vertexes) {
    // pick up the connection of elements.
    std::vector<itdp::SequentialElementPair> element_pair;
    std::map<std::string, ClusterVertex*> transition_vertexes;
    std::map<std::string, const itdp::SequentialElement*> remain_elements;

    // init the remain_elements.
    for (auto element : _origin_elements) {
        remain_elements.emplace(element->get_name(), element);
    }

    obtainElementPair(element_pair);

    // pick up the pair
    for (auto pair : element_pair) {
        ClusterVertex* vertex_1 = nullptr;
        ClusterVertex* vertex_2 = nullptr;
        auto element_1 = pair.element_1;
        auto element_2 = pair.element_2;
        if (isElementRemain(remain_elements, element_1)) {
            vertex_1 = makeClusterVertex(element_1->get_name(), element_1->get_coord());
        }
        if (isElementRemain(remain_elements, element_2)) {
            vertex_2 = makeClusterVertex(element_2->get_name(), element_2->get_coord());
        }
        if (vertex_1 && vertex_2) {
            auto transition_v = updateVertexFusion(vertex_1, vertex_2, pair.skew, 0);
            transition_vertexes.emplace(transition_v->get_name(), transition_v);

            // delete the remain element.
            eraseElement(remain_elements, element_1);
            eraseElement(remain_elements, element_2);
        }
    }

    // deal with the remain element.
    for (auto pair : remain_elements) {
        auto element = pair.second;
        ClusterVertex* vertex = makeClusterVertex(element->get_name(), element->get_coord());
        remain_vertexes.emplace(vertex->get_name(), vertex);
    }

    auto remain_pair = makeNextBinaryPair(remain_vertexes);
    for (auto pair : remain_pair) {
        auto transition_v = updateVertexFusion(pair.vertex_1, pair.vertex_2, 0, 0);
        transition_vertexes.emplace(transition_v->get_name(), transition_v);
    }

    // add the transition vertexes.
    remain_vertexes.insert(transition_vertexes.begin(), transition_vertexes.end());
}

/**
 * @description: Init the remain elements.
 * @param {const} itdp
 * @return {*}
 * @author: ShiJian Chen
 */
void SingleCluster::initRemainElements(std::map<std::string, const itdp::SequentialElement*>& remain_elements) {
    for (auto element : _origin_elements) {
        remain_elements.emplace(element->get_name(), element);
    }
}

/**
 * @description: Obtain the skew connection of the cluster.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SingleCluster::obtainElementPair(std::vector<itdp::SequentialElementPair>& element_pair) {
    // identify the skew case.
    for (auto current_e : _origin_elements) {
        for (auto compare_e : _origin_elements) {
            if (current_e->get_name() == compare_e->get_name()) {
                continue;
            } else {
                double skew = current_e->get_skew(compare_e->get_name());
                // if the skew exist.
                if (skew != DBL_MAX) {
                    auto pair = itdp::SequentialElementPair(compare_e, current_e);
                    pair.skew = skew;
                    element_pair.push_back(pair);
                }
            }
        }
    }
    std::sort(element_pair.begin(), element_pair.end(),
              [](const itdp::SequentialElementPair& pair1, const itdp::SequentialElementPair& pair2) {
                  return pair1.skew > pair2.skew;
              });
}

/**
 * @description: Make the next binary pair.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
std::vector<ClusterVertexPair> SingleCluster::makeNextBinaryPair(
    std::map<std::string, ClusterVertex*>& remain_vertexes) {
    std::vector<ClusterVertexPair> cluster_vertex_pair;
    std::vector<ClusterVertexPair> tmp_pair;

    for (auto it_1 = remain_vertexes.begin(); it_1 != remain_vertexes.end(); it_1++) {
        auto vertex_1 = (*it_1).second;
        auto tmp_it = it_1;
        for (auto it_2 = ++tmp_it; it_2 != remain_vertexes.end(); it_2++) {
            auto vertex_2 = (*it_2).second;
            ClusterVertexPair pair(vertex_1, vertex_2);
            tmp_pair.push_back(pair);
        }
    }

    // pick up the vertex_pair
    for (auto pair : tmp_pair) {
        bool flag_1 = isClusterVertexRemain(remain_vertexes, pair.vertex_1);
        bool flag_2 = isClusterVertexRemain(remain_vertexes, pair.vertex_2);

        if (flag_1 && flag_2) {
            cluster_vertex_pair.push_back(pair);
            eraseClusterVertex(remain_vertexes, pair.vertex_1);
            eraseClusterVertex(remain_vertexes, pair.vertex_2);
        }
    }

    return cluster_vertex_pair;
}

/**
 * @description: Identify the cluster vertex is remain.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
bool SingleCluster::isClusterVertexRemain(std::map<std::string, ClusterVertex*> remain_vertexes,
                                          ClusterVertex* vertex) {
    auto it = remain_vertexes.find(vertex->get_name());
    if (it != remain_vertexes.end()) {
        return true;
    } else {
        return false;
    }
}

/**
 * @description: Erase the cluster vertex.
 * @param {ClusterVertex*} vertex
 * @return {*}
 * @author: ShiJian Chen
 */
void SingleCluster::eraseClusterVertex(std::map<std::string, ClusterVertex*>& remain_vertexes, ClusterVertex* vertex) {
    auto it = remain_vertexes.find(vertex->get_name());
    if (it != remain_vertexes.end()) {
        remain_vertexes.erase(it);
    }
}

/**
 * @description: Update the Next level tree.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
std::map<std::string, ClusterVertex*> SingleCluster::updateNextLevelTree(std::vector<ClusterVertexPair> vertex_pair,
                                                                         int level) {
    std::map<std::string, ClusterVertex*> transition_vertexes;

    for (auto pair : vertex_pair) {
        auto transition_v = updateVertexFusion(pair.vertex_1, pair.vertex_2, 0, level);
        transition_vertexes.emplace(transition_v->get_name(), transition_v);
    }

    return transition_vertexes;
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
ClusterVertex* SingleCluster::updateVertexFusion(ClusterVertex* v1, ClusterVertex* v2, double skew, int level) {
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

    return fusion_vertex;
}

/**
 * @description: Identify if the element exist.
 * @param {const} itdp
 * @return {*}
 * @author: ShiJian Chen
 */
bool SingleCluster::isElementRemain(std::map<std::string, const itdp::SequentialElement*> remain_elements,
                                    const itdp::SequentialElement* element) {
    auto it = remain_elements.find(element->get_name());
    if (it != remain_elements.end()) {
        return true;
    } else {
        return false;
    }
}

/**
 * @description: Erase the element.
 * @param {const} itdp
 * @return {*}
 * @author: ShiJian Chen
 */
void SingleCluster::eraseElement(std::map<std::string, const itdp::SequentialElement*>& remain_elements,
                                 const itdp::SequentialElement* element) {
    auto it = remain_elements.find(element->get_name());
    if (it != remain_elements.end()) {
        remain_elements.erase(it);
    }
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

CtsBase::CtsBase(std::map<std::string, std::vector<const itdp::SequentialElement*>> clusters_map) {
    for (auto pair : clusters_map) {
        SingleCluster* cluster = new SingleCluster(pair.first, pair.second);
        _binary_clusters.push_back(cluster);
    }

    // parallel calculate the binary tree.
    omp_set_num_threads(16);

#pragma omp parallel for
    for (size_t i = 0; i < _binary_clusters.size(); i++) {
        auto binary_cluster = _binary_clusters[i];
        binary_cluster->buildBinaryTree();
    }
}

CtsBase::~CtsBase() {
    for (auto cluster : _binary_clusters) {
        delete cluster;
    }
}