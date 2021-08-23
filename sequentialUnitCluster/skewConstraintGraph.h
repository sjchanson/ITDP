/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-04 13:57:50
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-22 09:58:27
 * @Description:
 */

#ifndef ITDP_SKEW_CONSTRAINT_GRAPH
#define ITDP_SKEW_CONSTRAINT_GRAPH

#include <math.h>

#include <algorithm>
#include <list>
#include <memory>
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../include/common/logger.h"
#include "../include/common/parameter.h"
#include "sequentialElement.h"

struct SubCluster;  // Transition variable

namespace itdp {
class SequentialEdge;

class SequentialVertex {
public:
    SequentialVertex() = delete;
    SequentialVertex(SequentialElement* node);
    ~SequentialVertex() = default;

    // getter.
    std::string get_name() const { return _name; }
    SequentialElement* get_node() const { return _node; }
    std::vector<SequentialEdge*> get_src_edges() const { return _src_edges; }
    SequentialEdge* get_src_edge(SequentialVertex* source) const;
    std::vector<SequentialEdge*> get_sink_edges() const { return _sink_edges; }
    SequentialEdge* get_sink_edge(SequentialVertex* destination) const;
    std::list<SequentialVertex*> get_ancestors() const { return _ancestors; }
    std::list<SequentialVertex*> get_descendants() const { return _descendants; }

    bool isStart() const { return _is_start == 1; }
    bool isEnd() const { return _is_end == 1; }

    // setter.
    // special case need to set the set start/end.
    void set_start() { _is_start = 1; }
    void set_end() { _is_end = 1; }
    void add_src_edge(SequentialEdge* src_edge) { _src_edges.push_back(src_edge); }
    void add_sink_edge(SequentialEdge* sink_edge) { _sink_edges.push_back(sink_edge); }

    void add_descendant(SequentialVertex* v);
    void add_batch_descendants(std::list<SequentialVertex*> batch);
    void add_ancestor(SequentialVertex* v);
    void add_batch_ancestors(std::list<SequentialVertex*> batch);

    void sortDuplicateAncestors();
    void sortDuplicateDescendants();

    void removeAncestor(SequentialVertex* ancestor);
    void removeDescendant(SequentialVertex* descendant);
    void updateAncestors(std::list<SequentialVertex*> extra_ancestors);
    void updateDescendants(std::list<SequentialVertex*> extra_descendants);

    // void mergeOppositeEdges();

    void removeSrcEdge(SequentialEdge* e);
    void removeSinkEdge(SequentialEdge* e);

    bool operator==(const SequentialVertex& v) { return _name == v.get_name(); }

private:
    std::string _name;
    SequentialElement* _node;
    unsigned _is_start : 1;
    unsigned _is_end : 1;
    std::vector<SequentialEdge*> _src_edges;
    std::vector<SequentialEdge*> _sink_edges;
    std::list<SequentialVertex*> _ancestors;    // Record all Vertexs that can reach this.
    std::list<SequentialVertex*> _descendants;  // Record all Vertexs that can be reached at this.
};

class SequentialEdge {
public:
    SequentialEdge() = delete;
    SequentialEdge(SequentialVertex* src, SequentialVertex* sink);
    ~SequentialEdge() = default;

    // getter.
    std::string get_name() const { return _name; }
    SequentialVertex* get_src_vertex() const { return _src; }  // the vertex need to be modified.
    SequentialVertex* get_sink_vertex() const { return _sink; }
    double get_setup_skew() const { return _setup_skew; }

    // setter.
    void set_skew(double skew) { _setup_skew = skew; }
    void updateSink(SequentialVertex* sink);
    void updateSrc(SequentialVertex* src) { _src = src; }

    bool operator==(const SequentialEdge& e) { return _name == e.get_name(); }

private:
    std::string _name;
    SequentialVertex* _src;
    SequentialVertex* _sink;
    double _setup_skew;
};

class SkewConstraintGraph {
public:
    SkewConstraintGraph() = delete;
    SkewConstraintGraph(std::string name);
    ~SkewConstraintGraph();

    // getter.
    int get_sequential_vertexes_size() const { return _sequential_vertexes.size(); }
    int get_sequential_edges_size() const { return _sequential_edges.size(); }
    SequentialVertex* get_existent_vertex(std::string name);  // the return value need to modify.
    bool isExistentEdge(std::string src_name, std::string sink_name);

    std::vector<SequentialVertex*> get_start_vertexes() const;
    bool isStartVertex(std::string name) const;
    std::vector<SequentialVertex*> get_end_vertexes() const;
    bool isEndVertex(std::string name) const;

    // setter.
    void add_sequential_vertex(SequentialVertex* v);
    void add_sequential_edge(SequentialEdge* e);
    void add_x_vertex_mapping(int x, SequentialVertex* v);

    std::map<std::string, std::vector<const SequentialElement*>> subgraphPartition(int max_subgraph_size);

    void initReachableVertexes();
    void initRegion();

    void initDistanceMatrix();

    std::map<std::string, std::vector<const SequentialElement*>> makeVertexesFusion();
    std::map<std::string, std::vector<const SequentialElement*>> changeClusterName(
        std::map<std::string, std::vector<const SequentialElement*>> clusters);

private:
    std::string _name;
    Logger* _log;
    std::shared_ptr<Parameter> _parameter;

    // SeqeuntialVertex find and delete operations are required.
    std::unordered_map<std::string, SequentialVertex*> _sequential_vertexes;
    std::unordered_map<std::string, SequentialEdge*> _sequential_edges;
    std::map<std::string, SequentialVertex*> _start_vertexes;
    std::map<std::string, SequentialVertex*> _end_vertexes;

    // x_coordinate -> vertexes mapping.
    std::vector<int> _x_coordinates;
    std::map<int, std::vector<SequentialVertex*>> _x_to_vertexes;

    // distance matrix
    std::map<std::string, std::map<std::string, double>> _distance_matrix;

    // delete vertex edge.
    void deleteSequentialEdge(std::string name);
    void deleteSequentialVertex(std::string name);

    // subgraph partition function
    std::vector<SequentialEdge*> sortEdgeBySkew();
    void subClusterClassification(std::vector<SequentialEdge*> edges,
                                  std::unordered_map<int, SubCluster*>& sub_clusters, int max_subgraph_size);
    bool isExtraGreaterSkew(SequentialVertex* vertex, SequentialVertex* inner_vertex, SubCluster* sub_cluster,
                            double current_skew);
    void constructSubGraph(SkewConstraintGraph* sub_graph, SubCluster* sub_cluster);
    void addSubGraphInputOutput(SkewConstraintGraph* sub_graph, SequentialVertex* vertex,
                                std::set<std::string> cluster_vertexes);
    void DFSGenerateSubGraph(SkewConstraintGraph* sub_graph, std::set<std::string> cluster_vertexes,
                             std::stack<SequentialVertex*>& stack, std::set<std::string>& visited_vertexes);

    // initialize the reachable vertexes.
    void hopForwardDFS(std::stack<SequentialVertex*>& stack);
    void hopBackwardDFS(std::stack<SequentialVertex*>& stack);

    // region search.
    std::pair<Point<DBU>, Point<DBU>> obtainSearchRegion(SequentialVertex* vertex);
    std::vector<SequentialVertex*> obtainRegionVertexes(std::pair<Point<DBU>, Point<DBU>> region);

    // ring judge.
    bool findRing(SequentialVertex* vertex_1, SequentialVertex* vertex_2);
    bool isDirectHop(SequentialVertex* vertex_1, SequentialVertex* vertex_2);

    // distance manager.
    double get_direct_hop_skew(SequentialVertex* vertex_1, SequentialVertex* vertex_2);
    double calculateDistance(SequentialVertex* vertex_1, SequentialVertex* vertex_2, double skew);

    // arrival management.
    void updateDistanceMatrix(std::string v1_name, std::string v2_name, double distance);
    void modifyDidstanceMatrix(SequentialVertex* vertex_1, SequentialVertex* vertex_2, SequentialVertex* new_vertex);
    std::map<std::string, double> obtainDistanceMap(SequentialVertex* vertex_1, SequentialVertex* vertex_2);
    void deleteVertexDistanceMatrix(SequentialVertex* vertex);
    void deleteDistanceMatrix(std::string modify_name, std::string delete_name);
    void addVertexDistanceMatrix(std::string vertex_name, std::map<std::string, double> vertex_map);

    // least common ancestor.
    void obtainIndirectHopDistance();
    void leastCommonAncestorDFS(SequentialVertex* root, std::stack<SequentialVertex*>& stack,
                                std::unordered_map<std::string, std::string>& vertex_to_ancestor,
                                std::unordered_map<std::string, bool>& vertex_visited);
    std::string findAncestor(std::string vertex_name, std::unordered_map<std::string, std::string> vertex_to_ancestor,
                             double& skew);
    double skewAccumulation(std::stack<SequentialVertex*> stack, std::string ancestor_name);

    // vertexes fusion.
    std::pair<std::string, std::string> obtainMinDistancePair();
    void updateTwoVertexesFusion(SequentialVertex* vertex_1, SequentialVertex* vertex_2, SequentialVertex* new_vertex);
    void updateGraphConnection(SequentialVertex* vertex_1, SequentialVertex* vertex_2, SequentialVertex* new_vertex);
    void modifyVertexAncestors(SequentialVertex* vertex_1, SequentialVertex* vertex_2, SequentialVertex* new_vertex);
    void modifyVertexDescendants(SequentialVertex* vertex_1, SequentialVertex* vertex_2, SequentialVertex* new_vertex);
};

}  // namespace itdp

#endif
