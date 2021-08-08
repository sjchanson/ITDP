
/**
 * @file sequentialGraph.h
 * @author SJchan (13560469332@163.com)
 * @brief
 * @version 0.1
 * @date 2021-04-14
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <algorithm>
#include <iterator>
#include <list>
#include <memory>
#include <set>
#include <stack>
#include <vector>

// #include "../evaluate.h"
#include "common/logger.h"
#include "common/utility.h"
#include "sequentialElement.h"

class sequentialElement;
class sequentialVertex;
class sequentialArc;

struct vertexPtrLess;

class sequentialArc {
public:
    sequentialArc();
    sequentialArc(sequentialVertex* src, sequentialVertex* sink);

    // copy constructor
    sequentialArc(const sequentialArc& obj);

    ~sequentialArc();

    void add_logic_cells(cell* c) { _logic_cells.push_back(c); }
    std::vector<cell*>& get_logic_cells() { return _logic_cells; }

    void updateSrc(sequentialVertex* src) { _src = src; }
    void updateSink(sequentialVertex* sink) { _sink = sink; }

    sequentialVertex* get_src() const { return _src; }
    sequentialVertex* get_sink() const { return _sink; }

private:
    sequentialVertex* _src;
    sequentialVertex* _sink;
    std::vector<cell*> _logic_cells;
};

class sequentialVertex {
public:
    sequentialVertex();
    sequentialVertex(sequentialBase* node);

    // copy constructor
    sequentialVertex(const sequentialVertex& obj);

    ~sequentialVertex();

    bool operator()(sequentialVertex* v) const { return *v == *this; }

    void add_src_edges(sequentialArc* src_edge) { _src_edges.push_back(src_edge); }
    void add_sink_edges(sequentialArc* sink_edge) { _sink_edges.push_back(sink_edge); }

    void add_batch_ancestors(std::list<sequentialVertex*> batch);
    void add_batch_descendants(std::list<sequentialVertex*> batch);

    void updateAncestors(std::list<sequentialVertex*> extra_ancestors);
    void updateDescendants(std::list<sequentialVertex*> extra_descendants);

    void add_ancestor(sequentialVertex* v) { _ancestors.push_back(v); }
    void add_descendant(sequentialVertex* v) { _descendants.push_back(v); }

    void removeAncestor(sequentialVertex* v);
    void removeDescendant(sequentialVertex* v);

    void duplicateRemoveAncestors();
    void duplicateRemoveDescendants();

    void set_start() { _is_start = 1; }
    void set_end() { _is_end = 1; }
    void set_const() { _is_const = 1; }

    bool operator<(const sequentialVertex& v) { return _name < v.get_name(); }
    bool operator==(const sequentialVertex& v) { return _name == v.get_name(); }

    std::string get_name() const { return _name; }
    sequentialBase* get_base() const { return _node; }
    std::vector<sequentialArc*> get_src_edges() { return _src_edges; }
    std::vector<sequentialArc*> get_sink_edges() { return _sink_edges; }

    std::list<sequentialVertex*> get_ancestors() { return _ancestors; }
    std::list<sequentialVertex*> get_descendants() { return _descendants; }

    std::vector<sequentialVertex*> get_direct_vertexes();

    std::set<sequentialVertex*, vertexPtrLess> get_src_vertexes();
    std::set<sequentialVertex*, vertexPtrLess> get_sink_vertexes();

    unsigned isStart() const { return _is_start; }
    unsigned isEnd() const { return _is_end; }

private:
    std::string _name;
    sequentialBase* _node;
    unsigned _is_start : 1;                   // The vertex is start sequentialBase.
    unsigned _is_end : 1;                     // The vertex is end sequentialBase.
    unsigned _is_const : 1;                   // The vertex is const.
    std::vector<sequentialArc*> _src_edges;   // The sequentialArc sourced from the vertex.
    std::vector<sequentialArc*> _sink_edges;  // The sequentialArc sinked to the vertex.

    std::list<sequentialVertex*> _ancestors;    // Record all ancestors.
    std::list<sequentialVertex*> _descendants;  // Record all descendants.
};

class vertexPair {
public:
    vertexPair();
    vertexPair(sequentialVertex* v1, sequentialVertex* v2);
    ~vertexPair() = default;

    sequentialVertex* get_vertex1() const { return _vertex_1; }
    sequentialVertex* get_vertex2() const { return _vertex_2; }

private:
    sequentialVertex* _vertex_1;
    sequentialVertex* _vertex_2;
};

class sequentialPair {
public:
    sequentialPair();
    sequentialPair(sequentialVertex* v1, sequentialVertex* v2);
    sequentialPair(vertexPair* pair);
    ~sequentialPair() = default;

    bool operator<(const sequentialPair& right) const { return _name < right._name; }

    bool operator==(const sequentialPair& right) const {
        if (*_vertex_1 == *(right.get_vertex1()) && *_vertex_2 == *(right.get_vertex2())) {
            return true;
        }
        if (*_vertex_1 == *(right.get_vertex2()) && *_vertex_2 == *(right.get_vertex1())) {
            return true;
        }
        return false;
    }

    void set_distance(double value) { _distance = value; }

    double get_distance() const { return _distance; }
    sequentialVertex* get_vertex1() const { return _vertex_1; }
    sequentialVertex* get_vertex2() const { return _vertex_2; }

    bool findAnotherVetex(sequentialVertex* vetex_1, sequentialVertex*& another_vertex);

private:
    string _name;
    sequentialVertex* _vertex_1;
    sequentialVertex* _vertex_2;
    double _distance;
};

struct sequentialPairCmp {
    bool operator()(const sequentialPair* left, const sequentialPair* right) const {
        if (*left < *right) {
            return true;
        } else {
            return false;
        }
    }
};

struct sequentialpairCmp2 {
    bool operator()(sequentialPair* left, const sequentialPair* right) const { return *left < *right; }
};
static bool preClusterPairCMP(sequentialPair* pair_1, sequentialPair* pair_2) {
    return pair_1->get_distance() < pair_2->get_distance();
};

struct sequentialPairHash {
    size_t operator()(const sequentialPair* pair) const {
        size_t hash_1 = std::hash<string>()(pair->get_vertex1()->get_name());
        size_t hash_2 = std::hash<string>()(pair->get_vertex2()->get_name());
        return hash_1 ^ hash_2;
    }
};

struct sequentialPairEqual {
    bool operator()(sequentialPair* pair_1, sequentialPair* pair_2) const { return *pair_1 == *pair_2; }
};

struct vertexPtrHash {
    size_t operator()(const sequentialVertex* v_ptr) const { return std::hash<string>()((*v_ptr).get_name()); }
};

struct vertexPtrEqual {
    bool operator()(sequentialVertex* v_ptr1, sequentialVertex* v_ptr2) const { return (*v_ptr1) == (*v_ptr2); }
};
static bool compareVertexPtrEqual(sequentialVertex* v_ptr1, sequentialVertex* v_ptr2) { return (*v_ptr1) == (*v_ptr2); }

struct vertexPtrLess {
    bool operator()(sequentialVertex* v_ptr1, sequentialVertex* v_ptr2) const { return (*v_ptr1) < (*v_ptr2); }
};
static bool compareVertexPtrLess(sequentialVertex* v_ptr1, sequentialVertex* v_ptr2) { return (*v_ptr1) < (*v_ptr2); }

class sequentialGraph {
public:
    sequentialGraph();
    sequentialGraph(string name, Logger* log, parameter* para);
    sequentialGraph(const sequentialGraph& obj);
    ~sequentialGraph();

    // pre cluster
    bool isExtraDistanceConnection(std::string cluster_name, sequentialVertex* class_vertex,
                                   sequentialVertex* join_vertex, double distance);
    void initPreCluster();
    void calPreClusterPair(std::stack<sequentialVertex*>& stack);
    void preClusterSolve();
    void completePreCluster(std::string cluster_name);
    void constructSubGraphs(sequentialGraph* sub_graph, string cluster_name);
    void DFSSubGraphs(sequentialGraph* graph, std::stack<sequentialVertex*>& stack,
                      std::set<sequentialVertex*, vertexPtrLess> POs,
                      std::set<sequentialVertex*, vertexPtrLess> cluster_vertexes,
                      std::unordered_set<sequentialVertex*, vertexPtrHash, vertexPtrEqual>& visited_vertexes);

    void removeVertex(sequentialVertex* vertex);

    void add_vertex(std::string name, sequentialVertex* v);
    void add_edge(sequentialArc* e);
    void add_start_vertex(sequentialVertex* start_vertex);
    void add_end_vertex(sequentialVertex* end_vertex);
    void add_const_vertex(sequentialVertex* const_vertex);

    void initHop();
    void hopForwardDFS(std::stack<sequentialVertex*>& stack);
    void hopBackwardDFS(std::stack<sequentialVertex*>& stack);
    void updateHop(sequentialVertex* vertex_1, sequentialVertex* vertex_2, sequentialVertex* fusion_vertex);

    void getDirectHopSkew(sequentialVertex* v1, sequentialVertex* v2, double& skew);
    bool isDirectHop(sequentialVertex* v1, sequentialVertex* v2);

    void initDegree();
    bool reduceDegree(sequentialVertex* v, std::unordered_map<std::string, int>& vertex_to_degree);

    void initSeqentialPair(int side_length, int core_x, int core_y, double max_skew);
    double calDistance(vertexPair* pair, double skew, int core_x, int core_y, double max_skew);

    std::set<sequentialVertex*> getRegionVertexes(int lx, int ly, int ux, int uy);
    std::set<sequentialVertex*> getXCoordVertexes(int x);

    double getDirectConnectingSkew(sequentialVertex* v1, sequentialVertex* v2);

    void updatePairSkew(sequentialVertex* root);

    sequentialVertex* findAncestor(sequentialVertex* vertex, double& skew);

    bool findRing(sequentialVertex* vertex_1, sequentialVertex* vertex_2);
    // bool findRing(sequentialVertex* vertex_1, sequentialVertex* vertex_2,
    //               std::unordered_map<std::string, int> vertex_to_degree);

    void updateCoordMapping();

    void addVirtualRoot();

    void updateArrival(sequentialVertex* v1, sequentialVertex* v2, double distance);
    bool deleteArrival(sequentialVertex* src, sequentialVertex* arrival);

    void leastCommonAncestorDFS(sequentialVertex* root, int core_x, int core_y, double max_skew);

    double findAncestorByStack(std::stack<sequentialVertex*> stack, sequentialVertex* ancestor);

    bool makeVertexFusion(sequentialVertex* vertex_1, sequentialVertex* vertex_2, sequentialBase* new_base,
                          double extra_dist);

    void makeBatchVertexFusion(std::vector<sequentialVertex*> clus_vertexes, sequentialBase* lcb);

    std::unordered_map<std::string, sequentialVertex*> get_vertexes() const { return _vertexes; }
    std::set<sequentialArc*> get_edges() const { return _arcs; }

    std::set<sequentialVertex*> get_start_vertexes() const { return _start_vertexes; }
    std::set<sequentialVertex*> get_end_vertexes() const { return _end_vertexes; }
    std::set<sequentialVertex*> get_const_vertexes() const { return _const_vertexes; }

    std::set<sequentialPair*, sequentialPairCmp> get_sequential_pairs() const { return _sequential_pairs; }

    bool isVertexExist(std::string vertex_name);
    sequentialVertex* get_vertex(std::string vertex_name);

    void modifyVertexPairAncestors(sequentialVertex* vertex_1, sequentialVertex* vertex_2,
                                   sequentialVertex* fusion_vertex);
    void modifyVertexPairDescendants(sequentialVertex* vertex_1, sequentialVertex* vertex_2,
                                     sequentialVertex* fusion_vertex);

    void deleteSequentialPair(sequentialVertex* vertex_1, sequentialVertex* vertex_2);

    std::map<sequentialVertex*, std::map<sequentialVertex*, double, vertexPtrLess>, vertexPtrLess> get_arrival() {
        return _arrivals;
    }

    bool isSrcInsidePreCluster(sequentialVertex* vertex,
                               std::unordered_set<sequentialVertex*, vertexPtrHash, vertexPtrEqual> cluster);
    bool isSinkInsidePreCluster(sequentialVertex* vertex,
                                std::unordered_set<sequentialVertex*, vertexPtrHash, vertexPtrEqual> cluster);

    // debug
    void printArrival();
    void printSequentialPair();
    void printAncestors();
    void printDescendants();

    void printArrival(sequentialVertex* vertex);
    void printSequentialPair(sequentialVertex* vertex);

    // get timing cost
    double get_modify_arrival_cost() const { return _modify_arrival_cost; }
    double get_modify_relative_cost() const { return _modify_relative_cost; }
    double get_modify_topo_cost() const { return _modify_topo_cost; }

    string get_name() const { return _name; }

    // get subGraphes.
    std::vector<sequentialGraph*> get_sub_graphs() { return _sub_graphs; }

private:
    string _name;
    Logger* _log;
    parameter* _para;
    // using unordered_map to store vertexes.
    std::unordered_map<std::string, sequentialVertex*> _vertexes;
    std::set<sequentialArc*> _arcs;

    std::set<sequentialVertex*> _start_vertexes;
    std::set<sequentialVertex*> _end_vertexes;
    std::set<sequentialVertex*> _const_vertexes;

    // for coordinate mapping.
    std::vector<int> _x_coords;
    std::vector<int> _y_coords;
    std::unordered_multimap<int, sequentialVertex*> _x_to_vertexs;
    std::unordered_multimap<int, sequentialVertex*> _y_to_vertexs;

    // for sequential pair
    std::set<sequentialPair*, sequentialPairCmp> _sequential_pairs;
    std::map<sequentialVertex*, std::map<sequentialVertex*, double, vertexPtrLess>, vertexPtrLess> _arrivals;

    // for topological sorting.
    std::unordered_map<std::string, int> _vertex_to_degree;

    // for finding lowest common ancestor.
    sequentialVertex* _virtual_root;
    std::unordered_map<sequentialVertex*, sequentialVertex*, vertexPtrHash, vertexPtrEqual> _vertex_to_ancestor;
    std::unordered_map<sequentialVertex*, bool, vertexPtrHash, vertexPtrEqual> _visited;

    // for record the path.
    std::stack<sequentialVertex*> _stack;

    // for record the timing
    double _modify_arrival_cost;
    double _modify_relative_cost;
    double _modify_topo_cost;

    // for pre cluster.
    std::vector<sequentialPair*> _precluster_pair_vec;
    std::unordered_set<sequentialPair*, sequentialPairHash, sequentialPairEqual> _precluster_pair_set;
    // std::set<sequentialPair*, sequentialPairCmp> _precluster_pair_set;
    std::unordered_map<sequentialVertex*, bool, vertexPtrHash, vertexPtrEqual> _pre_clusters_visited;
    std::unordered_multimap<string, sequentialVertex*> _pre_clusters_multimap;
    std::unordered_map<sequentialVertex*, string, vertexPtrHash, vertexPtrEqual> _vertex_to_cluster_name;
    std::vector<sequentialGraph*> _sub_graphs;
};
