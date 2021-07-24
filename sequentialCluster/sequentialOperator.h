/**
 * @file sequentialOperator.h
 * @author SJchan (13560469332@163.com)
 * @brief
 * @version 0.1
 * @date 2021-04-07
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <algorithm>
#include <stack>
#include <string>
#include <unordered_map>
#include <vector>

#include "evaluate.h"
#include "common/logger.h"
#include "common/parameter.h"
#include "common/utility.h"
#include "../reviseFile/ReviseDEF.h"
#include "../reviseFile/ReviseVerilog.h"
#include "sequentialElement.h"
#include "sequentialGraph.h"

struct arrivalDistance {
    sequentialBase* arrival_base;
    double distance;
    arrivalDistance(sequentialBase* base, double dist);
};

struct baseHash {
    size_t operator()(const sequentialBase* base_ptr) const { return std::hash<string>()((*base_ptr).get_name()); }
};

struct baseEqual {
    bool operator()(sequentialBase* base_ptr1, sequentialBase* base_ptr2) const { return *base_ptr1 == *base_ptr2; }
};

class sequentialOperator {
public:
    sequentialOperator();
    sequentialOperator(parameter* para, circuit* circuit, Logger* log);
    sequentialOperator(parameter* para, circuit* circuit, Logger* log, sequentialGraph* graph);
    ~sequentialOperator();

    // test for modify def.
    void cleanLCBInDEF();
    void cleanLCBInVerilog();

    // generate the flipflops according to vertex.
    void traverseFlipflop();

    void pinIdToSinkPins(unsigned pin_idx, vector<pin*>& pins);
    vector<unsigned> cellToPinIds(cell* cell);
    uint stringToId(map<string, unsigned> port_map, string port_name);

    void initSubGraphs();
    void updatePreClusteres();

    void initSequentialPair();

    void updateVertexFusion();

    void test();
    void testDFS(std::stack<sequentialVertex*>& stack);

    void replenishCluster();

    void plotInitGraph(std::string name);
    void plotIncrementalGraph(string name, int iter, int interval);
    void plotCurGraph(std::string name);

    void printArrival();
    void printSequentialPair();
    std::unordered_set<sequentialCluster*, baseHash, baseEqual> get_clusters() const { return _clusters; }
    std::vector<sequentialFlipFlop*> get_sequential_flipflop_vec();

private:
    parameter* _para;
    Logger* _log;
    sequentialGraph* _graph;
    circuit* _circuit;
    vector<cell*> _cell_vec;
    vector<macro*> _macro_vec;
    vector<pin*> _pin_vec;
    vector<net*> _net_vec;
    vector<cell*> _lcb_vec;
    vector<cell*> _flipflop_vec;

    std::unordered_map<std::string, bool> _is_visited_ff;
    std::unordered_map<std::string, unsigned> _cell2Visited;
    std::unordered_map<std::string, bool> _is_visited_logic;
    std::unordered_map<std::string, sequentialLogicCell*> _name_to_logic;

    std::unordered_set<sequentialCluster*, baseHash, baseEqual> _clusters;
    std::unordered_set<sequentialFlipFlop*, baseHash, baseEqual> _flipflops;

    // normalization
    double _core_x;
    double _core_y;

    vector<double> _required_skews;
    double _max_required_skew;

    // pair store
    std::set<sequentialPair*, sequentialPairCmp> _pairs;

    // for record timing.
    double _sort_set_cost;
    double _find_ring_cost;
    double _fusion_total_cost;
    double _fusion_tmp_cost;
    double _modify_arrival_cost;

    // sub operators store.
    std::vector<sequentialOperator*> _sequential_opt_vec;

    void init();

    void modifyVisitedFFMap(std::string key, bool value);

    sequentialVertex* makeVertex(sequentialBase* seq, bool& flag);
    bool addSequentialGraph(sequentialBase* sink_seq, std::stack<sequentialBase*> stack);
    void ergodicGenerateGraph(std::stack<sequentialBase*>& stack);
    void printGraphInfo();

    void makeNormalization();
};
