/**
 * @file cluster.h
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

#include "../evaluate.h"
#include "../include/logger.h"
#include "../include/utility.h"
#include "sequentialElement.h"
#include "sequentialGraph.h"

class sequentialOpt {
public:
    sequentialOpt();
    sequentialOpt(circuit* circuit, Logger* log);
    ~sequentialOpt();

    void set_slack_flag(double flag) { _slack_flag = flag; }

    void pinToSinkPins(unsigned pin_idx, vector<pin*>& pins);
    vector<unsigned> cellToPins(cell* cell);
    uint stringToId(map<string, unsigned> port_map, string port_name);

    // modify.
    sequentialPrimaryIO* idToSeqIO(uint idx) { return _seq_io_vec[idx]; }
    sequentialLogicCell* idToSeqCell(uint idx) { return _seq_cell_vec[idx]; }
    sequentialFlipFlop* idToSeqFF(uint idx) { return _seq_ff_vec[idx]; }
    sequentialCluster* idToSeqClus(uint idx) { return _seq_cluster_vec[idx]; }

    void test();
    void testDFS(std::stack<sequentialVertex*>& stack);

    void plot();

private:
    double _slack_flag;
    Logger* _log;
    sequentialGraph* _graph;
    circuit* _circuit;
    vector<cell*> _cell_vec;
    vector<macro*> _macro_vec;
    vector<pin*> _pin_vec;
    vector<net*> _net_vec;
    vector<cell*> _flipflop_vec;

    // modify.
    vector<sequentialPrimaryIO*> _seq_io_vec;
    vector<sequentialLogicCell*> _seq_cell_vec;
    vector<sequentialFlipFlop*> _seq_ff_vec;
    vector<sequentialCluster*> _seq_cluster_vec;

    std::unordered_map<std::string, bool> _is_visited_ff;
    std::unordered_map<std::string, unsigned> _cell2Visited;
    std::unordered_map<std::string, uint> _vertex2Id;

    // for normalization
    double _core_x;
    double _core_y;

    vector<double> _required_skews;
    vector<double> _skews;

    double _max_required_skew;
    double _max_difference_skew;

    void init();

    void modifyVisitedFFMap(std::string key, bool value);

    sequentialVertex* makeVertex(sequentialBase* seq, bool& flag);
    bool addSequentialGraph(sequentialBase* sink_seq, std::stack<sequentialBase*> stack);
    void ergodicGenerateGraph(std::stack<sequentialBase*>& stack);
    void printGraphInfo();

    void makeNormalization();
};
