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

#include <queue>
#include <stack>
#include <string>
#include <unordered_map>
#include <vector>

#include "../evaluate.h"
#include "../include/logger.h"
#include "../include/utility.h"
#include "sequentialElement.h"
#include "sequentialGraph.h"

int NOTIMING_END_CELL = 0;
int NOOUTPUT_FLIPFLIP = 0;

class clusterFF {
public:
    clusterFF();
    clusterFF(circuit* circuit, Logger* log);
    ~clusterFF();

    uint stringToId(map<string, unsigned> port_map, string port_name);

    vector<pin*> pinToSinkPins(unsigned pin_idx);
    unsigned cellToPin(cell* cell);

    void ergodicGenerateGraph(std::stack<sequentialElement*>& stack);

private:
    sequentialGraph* _graph;
    circuit* _circuit;
    vector<cell*> _cell_vec;
    vector<macro*> _macro_vec;
    vector<pin*> _pin_vec;
    vector<net*> _net_vec;
    Logger* _log;
    vector<sequentialElement*> _flipflop_pvec;

    std::unordered_map<cell*, unsigned> _cell2Visited;

    void init();
};
