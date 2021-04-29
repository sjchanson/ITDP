#include "sequentialCluster.h"

#include <iostream>
#include <mutex>
#include <sstream>

sequentialCluster::sequentialCluster() : _log(nullptr), _graph(nullptr), _circuit(nullptr) {}

sequentialCluster::sequentialCluster(circuit* circuit, Logger* log) : sequentialCluster() {
    _circuit = circuit;
    _log = log;

    // get cells pointer.
    for (auto& cell : _circuit->getCells()) {
        _cell_vec.push_back(&cell);
    }
    // get macros pointer.
    for (auto& macro : _circuit->getMacros()) {
        _macro_vec.push_back(&macro);
    }
    // get pins pointer.
    for (auto& pin : _circuit->getPins()) {
        _pin_vec.push_back(&pin);
    }
    // get nets pointer.
    for (auto& net : _circuit->getNets()) {
        _net_vec.push_back(&net);
    }

    // get flipflops pointer.
    for (auto cur_pin : _pin_vec) {
        if (cur_pin->isFlopInput && !cur_pin->isFlopCkPort) {
            auto& cell = _cell_vec[cur_pin->owner];
            _is_visited_ff[cell->name] = false;
            _flipflop_vec.push_back(cell);
        }
    }

    init();
}

sequentialCluster::~sequentialCluster() {
    _log = nullptr;
    _graph = nullptr;
    _circuit = nullptr;

    for (auto& cell : _cell_vec) {
        cell = nullptr;
    }

    for (auto& macro : _macro_vec) {
        macro = nullptr;
    }

    for (auto& pin : _pin_vec) {
        pin = nullptr;
    }

    for (auto& net : _net_vec) {
        net = nullptr;
    }

    for (auto& flipflop : _flipflop_vec) {
        flipflop = nullptr;
    }
}

void sequentialCluster::init() {
    _graph = new sequentialGraph();

    vector<pin*> pi_vec;
    vector<pin*> po_vec;

    for (auto cur_pin : _pin_vec) {
        // skip the clock pin "iccad_clk"
        if (cur_pin->name == "iccad_clk") {
            continue;
        }
        if (cur_pin->type == 1) {  // PI case.
            pi_vec.push_back(cur_pin);
        }
        if (cur_pin->type == 2) {  // PO case.
            po_vec.push_back(cur_pin);
        }
    }
    _log->printInt("PI Count", pi_vec.size(), 1);
    _log->printInt("PO Count", po_vec.size(), 1);
    _log->printInt("FlipFlop Count", _flipflop_vec.size(), 1);

    // special case in ICCAD2015 benchmark
    // some filpflops have no output pin.
    uint lack_q_cnt = 0;
    for (auto cur_pin : _pin_vec) {
        if (cur_pin->isFlopInput && !cur_pin->isFlopCkPort) {
            auto& cell = _cell_vec[cur_pin->owner];
            if (stringToId(cell->ports, "q") == UINT_MAX) {
                lack_q_cnt++;
            }
        }
    }
    _log->printInt("Lack of 'q port' Flipflop", lack_q_cnt, 1);

    for (auto cur_pin : pi_vec) {
        std::stack<sequentialElement*> seq_stack;  // stack for DFS.
        sequentialElement* pi = new sequentialElement(cur_pin);
        seq_stack.push(pi);
        _cell2Visited.clear();  // avoid repeated visits to the logic cells.

        ergodicGenerateGraph(seq_stack);  // from the PI, traverse all paths to POs.
    }

    // special case in ICCAD2015 benchmark
    // some flipflops do not come form PI
    for (auto flipflop : _flipflop_vec) {
        if (_is_visited_ff[flipflop->name]) {  // Ignore flipflops that have been visited.
            continue;
        }
        std::stack<sequentialElement*> seq_stack;  // stack for DFS.
        sequentialElement* ff = new sequentialElement(flipflop);
        ff->set_ff_pi();
        seq_stack.push(ff);
        _cell2Visited.clear();  // avoid repeated visits to the logic cells.

        ergodicGenerateGraph(seq_stack);  // from the FlipFlop, traverse all paths to POs.
    }

    printGraphInfo();
}

/**
 * @brief primary function.
 * Use DFS to traverse all paths with a given PI(FlipFlop)
 *
 * @param stack
 */
void sequentialCluster::ergodicGenerateGraph(std::stack<sequentialElement*>& stack) {
    if (stack.empty()) {
        _log->error("NO SEQUENTIALELEMENT HERE.", 1, 1);
        return;
    }

    // sinks is pins of net's load.
    vector<pin*> sinks;

    sequentialElement* cur_seq = stack.top();

    // the sequential element has already be a vetex in graph.
    if (_vertex2Id.find(cur_seq->get_name()) != _vertex2Id.end()) {
        stack.pop();  // must pop current FlipFlop element.
        addSequentialGraph(cur_seq, stack);
        return;
    }

    if (cur_seq->isPi()) {
        pinToSinkPins(cur_seq->get_pio_pin()->id, sinks);
    } else {
        cell* begin_cell = cur_seq->get_cell();
        vector<unsigned> outpins = cellToPins(begin_cell);
        if (outpins.empty()) {
            return;
        }
        for (auto outpin_idx : outpins) {
            pinToSinkPins(outpin_idx, sinks);
        }
    }

    for (auto sink_pin : sinks) {
        if (sink_pin->type == 2) {  // PO case.
            sequentialElement* sink_seq = new sequentialElement(sink_pin);
            addSequentialGraph(sink_seq, stack);
            return;
        }

        // special case in ICCAD2015 contest.
        if (sink_pin->isFlopInput && !sink_pin->isFlopCkPort &&
            stringToId(_cell_vec[sink_pin->owner]->ports, "q") == UINT_MAX) {
            cell* sink_cell = _cell_vec[sink_pin->owner];

            sequentialElement* sink_seq = new sequentialElement(sink_cell);
            // treat it as PO processing
            sink_seq->set_ff_po();
            addSequentialGraph(sink_seq, stack);
            return;
        }

        auto& sink_cell = _cell_vec[sink_pin->owner];

        // The sink_cell has already visited.
        if (_cell2Visited.find(sink_cell->name) != _cell2Visited.end()) {
            continue;
        }
        _cell2Visited[sink_cell->name] = 1;  // signed for avoiding ring in one traverse.

        sequentialElement* sink_seq = new sequentialElement(sink_cell);

        if (sink_pin->isFlopInput && !sink_pin->isFlopCkPort) {
            sink_seq->set_ff();
            stack.push(sink_seq);
        } else {
            stack.push(sink_seq);  // Logic cell
        }

        // next recursive.
        ergodicGenerateGraph(stack);
    }

    sequentialElement* sink_seq = stack.top();  // get the latest sequential element.

    stack.pop();  // pop for advanced sequential element.

    if (sink_seq->isFlipFlop()) {
        addSequentialGraph(sink_seq, stack);
    }
}

/**
 * @brief Given a pin idx, Add the load pins.
 *
 * @param pin_idx
 * @param pins
 */
void sequentialCluster::pinToSinkPins(unsigned pin_idx, vector<pin*>& pins) {
    net* cur_net = _net_vec[_pin_vec[pin_idx]->net];
    if (cur_net->source != pin_idx) {
        _log->error("Pass The ERROR Pin", 1, 1);
    }
    for (auto sink_idx : cur_net->sinks) {
        pins.push_back(_pin_vec[sink_idx]);
    }
    return;
}

/**
 * @brief Given a cell, Return its output pins.
 *
 * @param cell
 * @return vector<unsigned>
 */
vector<unsigned> sequentialCluster::cellToPins(cell* cell) {
    vector<unsigned> outpins;
    std::map<string, unsigned>::iterator iter;
    for (iter = cell->ports.begin(); iter != cell->ports.end(); iter++) {
        uint port_idx = iter->second;
        net* net = _net_vec[_pin_vec[port_idx]->net];
        if (net->source == port_idx) {
            outpins.push_back(port_idx);
        }
    }
    return outpins;
}

/**
 * @brief Given a map and key, Return the value.
 *
 * @param port_map
 * @param port_name
 * @return uint
 */
uint sequentialCluster::stringToId(map<string, unsigned> port_map, string port_name) {
    auto cur_object = port_map.find(port_name);
    if (cur_object == port_map.end()) {
        return UINT_MAX;
    }
    return cur_object->second;
}

/**
 * @brief Return a vertex objcect with a sequential element. flag is true when create a new object.
 *
 * @param seq
 * @param flag
 * @return sequentialVertex*
 */
sequentialVertex* sequentialCluster::makeVertex(sequentialElement* seq, bool& flag) {
    auto v = _vertex2Id.find(seq->get_name());
    if (v == _vertex2Id.end()) {
        _vertex2Id[seq->get_name()] = _graph->get_ff_vertexes().size();

        // visited ff record.
        if (!seq->isPi() && !seq->isPo()) {
            modifyVisitedFFMap(seq->get_name(), true);
        }

        flag = true;
        return new sequentialVertex(seq);

    } else {
        flag = false;
        return _graph->get_ff_vertexes()[v->second];
    }
}

/**
 * @brief Given a sink sequential element, Find its forward src sequential element. And make Graph.
 *
 * @param sink_seq
 * @param stack
 * @return true
 * @return false
 */
bool sequentialCluster::addSequentialGraph(sequentialElement* sink_seq, std::stack<sequentialElement*> stack) {
    if (!sink_seq || stack.empty()) {
        _log->error("ERROR in addGraph", 1, 1);
        return false;
    }

    bool flag1 = true;
    bool flag2 = true;
    vector<cell*> edge_cells;

    while (!stack.empty()) {
        sequentialElement* src_seq = stack.top();
        if (src_seq->isFlipFlop() || src_seq->isPi() || src_seq->isFFPi()) {
            sequentialVertex* src_vertex = makeVertex(src_seq, flag1);
            if (flag1) {
                src_vertex->set_idx(_graph->get_ff_vertexes().size());
                _graph->add_vertex(src_vertex);
            }

            sequentialVertex* sink_vertex = makeVertex(sink_seq, flag2);
            if (flag2) {
                sink_vertex->set_idx(_graph->get_ff_vertexes().size());
                _graph->add_vertex(sink_vertex);
            }

            if (flag1 || flag2) {
                sequentialArc* edge = new sequentialArc(src_vertex, sink_vertex);
                for (auto cell : edge_cells) {
                    edge->add_logic_cells(cell);
                }
                edge->set_idx(_graph->get_ff_edges().size());
                _graph->add_edge(edge);

                src_vertex->add_connect_vertexes(sink_vertex->get_idx());
                src_vertex->add_sink_edges(edge);
                sink_vertex->add_connect_vertexes(src_vertex->get_idx());
                sink_vertex->add_src_edges(edge);

            } else {  // Check whether there are edges between two existing vertices
                bool is_exist = 0;
                for (auto idx : src_vertex->get_connect_vertexes()) {
                    if (idx == sink_vertex->get_idx()) {
                        is_exist = 1;
                        break;
                    }
                }
                if (!is_exist) {
                    sequentialArc* edge = new sequentialArc(src_vertex, sink_vertex);
                    for (auto cell : edge_cells) {
                        edge->add_logic_cells(cell);
                    }
                    edge->set_idx(_graph->get_ff_edges().size());
                    _graph->add_edge(edge);

                    src_vertex->add_connect_vertexes(sink_vertex->get_idx());
                    src_vertex->add_sink_edges(edge);
                    sink_vertex->add_connect_vertexes(src_vertex->get_idx());
                    sink_vertex->add_src_edges(edge);
                }
            }
            return true;
        }
        edge_cells.push_back(src_seq->get_cell());
        stack.pop();
    }
    _log->error("ERROR in addGraph", 1, 1);
    return false;
}

void sequentialCluster::modifyVisitedFFMap(std::string key, bool value) {
    auto iter = _is_visited_ff.find(key);
    if (iter == _is_visited_ff.end()) {
        _log->error("Error occur in ff map", 1, 0);
    } else {
        iter->second = value;
    }
}

void sequentialCluster::printGraphInfo() {
    _log->printInt("Vertexes Count", _graph->get_ff_vertexes().size(), 1);

    uint v_pi = 0, v_po = 0, v_ff_pi = 0, v_ff_po = 0, v_ff = 0;

    for (auto vertex : _graph->get_ff_vertexes()) {
        if (vertex->get_vertex()->isFlipFlop()) {
            v_ff++;
        }
        if (vertex->get_vertex()->isPi()) {
            v_pi++;
        }
        if (vertex->get_vertex()->isFFPi()) {
            v_ff_pi++;
            v_ff++;
        }
        if (vertex->get_vertex()->isPo()) {
            v_po++;
        }
        if (vertex->get_vertex()->isFFPo()) {
            v_ff_po++;
            v_ff++;
        }
    }
    _log->printInt("Vertex:FlipFlop", v_ff, 1);
    _log->printInt("Vertex:PI", v_pi, 1);
    _log->printInt("Vertex:PO", v_po, 1);
    _log->printInt("Vertex:PI(FF)", v_ff_pi, 1);
    _log->printInt("Vertex:PO(FF)", v_ff_po, 1);

    _log->printInt("Edges Count", _graph->get_ff_edges().size(), 1);
}