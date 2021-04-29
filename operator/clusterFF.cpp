#include "clusterFF.h"

#include <iostream>
#include <mutex>
#include <sstream>

clusterFF::clusterFF() : _graph(nullptr), _circuit(nullptr), _log(nullptr) {}

clusterFF::clusterFF(circuit* circuit, Logger* log) : clusterFF() {
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

clusterFF::~clusterFF() = default;

void clusterFF::init() {
    _graph = new sequentialGraph();

    vector<pin*> pi_vec;
    vector<pin*> po_vec;
    int flipflop_cnt = 0;

    // pick PI_PINS as our begin point.
    for (auto cur_pin : _pin_vec) {
        // skip the "iccad_clk"
        if (cur_pin->name == "iccad_clk") {
            continue;
        }
        if (cur_pin->type == 1) {
            // print
            // _log->printItself(cur_pin->name, 0);
            pi_vec.push_back(cur_pin);
        }
        if (cur_pin->type == 2) {
            // print
            // _log->printItself(cur_pin->name, 0);
            po_vec.push_back(cur_pin);
        }

        if (cur_pin->isFlopInput && !cur_pin->isFlopCkPort) {
            flipflop_cnt++;
        }
    }

    _log->printInt("PI Count", pi_vec.size(), 1);
    _log->printInt("PO Count", po_vec.size(), 1);
    _log->printInt("FlipFlop Count", flipflop_cnt, 1);

    int lack_q_cnt = 0;
    int lack_d_cnt = 0;
    for (auto cur_pin : _pin_vec) {
        if (cur_pin->isFlopInput && !cur_pin->isFlopCkPort) {
            auto& cell = _cell_vec[cur_pin->owner];
            if (stringToId(cell->ports, "q") == UINT_MAX) {
                // print
                // _log->printItself(cell->name, 0);
                lack_q_cnt++;
            }
            if (stringToId(cell->ports, "d") == UINT_MAX) {
                lack_d_cnt++;
            }
        }
    }

    _log->printInt("Lack of 'q' Flipflop", lack_q_cnt, 1);
    _log->printInt("Lack of 'd' Flipflop", lack_d_cnt, 1);

    // traverse all PI pins.
    for (auto cur_pin : pi_vec) {
        // set the stack for DFS.
        std::stack<sequentialElement*> seq_stack;
        sequentialElement* pi = new sequentialElement(cur_pin);
        seq_stack.push(pi);
        _cell2Visited.clear();
        ergodicGenerateGraph(seq_stack);
        // print
        // _log->printItself("ENDITER", 0);
    }

    // traver all not accessed flipflop
    for (auto flipflop : _flipflop_vec) {
        if (_is_visited_ff[flipflop->name]) {
            continue;
        }
        // set the stack for DFS.
        std::stack<sequentialElement*> seq_stack;
        sequentialElement* ff = new sequentialElement(flipflop);
        ff->set_ff_pi();
        seq_stack.push(ff);
        _cell2Visited.clear();
        ergodicGenerateGraph(seq_stack);
    }

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

void clusterFF::ergodicGenerateGraph(std::stack<sequentialElement*>& stack) {
    if (stack.empty()) {
        _log->error("NO SEQUENTIALELEMENT HERE.", 1, 1);
        return;
    }

    // sinks is pins of net's load.
    vector<pin*> sinks;

    // get current sequential element.
    sequentialElement* cur_seq = stack.top();

    if (_vertex2Id.find(cur_seq->get_name()) != _vertex2Id.end()) {
        stack.pop();  // must pop current FlipFlop element.
        addSequentialGraph(cur_seq, stack);
        return;
    }

    if (cur_seq->isPi()) {
        // pinToSinkPins looking for load pins with given a driven pin.
        sinks = pinToSinkPins(cur_seq->get_pio_pin()->id);
    } else {
        cell* begin_cell = cur_seq->get_cell();
        uint outpin_idx = cellToPin(begin_cell);
        if (outpin_idx == UINT_MAX) {  // The cell has no output pin.
            return;
        }
        sinks = pinToSinkPins(outpin_idx);
    }

    // traverse all sink pins.
    for (auto sink_pin : sinks) {
        // PO case
        if (sink_pin->type == 2) {
            sequentialElement* sink_seq = new sequentialElement(sink_pin);
            addSequentialGraph(sink_seq, stack);
            return;
        }

        // special case in ICCAD2015 contest.
        if (sink_pin->isFlopInput && !sink_pin->isFlopCkPort &&
            stringToId(_cell_vec[sink_pin->owner]->ports, "q") == UINT_MAX) {
            cell* sink_cell = _cell_vec[sink_pin->owner];

            // print
            // _log->printItself(sink_cell->name, 0);

            sequentialElement* sink_seq = new sequentialElement(sink_cell);  // FlipFlop
            // must take care.
            sink_seq->set_ff_po();
            addSequentialGraph(sink_seq, stack);
            return;
        }

        auto& sink_cell = _cell_vec[sink_pin->owner];

        // The sink_cell has already visited.
        if (_cell2Visited.find(sink_cell->name) != _cell2Visited.end()) {
            continue;
        }
        _cell2Visited[sink_cell->name] = 1;  // signed for avoiding ring in graph

        sequentialElement* sink_seq = new sequentialElement(sink_cell);

        // FlipFlop case
        if (sink_pin->isFlopInput && !sink_pin->isFlopCkPort) {
            sink_seq->set_ff();
            stack.push(sink_seq);
        } else {
            // Logic cell case.
            stack.push(sink_seq);
        }

        // next recursive.
        ergodicGenerateGraph(stack);
    }

    sequentialElement* sink_seq = stack.top();  // cur_seq has pushed a element.

    stack.pop();  // pop for advanced sequential element.

    if (sink_seq->isFlipFlop()) {
        addSequentialGraph(sink_seq, stack);
    }
}

vector<pin*> clusterFF::pinToSinkPins(unsigned pin_idx) {
    net* cur_net = _net_vec[_pin_vec[pin_idx]->net];
    if (cur_net->source != pin_idx) {
        _log->error("Pass The ERROR Pin", 1, 1);
    }
    vector<pin*> sinks;  // store the sink pins.
    for (auto sink_idx : cur_net->sinks) {
        sinks.push_back(_pin_vec[sink_idx]);
    }
    return sinks;
}

unsigned clusterFF::cellToPin(cell* cell) {
    std::map<string, unsigned>::iterator iter;
    for (iter = cell->ports.begin(); iter != cell->ports.end(); iter++) {
        uint port_idx = iter->second;
        net* net = _net_vec[_pin_vec[port_idx]->net];
        if (net->source == port_idx) {
            return port_idx;
        }
    }
    return UINT_MAX;
}

uint clusterFF::stringToId(map<string, unsigned> port_map, string port_name) {
    auto cur_object = port_map.find(port_name);
    if (cur_object == port_map.end()) {
        // _log->error("PORT " + port_name + " NOT FOUND", 1);
        return UINT_MAX;
    }
    return cur_object->second;
}

sequentialVertex* clusterFF::makeVertex(sequentialElement* seq, bool& flag) {
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

bool clusterFF::addSequentialGraph(sequentialElement* sink_seq, std::stack<sequentialElement*> stack) {
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

                // print
                // _log->printItself(src_seq->get_name(), 0);
            }

            sequentialVertex* sink_vertex = makeVertex(sink_seq, flag2);
            if (flag2) {
                sink_vertex->set_idx(_graph->get_ff_vertexes().size());
                _graph->add_vertex(sink_vertex);

                // print
                // _log->printItself(sink_seq->get_name(), 0);
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

void clusterFF::modifyVisitedFFMap(std::string key, bool value) {
    auto iter = _is_visited_ff.find(key);
    if (iter == _is_visited_ff.end()) {
        _log->error("Error occur in ff map", 1, 0);
    } else {
        iter->second = value;
    }
}