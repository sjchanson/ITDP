#include "clusterFF.h"

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
            pi_vec.push_back(cur_pin);
        }
        if (cur_pin->type == 2) {
            po_vec.push_back(cur_pin);
        }

        if (cur_pin->isFlopInput && !cur_pin->isFlopCkPort) {
            flipflop_cnt++;
        }
    }

    _log->printInt("PI COUNT", pi_vec.size());
    _log->printInt("PO COUNT", po_vec.size());
    _log->printInt("FlipFlop COUNT", flipflop_cnt);

    int lack_q_cnt = 0;
    int lack_d_cnt = 0;
    for (auto cur_pin : _pin_vec) {
        if (cur_pin->isFlopInput && !cur_pin->isFlopCkPort) {
            auto& cell = _cell_vec[cur_pin->owner];
            if (stringToId(cell->ports, "q") == UINT_MAX) {
                lack_q_cnt++;
            }
            if (stringToId(cell->ports, "d") == UINT_MAX) {
                lack_d_cnt++;
            }
        }
    }

    // traverse all PI pins.
    for (auto cur_pin : pi_vec) {
        // set the stack for DFS.
        std::stack<sequentialElement*> seq_stack;
        sequentialElement* pi = new sequentialElement(cur_pin);
        seq_stack.push(pi);
        _cell2Visited.clear();
        ergodicGenerateGraph(seq_stack);
    }
    _log->printInt("Vertexes Count", _graph->get_ff_vertexes().size());
    _log->printInt("Edges Count", _graph->get_ff_edges().size());
}

void clusterFF::ergodicGenerateGraph(std::stack<sequentialElement*>& stack) {
    if (stack.empty()) {
        _log->error("NO SEQUENTIALELEMENT HERE.", 1);
        return;
    }

    // sinks is pins of net's load.
    vector<pin*> sinks;

    // get current sequential element.
    sequentialElement* cur_seq = stack.top();

    // for repeated Vertex.
    int flag1 = 1;
    int flag2 = 1;

    if (_vertex2Id.find(cur_seq->get_name()) != _vertex2Id.end()) {
        vector<cell*> edge_cells;
        std::stack<sequentialElement*> copy_stack = stack;
        copy_stack.pop();

        while (!copy_stack.empty()) {
            sequentialElement* src_seq = copy_stack.top();
            if (src_seq->isFlipFlop() || src_seq->isPi()) {
                sequentialVertex* src_vertex = makeVertex(src_seq, flag1);
                if (flag1) {
                    _graph->add_vertex(src_vertex);
                }

                sequentialVertex* sink_vertex = makeVertex(cur_seq, flag2);
                if (flag2) {
                    _graph->add_vertex(sink_vertex);
                }

                if (flag1 || flag2) {
                    sequentialArc* edge = new sequentialArc(src_vertex, sink_vertex);
                    for (auto cell : edge_cells) {
                        edge->add_logic_cells(cell);
                    }
                    _graph->add_edge(edge);

                    src_vertex->add_sink_edges(edge);
                    sink_vertex->add_src_edges(edge);
                }

                break;
            }
            edge_cells.push_back(src_seq->get_cell());
            copy_stack.pop();
        }
        stack.pop();
        return;
    }

    if (cur_seq->isPi()) {
        // pinToSinkPins looking for load pins with given a driven pin.
        sinks = pinToSinkPins(cur_seq->get_pio_pin()->id);
    } else {
        cell* begin_cell = cur_seq->get_cell();
        uint outpin_idx = cellToPin(begin_cell);
        if (outpin_idx == UINT_MAX) {  // The cell has no output pin.
            NOTIMING_END_CELL++;
            return;
        }

        sinks = pinToSinkPins(outpin_idx);
    }

    // traverse all sink pins.
    for (auto sink_pin : sinks) {
        // PO case
        if (sink_pin->type == 2) {
            vector<cell*> edge_cells;
            sequentialElement* sink_seq = new sequentialElement(sink_pin);
            std::stack<sequentialElement*> copy_stack = stack;  // copy_stack for looking pair of Flipflops or Pis/Pos

            while (!copy_stack.empty()) {
                sequentialElement* src_seq = copy_stack.top();
                if (src_seq->isFlipFlop() || src_seq->isPi()) {
                    sequentialVertex* src_vertex = makeVertex(src_seq, flag1);
                    if (flag1) {
                        _graph->add_vertex(src_vertex);
                    }

                    sequentialVertex* sink_vertex = makeVertex(sink_seq, flag2);
                    if (flag2) {
                        _graph->add_vertex(sink_vertex);
                    }

                    if (flag1 || flag2) {
                        sequentialArc* edge = new sequentialArc(src_vertex, sink_vertex);
                        for (auto cell : edge_cells) {
                            edge->add_logic_cells(cell);
                        }
                        _graph->add_edge(edge);

                        src_vertex->add_sink_edges(edge);
                        sink_vertex->add_src_edges(edge);
                    }

                    return;
                }
                edge_cells.push_back(src_seq->get_cell());
                copy_stack.pop();
            }
        }

        // special case in ICCAD2015 contest.
        if (sink_pin->isFlopInput && !sink_pin->isFlopCkPort &&
            stringToId(_cell_vec[sink_pin->owner]->ports, "q") == UINT_MAX) {
            NOOUTPUT_FLIPFLIP++;
            vector<cell*> edge_cells;
            cell* sink_cell = _cell_vec[sink_pin->owner];
            sequentialElement* sink_seq = new sequentialElement(sink_cell);  // FlipFlop
            std::stack<sequentialElement*> copy_stack = stack;

            while (!copy_stack.empty()) {
                sequentialElement* src_seq = copy_stack.top();
                if (src_seq->isFlipFlop() || src_seq->isPi()) {
                    sequentialVertex* src_vertex = makeVertex(src_seq, flag1);
                    if (flag1) {
                        _graph->add_vertex(src_vertex);
                    }

                    sequentialVertex* sink_vertex = makeVertex(sink_seq, flag2);
                    if (flag2) {
                        _graph->add_vertex(sink_vertex);
                    }

                    if (flag1 || flag2) {
                        sequentialArc* edge = new sequentialArc(src_vertex, sink_vertex);
                        for (auto cell : edge_cells) {
                            edge->add_logic_cells(cell);
                        }
                        _graph->add_edge(edge);

                        src_vertex->add_sink_edges(edge);
                        sink_vertex->add_src_edges(edge);
                    }

                    return;
                }
                edge_cells.push_back(src_seq->get_cell());

                copy_stack.pop();
            }
        }

        auto& sink_cell = _cell_vec[sink_pin->owner];

        // The sink_cell has already visited.
        if (_cell2Visited.find(sink_cell) != _cell2Visited.end()) {
            continue;
        }
        _cell2Visited[sink_cell] = 1;  // signed for avoiding ring in graph

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

    sequentialElement* front_seq = stack.top();
    stack.pop();

    if (front_seq->isFlipFlop()) {
        vector<cell*> edge_cells;
        std::stack<sequentialElement*> copy_stack = stack;
        while (!copy_stack.empty()) {
            sequentialElement* pre_seq = copy_stack.top();
            if (pre_seq->isFlipFlop() || pre_seq->isPi()) {
                sequentialVertex* src_vertex = makeVertex(pre_seq, flag1);
                if (flag1) {
                    _graph->add_vertex(src_vertex);
                }

                sequentialVertex* sink_vertex = makeVertex(front_seq, flag2);
                if (flag2) {
                    _graph->add_vertex(sink_vertex);
                }

                if (flag1 || flag2) {
                    sequentialArc* edge = new sequentialArc(src_vertex, sink_vertex);
                    for (auto cell : edge_cells) {
                        edge->add_logic_cells(cell);
                    }
                    _graph->add_edge(edge);

                    src_vertex->add_sink_edges(edge);
                    sink_vertex->add_src_edges(edge);
                }

                return;
            }
            edge_cells.push_back(pre_seq->get_cell());
            copy_stack.pop();
        }
    }
}

vector<pin*> clusterFF::pinToSinkPins(unsigned pin_idx) {
    net* cur_net = _net_vec[_pin_vec[pin_idx]->net];
    if (cur_net->source != pin_idx) {
        _log->error("Pass The ERROR Pin", 1);
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

sequentialVertex* clusterFF::makeVertex(sequentialElement* seq, int& flag) {
    auto v = _vertex2Id.find(seq->get_name());
    if (v == _vertex2Id.end()) {
        _vertex2Id[seq->get_name()] = _graph->get_ff_vertexes().size();
        flag = 1;
        return new sequentialVertex(seq);

    } else {
        flag = 0;
        return _graph->get_ff_vertexes()[v->second];
    }
}

// tape out something inportamvr