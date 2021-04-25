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
    }

    // traverse all PI pins.
    for (auto cur_pin : pi_vec) {
        // set the stack for DFS.
        std::stack<sequentialElement*> seq_stack;
        sequentialElement* pi = new sequentialElement(cur_pin);
        seq_stack.push(pi);
        ergodicGenerateGraph(seq_stack);
    }
    _log->printInt("Vertexes Count", _graph->get_ff_vertexes().size());
    _log->printInt("Edges Count", _graph->get_ff_edges().size());
}

void clusterFF::ergodicGenerateGraph(std::stack<sequentialElement*>& stack) {
    if (stack.empty()) {
        _log->warn("NO SEQUENTIALELEMENT HERE.", 0);
        return;
    }
    sequentialElement* cur_seq = stack.top();

    // if current element has been store. that stop.
    for (auto cur_vertex : _graph->get_ff_vertexes()) {
        if (cur_vertex->get_vertex()->get_name() == cur_seq->get_name()) {
            return;
        }
    }

    vector<pin*> sinks;
    cell* begin_cell;
    // if current is PI
    if (cur_seq->isPi()) {
        sinks = pinToSinkPins(cur_seq->get_pio_pin()->id);
    } else {
        begin_cell = cur_seq->get_cell();
        uint outpin_idx = cellToPin(begin_cell);
        if (outpin_idx == UINT_MAX) {
            NOTIMING_END_CELL++;
            return;
            // _log->error("CELL HAS NO OUTPUT PIN", 1);
        }
        sinks = pinToSinkPins(outpin_idx);
    }

    for (auto sink_pin : sinks) {
        // PO case
        if (sink_pin->type == 2) {
            sequentialElement* seq = new sequentialElement(sink_pin);  // PO
            std::stack<sequentialElement*> copy_stack = stack;
            vector<cell*> edge_cells;
            while (!copy_stack.empty()) {
                sequentialElement* pre_seq = copy_stack.top();
                if (pre_seq->isFlipFlop() || pre_seq->isPi()) {
                    sequentialVertex* src_vertex = new sequentialVertex(pre_seq);
                    _graph->add_vertex(src_vertex);

                    sequentialVertex* sink_vertex = new sequentialVertex(seq);
                    _graph->add_vertex(sink_vertex);

                    sequentialArc* edge = new sequentialArc(src_vertex, sink_vertex);
                    for (auto cell : edge_cells) {
                        edge->add_logic_cells(cell);
                    }
                    _graph->add_edge(edge);
                    return;
                }
                edge_cells.push_back(pre_seq->get_cell());
                copy_stack.pop();
            }
        }

        // special case in ICCAD2015 contest.
        if (sink_pin->isFlopInput && stringToId(_cell_vec[sink_pin->owner]->ports, "q") == UINT_MAX) {
            NOOUTPUT_FLIPFLIP++;
            cell* cur_cell = _cell_vec[sink_pin->owner];
            sequentialElement* seq = new sequentialElement(cur_cell);  // FlipFlop
            std::stack<sequentialElement*> copy_stack = stack;
            vector<cell*> edge_cells;
            while (!copy_stack.empty()) {
                sequentialElement* pre_seq = copy_stack.top();
                if (pre_seq->isFlipFlop() || pre_seq->isPi()) {
                    sequentialVertex* src_vertex = new sequentialVertex(pre_seq);
                    _graph->add_vertex(src_vertex);

                    sequentialVertex* sink_vertex = new sequentialVertex(seq);
                    _graph->add_vertex(sink_vertex);

                    sequentialArc* edge = new sequentialArc(src_vertex, sink_vertex);
                    for (auto cell : edge_cells) {
                        edge->add_logic_cells(cell);
                    }
                    _graph->add_edge(edge);
                    return;
                }
                edge_cells.push_back(pre_seq->get_cell());
                copy_stack.pop();
            }
        }

        cell* cur_cell = _cell_vec[sink_pin->owner];
        sequentialElement* next_seq = new sequentialElement(cur_cell);

        // avoid ring in graph
        bool is_existed = 0;
        std::stack<sequentialElement*> copy_stack = stack;
        while (!copy_stack.empty()) {
            sequentialElement* tmp_seq = copy_stack.top();
            if (next_seq->get_name() == tmp_seq->get_name()) {
                is_existed = 1;
                break;
            }
            copy_stack.pop();
        }
        if (!is_existed) {
            // FlipFlop case
            if (sink_pin->isFlopInput) {
                next_seq->set_ff();
                stack.push(next_seq);
            } else {
                // Logic cell case.
                stack.push(next_seq);
            }

            // next recursive.
            ergodicGenerateGraph(stack);
        } else {
            return;
        }
    }
    sequentialElement* front_seq = stack.top();
    stack.pop();

    if (front_seq->isFlipFlop()) {
        std::stack<sequentialElement*> copy_stack = stack;
        vector<cell*> edge_cells;
        while (!copy_stack.empty()) {
            sequentialElement* pre_seq = copy_stack.top();
            if (pre_seq->isFlipFlop() || pre_seq->isPi()) {
                sequentialVertex* src_vertex = new sequentialVertex(pre_seq);
                _graph->add_vertex(src_vertex);

                // avoid repeat vertex.
                sequentialVertex* sink_vertex = nullptr;
                for (auto cur_vertex : _graph->get_ff_vertexes()) {
                    if (front_seq->get_name() == cur_vertex->get_vertex()->get_name()) {
                        sink_vertex = cur_vertex;
                    }
                }

                if (!sink_vertex) {
                    sink_vertex = new sequentialVertex(front_seq);
                    _graph->add_vertex(sink_vertex);
                }

                sequentialArc* edge = new sequentialArc(src_vertex, sink_vertex);
                for (auto cell : edge_cells) {
                    edge->add_logic_cells(cell);
                }
                _graph->add_edge(edge);
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