#include "sequentialCluster.h"

#include <iomanip>
#include <iostream>
#include <sstream>

sequentialOpt::sequentialOpt()
    : _slack_flag(0.0)
    , _log(nullptr)
    , _graph(nullptr)
    , _circuit(nullptr)
    , _core_x(0.0)
    , _core_y(0.0)
    , _max_required_skew(0.0)
    , _max_difference_skew(0.0) {}

sequentialOpt::sequentialOpt(circuit* circuit, Logger* log) : sequentialOpt() {
    _circuit = circuit;
    _log = log;

    _core_x = _circuit->get_rx() - _circuit->get_lx();
    _core_y = _circuit->get_ty() - _circuit->get_by();

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

sequentialOpt::~sequentialOpt() {
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

void sequentialOpt::init() {
    _graph = new sequentialGraph(_log, 100);

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
        std::stack<sequentialBase*> seq_stack;  // stack for DFS.

        // modify.
        sequentialPrimaryIO* pi = new sequentialPrimaryIO(cur_pin);
        pi->set_id(_seq_io_vec.size());
        _seq_io_vec.push_back(pi);

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
        std::stack<sequentialBase*> seq_stack;  // stack for DFS.

        // modify.
        sequentialFlipFlop* ff = new sequentialFlipFlop(flipflop);

        ff->set_ff_pi();
        ff->set_skew(0.0);
        ff->set_id(_seq_ff_vec.size());
        _seq_ff_vec.push_back(ff);

        seq_stack.push(ff);
        _cell2Visited.clear();  // avoid repeated visits to the logic cells.

        ergodicGenerateGraph(seq_stack);  // from the FlipFlop, traverse all paths to POs.
    }

    makeNormalization();
    _graph->updateHop();

    printGraphInfo();
}

/**
 * @brief primary function.
 * Use DFS to traverse all paths with a given PI(FlipFlop)
 *
 * @param stack
 */
void sequentialOpt::ergodicGenerateGraph(std::stack<sequentialBase*>& stack) {
    if (stack.empty()) {
        _log->error("NO SEQUENTIALELEMENT HERE.", 1, 1);
        return;
    }

    // sinks is pins of net's load.
    vector<pin*> sinks;

    sequentialBase* cur_seq = stack.top();

    // the sequential element has already be a vetex in graph.
    if (_vertex2Id.find(cur_seq->get_name()) != _vertex2Id.end()) {
        stack.pop();  // must pop current FlipFlop element.
        addSequentialGraph(cur_seq, stack);
        return;
    }

    if (cur_seq->get_type() == 0) {  // PI case
        auto seq_io = idToSeqIO(cur_seq->get_id());
        pinToSinkPins(seq_io->get_pin()->id, sinks);
    } else {  // FlipFlop/LogicCell case
        cell* begin_cell;
        if (cur_seq->get_type() == 2) {
            auto seq_cell = idToSeqCell(cur_seq->get_id());
            begin_cell = seq_cell->get_logic_cell();
        }

        if (cur_seq->get_type() == 3) {
            auto seq_cell = idToSeqFF(cur_seq->get_id());
            begin_cell = seq_cell->get_flipflop();
        }

        _cell2Visited[begin_cell->name] = 1;  // signed for avoiding ring in one traverse.

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
            sequentialPrimaryIO* sink_seq = new sequentialPrimaryIO(sink_pin);
            sink_seq->set_id(_seq_io_vec.size());
            _seq_io_vec.push_back(sink_seq);

            addSequentialGraph(sink_seq, stack);
            return;
        }

        // special case in ICCAD2015 contest.
        if (sink_pin->isFlopInput && stringToId(_cell_vec[sink_pin->owner]->ports, "q") == UINT_MAX) {
            cell* sink_cell = _cell_vec[sink_pin->owner];

            sequentialFlipFlop* sink_seq = new sequentialFlipFlop(sink_cell, sink_pin, _slack_flag);
            // treat it as PO processing
            sink_seq->set_ff_po();
            sink_seq->set_id(_seq_ff_vec.size());
            _seq_ff_vec.push_back(sink_seq);

            // setting skew
            if (sink_pin->lateSlk <= 0) {
                double required_skew = abs(sink_pin->lateSlk) + sink_pin->earlySlk;
                _required_skews.push_back(required_skew);
                _skews.push_back(sink_pin->lateSlk);
            }

            addSequentialGraph(sink_seq, stack);
            return;
        }

        auto& sink_cell = _cell_vec[sink_pin->owner];

        // The sink_cell has already visited.
        if (_cell2Visited.find(sink_cell->name) != _cell2Visited.end()) {
            continue;
        }
        _cell2Visited[sink_cell->name] = 1;  // signed for avoiding ring in one traverse.

        if (sink_pin->isFlopInput) {
            sequentialFlipFlop* sink_seq = new sequentialFlipFlop(sink_cell, sink_pin, _slack_flag);
            sink_seq->set_id(_seq_ff_vec.size());
            _seq_ff_vec.push_back(sink_seq);

            // record skew
            if (sink_pin->lateSlk <= 0) {
                double required_skew = abs(sink_pin->lateSlk) + sink_pin->earlySlk;
                _required_skews.push_back(required_skew);
                _skews.push_back(sink_pin->lateSlk);
            }

            stack.push(sink_seq);
        } else {
            sequentialLogicCell* sink_seq = new sequentialLogicCell(sink_cell);
            sink_seq->set_id(_seq_cell_vec.size());
            _seq_cell_vec.push_back(sink_seq);

            stack.push(sink_seq);
        }

        // next recursive.
        ergodicGenerateGraph(stack);
    }

    sequentialBase* sink_seq = stack.top();  // get the latest sequential element.

    stack.pop();  // pop for advanced sequential element.

    if (sink_seq->get_type() == 3) {
        addSequentialGraph(sink_seq, stack);
    }
}

/**
 * @brief Given a pin idx, Add the load pins.
 *
 * @param pin_idx
 * @param pins
 */
void sequentialOpt::pinToSinkPins(unsigned pin_idx, vector<pin*>& pins) {
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
vector<unsigned> sequentialOpt::cellToPins(cell* cell) {
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
uint sequentialOpt::stringToId(map<string, unsigned> port_map, string port_name) {
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
sequentialVertex* sequentialOpt::makeVertex(sequentialBase* seq, bool& flag) {
    auto v = _vertex2Id.find(seq->get_name());
    if (v == _vertex2Id.end()) {
        _vertex2Id[seq->get_name()] = _graph->get_vertexes().size();

        // visited ff record.
        if (seq->get_type() == 3) {
            modifyVisitedFFMap(seq->get_name(), true);
        }

        flag = true;
        return new sequentialVertex(seq);

    } else {
        flag = false;
        return _graph->get_vertexes()[v->second];
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
bool sequentialOpt::addSequentialGraph(sequentialBase* sink_seq, std::stack<sequentialBase*> stack) {
    if (!sink_seq) {
        _log->error("ERROR in addGraph", 1, 1);
        return false;
    }
    if (stack.empty()) {
        _log->warn("Current Stack is empty", 1, 1);
        return false;
    }
    bool flag1 = true;
    bool flag2 = true;
    vector<cell*> edge_cells;

    while (!stack.empty()) {
        sequentialBase* src_seq = stack.top();
        if (src_seq->get_type() == 0 || src_seq->get_type() == 3) {  // PI Flipflop case.
            sequentialVertex* src_vertex = makeVertex(src_seq, flag1);
            if (flag1) {
                src_vertex->set_idx(_graph->get_vertexes().size());
                _graph->add_vertex(src_vertex);
                // modify.
                if (src_seq->get_type() == 0) {
                    _graph->add_start_vertex(src_vertex);
                } else {
                    auto src_seq_obj = idToSeqFF(src_seq->get_id());
                    if (src_seq_obj->isFFPi()) {
                        _graph->add_start_vertex(src_vertex);
                    } else {
                        _graph->add_const_vertex(src_vertex);
                    }
                }
            }

            sequentialVertex* sink_vertex = makeVertex(sink_seq, flag2);
            if (flag2) {
                sink_vertex->set_idx(_graph->get_vertexes().size());
                _graph->add_vertex(sink_vertex);
                // modify.
                if (sink_seq->get_type() == 1) {
                    _graph->add_end_vertex(sink_vertex);
                } else {
                    auto sink_seq_obj = idToSeqFF(sink_seq->get_id());
                    if (sink_seq_obj->isFFPo()) {
                        _graph->add_end_vertex(sink_vertex);
                    } else {
                        _graph->add_const_vertex(sink_vertex);
                    }
                }
            }

            if (flag1 || flag2) {
                sequentialArc* edge = new sequentialArc(src_vertex, sink_vertex);
                for (auto cell : edge_cells) {
                    edge->add_logic_cells(cell);
                }
                edge->set_idx(_graph->get_edges().size());
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
                    edge->set_idx(_graph->get_edges().size());
                    _graph->add_edge(edge);

                    src_vertex->add_connect_vertexes(sink_vertex->get_idx());
                    src_vertex->add_sink_edges(edge);
                    sink_vertex->add_connect_vertexes(src_vertex->get_idx());
                    sink_vertex->add_src_edges(edge);
                }
            }
            // add sequential element's direct predecessor
            return true;
        }
        auto seq_obj = idToSeqCell(src_seq->get_id());
        edge_cells.push_back(seq_obj->get_logic_cell());
        stack.pop();
    }
    _log->error("ERROR in addGraph", 1, 1);
    return false;
}

void sequentialOpt::modifyVisitedFFMap(std::string key, bool value) {
    auto iter = _is_visited_ff.find(key);
    if (iter == _is_visited_ff.end()) {
        _log->error("Error occur in ff map", 1, 0);
    } else {
        iter->second = value;
    }
}

void sequentialOpt::printGraphInfo() {
    _log->printInt("Vertexes Count", _graph->get_vertexes().size(), 1);

    uint v_pi = 0, v_po = 0, v_ff_pi = 0, v_ff_po = 0, v_ff = 0;

    for (auto vertex : _graph->get_vertexes()) {
        if (vertex->get_vertex()->get_type() == 3) {
            v_ff++;
            auto seq_obj = idToSeqFF(vertex->get_vertex()->get_id());
            if (seq_obj->isFFPi()) {
                v_ff_pi++;
            }
            if (seq_obj->isFFPo()) {
                v_ff_po++;
            }
        }
        if (vertex->get_vertex()->get_type() == 0) {
            v_pi++;
        }

        if (vertex->get_vertex()->get_type() == 1) {
            v_po++;
        }
    }
    _log->printInt("Vertex:FlipFlop", v_ff, 1);
    _log->printInt("Vertex:PI", v_pi, 1);
    _log->printInt("Vertex:PO", v_po, 1);
    _log->printInt("Vertex:PI(FF)", v_ff_pi, 1);
    _log->printInt("Vertex:PO(FF)", v_ff_po, 1);

    _log->printInt("Edges Count", _graph->get_edges().size(), 1);
}

// void sequentialCluster::test() {
//     auto graph = _graph;
//     std::stack<sequentialVertex*> stack;

//     for (auto& pi : graph->get_start_vertexes()) {
//         stack.push(pi);
//         testDFS(stack);
//     }
// }

// void sequentialCluster::testDFS(std::stack<sequentialVertex*>& stack) {
//     auto cur_v = stack.top();

//     if (cur_v->get_vertex()->get_skew() == DBL_MAX) {
//         _log->warn("Skew has not arranged", 1, 1);
//     }

//     if (cur_v->isEnd()) {
//         stack.pop();
//         return;
//     }

//     for (int i = 0; i < cur_v->get_sink_edges().size(); i++) {
//         auto next_v = cur_v->get_sink_edges()[i]->get_sink();
//         stack.push(next_v);
//         testDFS(stack);
//     }
//     stack.pop();
// }

void sequentialOpt::plot() {
    ofstream dot_seq(_circuit->get_design_name() + ".gds");
    if (!dot_seq.good()) {
        _log->error("Cannot open file for writing", 1, 1);
    }

    stringstream feed;
    feed.precision(0);

    // Header
    feed << "HEADER 5" << endl;
    feed << "BGNLIB" << endl;
    feed << "LIBNAME TDP_Lib" << endl;
    feed << "UNITS 0.0005 1e-9" << endl;
    feed << "BGNSTR" << endl;
    feed << "STRNAME plot" << endl;
    feed << std::fixed << endl;

    // print the die area
    feed << "BOUNDARY" << endl;
    feed << "LAYER 0" << endl;
    feed << "DATATYPE 0" << endl;
    feed << "XY" << endl;
    feed << _circuit->get_lx() << " : " << _circuit->get_by() << endl;
    feed << _circuit->get_rx() << " : " << _circuit->get_by() << endl;
    feed << _circuit->get_rx() << " : " << _circuit->get_ty() << endl;
    feed << _circuit->get_lx() << " : " << _circuit->get_ty() << endl;
    feed << _circuit->get_lx() << " : " << _circuit->get_by() << endl;
    feed << "ENDEL" << endl;
    feed << endl;

    for (auto& vertex : _graph->get_vertexes()) {
        auto seq_obj = idToSeqFF(vertex->get_vertex()->get_id());
        cell* cur_cell = seq_obj->get_flipflop();
        if (cur_cell) {
            feed << "BOUNDARY" << endl;
            feed << "LAYER 1" << endl;
            feed << "DATATYPE 0" << endl;
            feed << "XY" << endl;

            feed << cur_cell->x_coord << " : " << cur_cell->y_coord << endl;
            feed << cur_cell->x_coord + cur_cell->width << " : " << cur_cell->y_coord << endl;
            feed << cur_cell->x_coord + cur_cell->width << " : " << cur_cell->y_coord + cur_cell->height << endl;
            feed << cur_cell->x_coord << " : " << cur_cell->y_coord + cur_cell->height << endl;
            feed << cur_cell->x_coord << " : " << cur_cell->y_coord << endl;

            feed << "ENDEL" << endl;
            feed << endl;
        }
    }

    feed << "ENDSTR" << endl;
    feed << "ENDLIB" << endl;

    dot_seq << feed.str();
    feed.clear();
    dot_seq.close();
}

void sequentialOpt::makeNormalization() {
    std::sort(_required_skews.begin(), _required_skews.end());
    _max_required_skew = _required_skews.back();

    std::sort(_skews.begin(), _skews.end());
    double max_skew = _skews.back();
    double min_skew = _skews.front();
    _max_difference_skew = max_skew - min_skew;
}