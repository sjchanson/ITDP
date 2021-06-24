#include "sequentialOperator.h"

#include <iomanip>
#include <iostream>
#include <sstream>

arrivalDistance::arrivalDistance(sequentialBase* base, double dist) {
    arrival_base = base;
    distance = dist;
}

sequentialOperator::sequentialOperator()
    : _para(nullptr)
    , _log(nullptr)
    , _graph(nullptr)
    , _circuit(nullptr)
    , _core_x(0.0)
    , _core_y(0.0)
    , _max_required_skew(0.0) {}

sequentialOperator::sequentialOperator(parameter* para, circuit* circuit, Logger* log) : sequentialOperator() {
    _para = para;
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
            _is_visited_ff[cell->name] = false;  // init the flipflop visit.
            _flipflop_vec.push_back(cell);
        }
    }

    init();
}

sequentialOperator::~sequentialOperator() {
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

void sequentialOperator::init() {
    _graph = new sequentialGraph(_log);

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

    double begin, end;
    begin = microtime();
    for (auto cur_pin : pi_vec) {
        std::stack<sequentialBase*> seq_stack;  // stack for DFS.

        sequentialPrimaryIO* pi = new sequentialPrimaryIO(cur_pin);

        seq_stack.push(pi);
        _cell2Visited.clear();  // avoid repeated visits to the logic cells.

        ergodicGenerateGraph(seq_stack);  // from the PI, traverse all paths to POs / FlipFlops.
    }

    // special case in ICCAD2015 benchmark
    // some flipflops do not come form PI
    for (auto flipflop : _flipflop_vec) {
        if (_is_visited_ff[flipflop->name]) {  // Ignore flipflops that have been visited.
            continue;
        }
        std::stack<sequentialBase*> seq_stack;  // stack for DFS.

        sequentialFlipFlop* ff = new sequentialFlipFlop(flipflop);
        _flipflops.emplace(ff);  // add.

        ff->set_ff_pi();
        // ff->add_skew(ff, 0.0);

        seq_stack.push(ff);
        _cell2Visited.clear();  // avoid repeated visits to the logic cells.

        ergodicGenerateGraph(seq_stack);  // from the FlipFlop, traverse all paths to POs.
    }
    end = microtime();
    _log->printTime("Init the sequential graph", end - begin, 1);
    printGraphInfo();

    makeNormalization();

    _graph->initHop();
}

/**
 * @brief primary function.
 * Use DFS to traverse all paths with a given PI(FlipFlop)
 *
 * @param stack
 */
void sequentialOperator::ergodicGenerateGraph(std::stack<sequentialBase*>& stack) {
    if (stack.empty()) {
        _log->error("NO SEQUENTIALELEMENT HERE.", 1, 1);
        return;
    }

    // sinks is pins of net's load.
    vector<pin*> sinks;

    sequentialBase* cur_seq = stack.top();

    // visited ff record.
    if (cur_seq->get_type() == 3) {
        modifyVisitedFFMap(cur_seq->get_name(), true);
    }

    // visited logic record.
    if (cur_seq->get_type() == 2) {
        _is_visited_logic[cur_seq->get_name()] = true;
    }

    // the sequential element has already be a vetex in graph.
    if (_graph->isVertexExist(cur_seq->get_name())) {
        return;
    }

    if (cur_seq->get_type() == 0) {  // PI case
        sequentialPrimaryIO* seq_io = dynamic_cast<sequentialPrimaryIO*>(cur_seq);
        pinIdToSinkPins(seq_io->get_pin()->id, sinks);
    } else {  // FlipFlop/LogicCell case
        cell* begin_cell;
        if (cur_seq->get_type() == 2) {
            sequentialLogicCell* seq_cell = dynamic_cast<sequentialLogicCell*>(cur_seq);
            begin_cell = seq_cell->get_logic_cell();
        }

        if (cur_seq->get_type() == 3) {
            sequentialFlipFlop* seq_cell = dynamic_cast<sequentialFlipFlop*>(cur_seq);
            begin_cell = seq_cell->get_flipflop();
        }

        _cell2Visited[begin_cell->name] = 1;  // signed for avoiding ring in one traverse.

        vector<unsigned> outpins = cellToPinIds(begin_cell);
        if (outpins.empty()) {
            return;
        }
        for (auto outpin_idx : outpins) {
            pinIdToSinkPins(outpin_idx, sinks);
        }
    }

    for (auto sink_pin : sinks) {
        if (sink_pin->type == 2) {  // PO case.
            sequentialPrimaryIO* sink_seq = new sequentialPrimaryIO(sink_pin);

            // cur_v is logic cell.
            if (cur_seq->get_type() == 2) {
                sequentialLogicCell* logic_cell = dynamic_cast<sequentialLogicCell*>(cur_seq);
                logic_cell->add_arrival_bases(sink_seq);
            }

            addSequentialGraph(sink_seq, stack);
            return;
        }

        // special case in ICCAD2015 contest.
        if (sink_pin->isFlopInput && stringToId(_cell_vec[sink_pin->owner]->ports, "q") == UINT_MAX) {
            cell* sink_cell = _cell_vec[sink_pin->owner];
            sequentialFlipFlop* sink_seq = new sequentialFlipFlop(sink_cell, sink_pin, _para);
            _flipflops.emplace(sink_seq);  // add.

            // cur_v is logic cell.
            if (cur_seq->get_type() == 2) {
                sequentialLogicCell* logic_cell = dynamic_cast<sequentialLogicCell*>(cur_seq);
                logic_cell->add_arrival_bases(sink_seq);
            }

            // treat it as PO processing
            sink_seq->set_ff_po();
            addSequentialGraph(sink_seq, stack);
            return;
        }

        auto sink_cell = _cell_vec[sink_pin->owner];

        // the logic cell has been visited.
        if (_is_visited_logic[sink_cell->name]) {
            auto sink_logic = _name_to_logic[sink_cell->name];
            for (auto sink_seq : sink_logic->get_arrival_bases()) {
                addSequentialGraph(sink_seq, stack);
            }
            continue;
        }

        // The sink_cell has already visited.
        if (_cell2Visited.find(sink_cell->name) != _cell2Visited.end()) {
            continue;
        }
        _cell2Visited[sink_cell->name] = 1;  // signed for avoiding ring in one traverse.

        if (sink_pin->isFlopInput) {
            sequentialFlipFlop* sink_seq = new sequentialFlipFlop(sink_cell, sink_pin, _para);
            _flipflops.emplace(sink_seq);  // add.

            stack.push(sink_seq);
        } else {
            sequentialLogicCell* sink_seq = new sequentialLogicCell(sink_cell);
            _name_to_logic[sink_seq->get_name()] = sink_seq;
            stack.push(sink_seq);
        }

        // next recursive.
        ergodicGenerateGraph(stack);

        // if sink_seq is the filpflop , backup for searching src.
        sequentialBase* sink_seq = stack.top();  // get the latest sequential element.

        stack.pop();  // pop for advanced sequential element.

        if (sink_seq->get_type() == 2) {
            if (cur_seq->get_type() == 2) {
                sequentialLogicCell* src = dynamic_cast<sequentialLogicCell*>(cur_seq);
                sequentialLogicCell* sink = dynamic_cast<sequentialLogicCell*>(sink_seq);
                vector<sequentialBase*> bases = sink->get_arrival_bases();
                src->copy_bases(bases);
            }
        }

        if (sink_seq->get_type() == 3) {
            addSequentialGraph(sink_seq, stack);
            if (cur_seq->get_type() == 2) {
                // add the arrival flipflop.
                sequentialLogicCell* src = dynamic_cast<sequentialLogicCell*>(cur_seq);
                src->add_arrival_bases(sink_seq);
            }
        }
    }

    // sequentialBase* sink_seq = stack.top();  // get the latest sequential element.

    // break point.
    // if (sink_seq->get_name() == "A1_B18_o307596") {
    //     _log->printItself(sink_seq->get_name(), 1);
    // }

    // stack.pop();  // pop for advanced sequential element.

    // if (sink_seq->get_type() == 3) {
    //     addSequentialGraph(sink_seq, stack);
    // }
}

/**
 * @brief Given a pin idx, Add the load pins.
 *
 * @param pin_idx
 * @param pins
 */
void sequentialOperator::pinIdToSinkPins(unsigned pin_idx, vector<pin*>& pins) {
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
vector<unsigned> sequentialOperator::cellToPinIds(cell* cell) {
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
uint sequentialOperator::stringToId(map<string, unsigned> port_map, string port_name) {
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
sequentialVertex* sequentialOperator::makeVertex(sequentialBase* seq, bool& flag) {
    auto vertex = _graph->get_vertex(seq->get_name());

    if (!vertex) {  // if not found the vertex in graph.
        flag = true;

        // // visited ff record.
        // if (seq->get_type() == 3) {
        //     modifyVisitedFFMap(seq->get_name(), true);
        // }

        sequentialVertex* vertex = new sequentialVertex(seq);
        _graph->add_vertex(seq->get_name(), vertex);

        return vertex;
    } else {
        flag = false;
        return vertex;
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
bool sequentialOperator::addSequentialGraph(sequentialBase* sink_seq, std::stack<sequentialBase*> stack) {
    if (!sink_seq) {
        _log->error("ERROR in addGraph", 1, 1);
        return false;
    }
    if (stack.empty()) {
        // _log->warn("Current Stack is empty", 1, 1);
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
                // Supplementary vertex information
                if (src_seq->get_type() == 0) {
                    // setting skew.
                    src_seq->add_skew(src_seq, 0);
                    src_vertex->set_start();
                    _graph->add_start_vertex(src_vertex);
                } else {
                    sequentialFlipFlop* src_seq_obj = dynamic_cast<sequentialFlipFlop*>(src_seq);
                    if (src_seq_obj->isFFPi()) {
                        // setting skew.
                        src_seq->add_skew(src_seq, 0);
                        src_vertex->set_start();
                        _graph->add_start_vertex(src_vertex);
                    } else {
                        // only the sink_seq setting skew.
                        src_vertex->set_const();
                        _graph->add_const_vertex(src_vertex);
                    }
                }
            }

            sequentialVertex* sink_vertex = makeVertex(sink_seq, flag2);
            if (flag2) {
                // Supplementary vertex information
                if (sink_seq->get_type() == 1) {
                    // PO case no need to set skew.
                    sink_seq->add_skew(src_seq, 0);
                    sink_vertex->set_end();
                    _graph->add_end_vertex(sink_vertex);
                } else {
                    sequentialFlipFlop* sink_seq_obj = dynamic_cast<sequentialFlipFlop*>(sink_seq);

                    if (sink_seq_obj->isFFPo()) {
                        sink_vertex->set_end();
                        _graph->add_end_vertex(sink_vertex);
                    } else {
                        sink_vertex->set_const();
                        _graph->add_const_vertex(sink_vertex);
                    }
                }
            }

            if (flag1 || flag2) {
                sequentialArc* edge = new sequentialArc(src_vertex, sink_vertex);
                for (auto cell : edge_cells) {
                    edge->add_logic_cells(cell);
                }
                _graph->add_edge(edge);

                // Supplementary vertex information
                src_vertex->add_sink_edges(edge);
                sink_vertex->add_src_edges(edge);

            } else {  // Check whether there are edges between two existing vertices
                bool is_exist = 0;
                if (_graph->isDirectHop(src_vertex, sink_vertex)) {
                    is_exist = 1;
                }
                if (!is_exist) {
                    sequentialArc* edge = new sequentialArc(src_vertex, sink_vertex);
                    for (auto cell : edge_cells) {
                        edge->add_logic_cells(cell);
                    }
                    _graph->add_edge(edge);

                    src_vertex->add_sink_edges(edge);
                    sink_vertex->add_src_edges(edge);
                }
            }

            // setting skew
            if (sink_seq->get_type() == 3) {
                sequentialFlipFlop* sink_flipflop = dynamic_cast<sequentialFlipFlop*>(sink_seq);
                auto sink_pin = sink_flipflop->get_input_pin();

                if (sink_pin) {  // avoid ff pi case.
                    if (sink_pin->lateSlk <= 0) {
                        double required_skew = abs(sink_pin->lateSlk) + sink_pin->earlySlk;
                        // should flag in graph.
                        sink_seq->add_skew(src_seq, required_skew);

                        _required_skews.push_back(required_skew);
                    } else {
                        sink_seq->add_skew(src_seq, _para->skew_flag);
                    }
                }
            }

            return true;
        }
        sequentialLogicCell* seq_obj = dynamic_cast<sequentialLogicCell*>(src_seq);
        edge_cells.push_back(seq_obj->get_logic_cell());
        stack.pop();
    }
    _log->error("ERROR in addGraph", 1, 1);
    return false;
}

/**
 * @brief Already confirm that can modify the value.
 *
 * @param key
 * @param value
 */
void sequentialOperator::modifyVisitedFFMap(std::string key, bool value) {
    auto iter = _is_visited_ff.find(key);
    if (iter == _is_visited_ff.end()) {
        _log->error("Error occur in ff map", 1, 0);
    } else {
        iter->second = value;
    }
}

/**
 * @brief init the pair.
 *
 */
void sequentialOperator::initSequentialPair() {
    _graph->updateCoordMapping();
    _graph->initSeqentialPair(_para->side_length, _core_x, _core_y, _max_required_skew);
}

/**
 * @brief major function. do vertex fusion.
 *
 */
void sequentialOperator::updateVertexFusion() {
    int clus_num;
    while (clus_num < _para->clus_num) {
        double begin, end;
        begin = microtime();
        sequentialCluster* new_node = nullptr;

        // sort the pairs.
        _pairs = _graph->get_sequential_pairs();

        if (_pairs.size() < 1) {
            break;
        }

        // get the first pair
        std::set<sequentialPair*, sequentialPairCmp>::iterator sort_iter = _pairs.begin();
        sequentialVertex* v1 = (*sort_iter)->get_vertex1();
        sequentialVertex* v2 = (*sort_iter)->get_vertex2();
        double distance = (*sort_iter)->get_distance();

        // identify the ring.
        if (_graph->findRing(v1, v2)) {
            // update the arrival metrix.
        } else {
            // update the sequential base.
            sequentialBase* base_1 = v1->get_base();
            sequentialBase* base_2 = v2->get_base();

            // make sure here is filpflop or cluster.
            if (base_1->get_type() == 3 && base_2->get_type() == 3) {
                new_node = new sequentialCluster("cluster_" + base_1->get_name());
                // new_node->set_id(_seq_cluster_vec.size());

                sequentialFlipFlop* element_1 = dynamic_cast<sequentialFlipFlop*>(base_1);
                sequentialFlipFlop* element_2 = dynamic_cast<sequentialFlipFlop*>(base_2);

                // add element to cluster.
                new_node->add_flipflop(element_1);
                new_node->add_flipflop(element_2);

                // the element add cluster.
                element_1->set_cluster(new_node);
                element_2->set_cluster(new_node);

            } else if (base_1->get_type() == 3 && base_2->get_type() == 4) {
                new_node = dynamic_cast<sequentialCluster*>(base_2);
                // cluster size control.
                if (new_node->get_subordinate_flipflops().size() > _para->clus_size) {
                    continue;
                }

                sequentialFlipFlop* flipflop = dynamic_cast<sequentialFlipFlop*>(base_1);

                // add element to cluster.
                new_node->add_flipflop(flipflop);
                // the element add cluster.
                flipflop->set_cluster(new_node);
            } else if (base_1->get_type() == 4 && base_2->get_type() == 3) {
                new_node = dynamic_cast<sequentialCluster*>(base_1);
                // cluster size control.
                if (new_node->get_subordinate_flipflops().size() > _para->clus_size) {
                    continue;
                }

                sequentialFlipFlop* flipflop = dynamic_cast<sequentialFlipFlop*>(base_2);

                // add element to cluster.
                new_node->add_flipflop(flipflop);
                // the element add cluster.
                flipflop->set_cluster(new_node);
            } else if (base_1->get_type() == 4 && base_2->get_type() == 4) {
                new_node = dynamic_cast<sequentialCluster*>(base_1);
                // cluster size control.
                if (new_node->get_subordinate_flipflops().size() > _para->clus_size) {
                    continue;
                }

                sequentialCluster* cluster = dynamic_cast<sequentialCluster*>(base_2);
                // cluster size control.
                if (cluster->get_subordinate_flipflops().size() > _para->clus_size) {
                    continue;
                }

                // add all another cluster's element to cur cluster,and delete front cluster.
                std::unordered_set<sequentialFlipFlop*>::iterator iter;
                auto sub_flipflops = cluster->get_subordinate_flipflops();
                for (iter = sub_flipflops.begin(); iter != sub_flipflops.end(); iter++) {
                    new_node->add_flipflop(*iter);
                }
                auto it = _clusters.find(cluster);
                if (it != _clusters.end()) {
                    _clusters.erase(it);
                } else {
                    _log->warn("Error in found the cluster", 1, 1);
                }
                delete cluster;
            } else {
                _log->error("ERROR in update seqential element.", 1, 1);
            }

            // make vertex fusion.
            _graph->makeVertexFusion(v1, v2, new sequentialVertex(new_node), _para->extra_dist);

            // add to the clusters.
            _clusters.emplace(new_node);

            end = microtime();
            _log->printTime("Fusion :" + v1->get_name() + "," + v2->get_name(), end - begin, 2);
        }
        clus_num = _clusters.size();
        if (clus_num % 10 == 0) {
            _log->printInt("Cluster", clus_num, 1);
        }
    }
    replenishCluster();
    // print the final cluster num.
    _log->printInt("Final Cluster", _clusters.size(), 1);
}

/**
 * @brief print the information on screen.
 *
 */
void sequentialOperator::printGraphInfo() {
    _log->printInt("Vertexes Count", _graph->get_vertexes().size(), 1);

    uint v_pi = 0, v_po = 0, v_ff_pi = 0, v_ff_po = 0, v_ff = 0;

    std::unordered_map<std::string, sequentialVertex*>::iterator iter;
    std::unordered_map<std::string, sequentialVertex*> vertexes = _graph->get_vertexes();

    for (iter = vertexes.begin(); iter != vertexes.end(); iter++) {
        if ((*iter).second->get_base()->get_type() == 3) {
            v_ff++;
            auto seq_obj = dynamic_cast<sequentialFlipFlop*>((*iter).second->get_base());
            if (seq_obj->isFFPi()) {
                v_ff_pi++;
            }
            if (seq_obj->isFFPo()) {
                v_ff_po++;
            }
        }
        if ((*iter).second->get_base()->get_type() == 0) {
            v_pi++;
        }

        if ((*iter).second->get_base()->get_type() == 1) {
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

//     if (cur_v->get_base()->get_avg_skew() == DBL_MAX) {
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

void sequentialOperator::replenishCluster() {
    for (auto it = _flipflops.begin(); it != _flipflops.end(); it++) {
        if ((*it)->get_cluster() == nullptr) {
            sequentialCluster* clus = new sequentialCluster((*it)->get_name());
            clus->add_flipflop(*it);
            _clusters.emplace(clus);
        }
    }
}

void sequentialOperator::plot() {
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
        auto base = vertex.second->get_base();

        // pi po case.
        if (base->get_type() == 0 || base->get_type() == 1) {
            continue;
        }
        auto seq_obj = dynamic_cast<sequentialFlipFlop*>(base);  // can do that?
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

void sequentialOperator::makeNormalization() {
    std::sort(_required_skews.begin(), _required_skews.end());
    _max_required_skew = _required_skews.back();
}