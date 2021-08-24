/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-04 19:57:39
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-24 20:47:56
 * @Description:
 */

#include "sequentialOperator.h"

#include <math.h>

namespace itdp {
class ProcessData {
public:
    ProcessData() = default;
    ~ProcessData();

    // getter.
    int get_logic_size() const { return _logic_map.size(); }
    bool isInstanceAccessed(std::string name);
    bool isFlipflopAccessed(std::string name);
    SequentialLogic* get_sequential_logic(std::string name) const;

    // setter.
    void add_accessed_instance(std::string name) { _instance_visited_record.emplace(name); }
    void add_accessed_flipflop(std::string name) { _flipflop_visited_record.emplace(name); }
    void add_logic_map(SequentialLogic* logic) { _logic_map.emplace(logic->get_name(), logic); }

private:
    std::set<std::string> _instance_visited_record;
    std::set<std::string> _flipflop_visited_record;
    std::map<std::string, SequentialLogic*> _logic_map;
};
inline ProcessData::~ProcessData() {
    _instance_visited_record.clear();
    _flipflop_visited_record.clear();
    for (auto logic : _logic_map) {
        delete logic.second;
    }
    _logic_map.clear();
}
inline bool ProcessData::isInstanceAccessed(std::string name) {
    return _instance_visited_record.find(name) != _instance_visited_record.end();
}
inline bool ProcessData::isFlipflopAccessed(std::string name) {
    return _flipflop_visited_record.find(name) != _flipflop_visited_record.end();
}
inline SequentialLogic* ProcessData::get_sequential_logic(std::string name) const {
    auto iter = _logic_map.find(name);
    if (iter == _logic_map.end()) {
        return nullptr;
    } else {
        return (*iter).second;
    }
}
}  // namespace itdp

namespace itdp {

SequentialOperator::SequentialOperator(AdapterInterface* data_interface) {
    _data_interface = data_interface;
    _sequential_base = new SequentialBase();
    _skew_constraint_graph = new SkewConstraintGraph("origin");
    _log = Logger::get_logger_obj("xxx", 0);
    _parameter = Parameter::get_parameter_pointer();
    _parameter->set_core_x(data_interface->get_core_edge_x());
    _parameter->set_core_y(data_interface->get_core_edge_y());
    _parameter->set_row_height(data_interface->get_row_height());
    _max_required_skew = 0;
    init();
}

SequentialOperator::~SequentialOperator() {
    delete _sequential_base;
    delete _skew_constraint_graph;
}

/**
 * @description: Clean the exist clusters.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialOperator::cleanClusters() { _clusters_map.clear(); }

void SequentialOperator::initDistanceMatrix() { _skew_constraint_graph->initDistanceMatrix(); }

/**
 * @description: Init the sequential element and sequential graph.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialOperator::init() {
    auto pin_pvec = _data_interface->get_pin_pvec();
    auto instance_pvec = _data_interface->get_instance_pvec();
    auto net_pvec = _data_interface->get_net_pvec();
    std::vector<Pin*> primary_inputs;
    std::vector<Instance*> flipflops;
    ProcessData* process_data = new ProcessData();

    for (auto pin : pin_pvec) {
        if (pin->get_name() == "iccad_clk") {
            continue;
        }
        if (pin->isPI()) {
            primary_inputs.push_back(pin);
        } else if (pin->isFlipFlopClk()) {
            Instance* inst = pin->get_instance();
            flipflops.push_back(inst);
        } else {
            continue;
        }
    }

    for (auto pi : primary_inputs) {
        SequentialPI* sequential_pi = new SequentialPI(pi);
        _sequential_base->add_pi(sequential_pi);
        std::stack<SequentialElement*> stack;
        std::set<std::string> accessed_instance;
        stack.push(sequential_pi);
        ergodicGenerateGraph(stack, accessed_instance, process_data);
    }

    // handle the flipflops that haven't been visited.
    for (auto flipflop : flipflops) {
        if (process_data->isFlipflopAccessed(flipflop->get_name())) {
            continue;
        } else {
            SequentialFlipFlop* remain_flipflop = new SequentialFlipFlop(flipflop);
            _sequential_base->add_flipflop(remain_flipflop);
            SequentialVertex* remain_vertex = new SequentialVertex(remain_flipflop);
            _skew_constraint_graph->add_sequential_vertex(remain_vertex);
        }
    }

    // set the max required skew.
    _parameter->set_max_required_skew(_max_required_skew);

    // print the circuit info.
    _log->printInt("Sequential PI", _sequential_base->get_sequential_pi_size(), 1);
    _log->printInt("Sequential PO", _sequential_base->get_sequential_po_size(), 1);
    _log->printInt("Sequential FlipFlop", _sequential_base->get_sequential_flipflop_size(), 1);
    _log->printInt("Sequential Logic", process_data->get_logic_size(), 1);
    _log->printInt("Sequential Vertex", _skew_constraint_graph->get_sequential_vertexes_size(), 1);
    _log->printInt("Sequential Edges", _skew_constraint_graph->get_sequential_edges_size(), 1);

    delete process_data;
}

/**
 * @description: select the max skew to normalize.
 * @param {double} skew
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialOperator::add_max_required_skew(double skew) {
    if (skew > _max_required_skew) {
        _max_required_skew = skew;
    }
}

/**
 * @description: Primary function. Use DFS to traverse all paths with a given primary input.
 * @param {std::stack<SequentialElement*>&}
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialOperator::ergodicGenerateGraph(std::stack<SequentialElement*>& stack,
                                              std::set<std::string>& accessed_instance, ProcessData* process_data) {
    if (stack.empty()) {
        _log->error("There are no elements in the current stack", 1, 0);
    }
    SequentialElement* current_element = stack.top();
    std::vector<Pin*> sink_pins;  // pins of net's load.
    obtainSinkPins(current_element, sink_pins);
    // for those instances have no output.
    if (sink_pins.empty()) {
        // _log->warn(current_element->get_name() + " has no output", 1, 0);
        if (current_element->isFlipFlop()) {
            std::vector<SequentialElement*> elements = {current_element};
            addSequentialGraph(elements, stack);
        } else {
            // Logic case no need to manage.
        }
    }
    // Handle the sink elements of sink pins.
    for (Pin* sink_pin : sink_pins) {
        SequentialElement* sink_element = addSequentialElement(sink_pin, current_element, process_data);
        // Termination conditions.
        if (sink_element->isPo()) {
            // Add graph vertices and edges.
            std::vector<SequentialElement*> elements = {sink_element};
            addSequentialGraph(elements, stack);
            if (current_element->isLogic()) {
                SequentialLogic* logic_element = dynamic_cast<SequentialLogic*>(current_element);
                logic_element->add_accessible_element(sink_element);
            }
            continue;
        }
        if (process_data->isInstanceAccessed(sink_element->get_name())) {
            if (sink_element->isFlipFlop()) {
                // Add graph vertices and edges.
                std::vector<SequentialElement*> elements = {sink_element};
                addSequentialGraph(elements, stack);
            } else if (sink_element->isLogic()) {
                // Add graph vertices and edges.
                auto sink_logic = dynamic_cast<SequentialLogic*>(sink_element);
                std::vector<SequentialElement*> elements = sink_logic->get_accessible_elements();
                if (!elements.empty()) {  // logic has no flipflop connection.
                    addSequentialGraph(elements, stack);
                }
            } else {
                // PI/PO case is impossible.
            }
            continue;
        }
        // Instance already accessed by the current path.
        if (accessed_instance.find(sink_element->get_name()) != accessed_instance.end()) {
            continue;
        }
        accessed_instance.emplace(sink_element->get_name());
        stack.push(sink_element);
        // next recursive.
        ergodicGenerateGraph(stack, accessed_instance, process_data);
        // backtraking.
        if (sink_element->isFlipFlop()) {
            // Add graph vertices and edges.
            std::vector<SequentialElement*> elements = {sink_element};
            addSequentialGraph(elements, stack);
            if (current_element->isLogic()) {
                SequentialLogic* current_logic = dynamic_cast<SequentialLogic*>(current_element);
                current_logic->add_accessible_element(sink_element);
            }
            // add accessible instance.
        } else if (sink_element->isLogic()) {
            if (current_element->isLogic()) {
                SequentialLogic* current_logic = dynamic_cast<SequentialLogic*>(current_element);
                SequentialLogic* sink_logic = dynamic_cast<SequentialLogic*>(sink_element);
                current_logic->add_batch_elements(sink_logic->get_accessible_elements());
            }
        } else {
            //
        }
        process_data->add_accessed_instance(current_element->get_name());
    }
    stack.pop();
}

/**
 * @description: Build the sequential element, and tag the skew.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
SequentialElement* SequentialOperator::addSequentialElement(Pin* sink_pin, SequentialElement* src_element,
                                                            ProcessData* process_data) {
    SequentialElement* sink_element;
    if (sink_pin->isPO()) {
        SequentialPO* sink_po = _sequential_base->get_sequential_po(sink_pin->get_name());
        if (!sink_po) {
            sink_po = new SequentialPO(sink_pin);
            _sequential_base->add_po(sink_po);
        }
        sink_element = sink_po;
        return sink_element;
    } else {
        Instance* sink_instance = sink_pin->get_instance();
        // Create sequential element.
        if (sink_instance->isFlipflop()) {
            SequentialFlipFlop* sink_flipflop = _sequential_base->get_sequential_flipflop(sink_instance->get_name());
            if (!sink_flipflop) {
                sink_flipflop = new SequentialFlipFlop(sink_instance);

                _sequential_base->add_flipflop(sink_flipflop);
                process_data->add_accessed_flipflop(sink_instance->get_name());
            }
            sink_element = sink_flipflop;
        } else {
            SequentialLogic* sink_logic = process_data->get_sequential_logic(sink_instance->get_name());
            if (!sink_logic) {
                sink_logic = new SequentialLogic(sink_instance);
                process_data->add_logic_map(sink_logic);
            }
            sink_element = sink_logic;
        }
        return sink_element;
    }
}

/**
 * @description: Calculate the skew according to the pin.
 * @param {const Pin*} pin
 * @return {*}
 * @author: ShiJian Chen
 */
double SequentialOperator::calculateSetupSkew(const Pin* pin) {
    double setup_skew;
    if (pin->get_late_slack() <= 0) {
        setup_skew = (fabs(pin->get_late_slack()) + pin->get_early_slack()) / 2;
    } else {
        setup_skew = _parameter->get_skew_flag();
    }
    return setup_skew;
}

/**
 * @description: According to the sink, find the source and build the graph.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialOperator::addSequentialGraph(std::vector<SequentialElement*> sink_elements,
                                            std::stack<SequentialElement*> duplicate_stack) {
    if (sink_elements.empty()) {
        _log->error("There are no sinks to add sequential graph", 1, 0);
    }
    if (duplicate_stack.empty()) {
        _log->warn("Duplicate stack is empty when addSequentialGraph", 1, 0);
    }

    // find the source_element(pi/flipflop).
    while (!duplicate_stack.empty()) {
        SequentialElement* source_element = duplicate_stack.top();
        if (source_element->isPi() || source_element->isFlipFlop()) {
            // add vertexes and edge.
            addVertexes(sink_elements, source_element);
            return;
        } else if (source_element->isLogic()) {
            duplicate_stack.pop();
        } else {
            _log->warn("The source_element type maybe wrong", 1, 0);
        }
    }
    _log->error("There is no source in duplicate_stack", 1, 0);
}

/**
 * @description: Add vertices and edges to the graph.
 * @param {vector<SequentialElement*>} sink_elements
 * @param {SequentialElement*} source_element
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialOperator::addVertexes(std::vector<SequentialElement*> sink_elements, SequentialElement* source_element) {
    bool existence_flag1, existence_flag2;
    for (auto sink : sink_elements) {
        // add skew to the sequential flipflop.
        if (sink->isFlipFlop()) {
            SequentialFlipFlop* sink_flipflop = static_cast<SequentialFlipFlop*>(sink);
            double skew = calculateSetupSkew(sink_flipflop->get_data_input_pin());
            // max required skew used for flipflop cluster.
            add_max_required_skew(skew);
            sink_flipflop->add_name_to_skew(source_element->get_name(), skew);
        }
        // add skew to the sequential po.
        if (sink->isPo()) {
            SequentialPO* sink_po = static_cast<SequentialPO*>(sink);
            double skew = calculateSetupSkew(sink_po->get_pin());
            sink_po->add_name_to_skew(source_element->get_name(), skew);
        }

        // if the sink is itself
        if (sink->get_name() == source_element->get_name()) {
            continue;
        }

        SequentialVertex* source_vertex = _skew_constraint_graph->get_existent_vertex(source_element->get_name());
        source_vertex == nullptr ? existence_flag1 = false : existence_flag1 = true;
        if (!existence_flag1) {
            source_vertex = new SequentialVertex(source_element);
            _skew_constraint_graph->add_sequential_vertex(source_vertex);
        }

        SequentialVertex* sink_vertex = _skew_constraint_graph->get_existent_vertex(sink->get_name());
        sink_vertex == nullptr ? existence_flag2 = false : existence_flag2 = true;
        if (!existence_flag2) {
            sink_vertex = new SequentialVertex(sink);
            _skew_constraint_graph->add_sequential_vertex(sink_vertex);
        }

        if (!existence_flag1 || !existence_flag2) {
            SequentialEdge* edge = new SequentialEdge(source_vertex, sink_vertex);
            _skew_constraint_graph->add_sequential_edge(edge);
            source_vertex->add_sink_edge(edge);
            sink_vertex->add_src_edge(edge);
        } else {  // Check whether there are edges between two existing vertexes.
            bool is_edge_existence =
                _skew_constraint_graph->isExistentEdge(source_vertex->get_name(), sink_vertex->get_name());
            if (!is_edge_existence) {
                SequentialEdge* edge = new SequentialEdge(source_vertex, sink_vertex);
                _skew_constraint_graph->add_sequential_edge(edge);
                source_vertex->add_sink_edge(edge);
                sink_vertex->add_src_edge(edge);
            }
        }
    }
}

/**
 * @description: Find the load pin for this element.
 * @param {SequentialElement*} element
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialOperator::obtainSinkPins(SequentialElement* element, std::vector<Pin*>& sink_pins) {
    if (element->isPi()) {
        SequentialPI* pi_element = dynamic_cast<SequentialPI*>(element);
        sink_pins = pi_element->get_pin()->get_net()->get_sink_pins();
    } else if (element->isFlipFlop() || element->isLogic()) {
        std::vector<Pin*> instance_output_pins;
        if (element->isFlipFlop()) {
            SequentialFlipFlop* flipflop_element = dynamic_cast<SequentialFlipFlop*>(element);
            instance_output_pins = flipflop_element->get_instance()->get_outpin_vec();
        } else {
            SequentialLogic* logic_element = dynamic_cast<SequentialLogic*>(element);
            auto logic_instance = logic_element->get_logic_cell();
            instance_output_pins = logic_instance->get_outpin_vec();
        }
        for (auto out_pin : instance_output_pins) {
            auto net_sink_pins = out_pin->get_net()->get_sink_pins();
            sink_pins.insert(sink_pins.end(), net_sink_pins.begin(), net_sink_pins.end());
        }
    } else {
        _log->error("Break traversal. Element: " + element->get_name(), 1, 0);
    }
}

/**
 * @description: Finish the sequential cluster.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialOperator::sequentialClusterSolve() {
    cleanClusters();

    auto map = _skew_constraint_graph->makeVertexesFusion();
    for (auto pair : map) {
        std::string buffer_name = "buffer_" + std::to_string(_sequential_base->get_sequential_buffer_size());
        // add buffer and build the clockTree.
        SequentialBuffer* buffer = new SequentialBuffer(buffer_name);
        _sequential_base->add_buffer(buffer, pair.second);
        // change the vertex name.
        _skew_constraint_graph->modifyVertexName(pair.first, buffer);
        // add to the _clusters_map.
        _clusters_map.emplace(buffer_name, pair.second);
    }

    // reset the sink element's belonging.
    _sequential_base->resetBelonging();
}

/**
 * @description: Run the InitLevelCluster.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void SequentialOperator::sloveInitLevelCluster() {
    _clusters_map.clear();

    std::map<std::string, std::vector<const SequentialElement*>> tmp_map;
    tmp_map = _skew_constraint_graph->subgraphPartition(_parameter->get_pre_clus_size());

    // update the origin graph.
    for (auto pair : tmp_map) {
        // create the local buffer.
        std::string buffer_name = "buffer_" + std::to_string(_sequential_base->get_sequential_buffer_size());
        SequentialBuffer* local_buffer = new SequentialBuffer(buffer_name);
        _sequential_base->add_buffer(local_buffer, pair.second);

        _skew_constraint_graph->updateGraphConnectionFromSubGraph(buffer_name, pair.second, local_buffer);

        _clusters_map.emplace(buffer_name, pair.second);
    }
}

void SequentialOperator::buildNextLevelCluster(std::map<std::string, Point<DBU>> _coord_map) {
    // set the new coord of buffer.
    for (auto pair : _coord_map) {
        SequentialBuffer* buffer = _sequential_base->get_sequential_buffer(pair.first);
        buffer->set_center_coord(pair.second);
    }

    _skew_constraint_graph->initDistanceMatrix();
    sequentialClusterSolve();
}

}  // namespace itdp