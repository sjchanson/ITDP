/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-04 19:57:39
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-08 22:07:31
 * @Description:
 */

#include "sequentialOperator.h"

#include <math.h>

namespace itdp {

SequentialOperator::SequentialOperator(AdapterInterface* data_interface) {
    _data_interface = data_interface;
    _sequential_base = new SequentialBase();
    _skew_constraint_graph = new SkewConstraintGraph("origin");
    _log = Logger::get_logger_obj("xxx", 0);
    _parameter = Parameter::get_parameter_pointer();
    init();
}

SequentialOperator::~SequentialOperator() {
    delete _sequential_base;
    delete _skew_constraint_graph;
}

void SequentialOperator::cleanClusters() { clusters_map.clear(); }

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

    // Termination conditions.
    if (current_element->isPo()) {
        // Add graph vertices and edges.
        std::vector<SequentialElement*> elements = {current_element};
        addSequentialGraph(elements, stack);
        stack.pop();
        return;
    }
    if (process_data->isInstanceAccessed(current_element->get_name())) {
        if (current_element->isFlipFlop()) {
            // Add graph vertices and edges.
            std::vector<SequentialElement*> elements = {current_element};
            addSequentialGraph(elements, stack);
        } else if (current_element->isLogic()) {
            // Add graph vertices and edges.
            auto current_logic = dynamic_cast<SequentialLogic*>(current_element);
            std::vector<SequentialElement*> elements = current_logic->get_accessible_elements();
            if (!elements.empty()) {  // logic has no flipflop connection.
                addSequentialGraph(elements, stack);
            }
        } else {
            // PI/PO case is impossible.
        }
        stack.pop();
        return;
    }
    // Instance already accessed by the current path.
    if (accessed_instance.find(current_element->get_name()) != accessed_instance.end()) {
        stack.pop();
        return;
    }
    accessed_instance.emplace(current_element->get_name());

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
        SequentialElement* sink_element =
            addSequentialElement(sink_pin, current_element, stack, accessed_instance, process_data);
        if (sink_element == nullptr) {
            _log->error(sink_pin->get_name() + " doesn't belong to an instance", 1, 0);
        } else {
            if (sink_element->isPo() && current_element->isLogic()) {
                SequentialLogic* logic_element = dynamic_cast<SequentialLogic*>(current_element);
                logic_element->add_accessible_element(sink_element);
            }
        }

        // next recursive.
        ergodicGenerateGraph(stack, accessed_instance, process_data);
        // backtraking.
        if (current_element->isFlipFlop() || current_element->isLogic()) {
            if (current_element->isFlipFlop()) {
                // Add graph vertices and edges.
                std::vector<SequentialElement*> elements = {current_element};
                addSequentialGraph(elements, stack);
                // add accessible instance.
            } else {
                if (sink_element->isFlipFlop()) {
                    SequentialLogic* current_logic = dynamic_cast<SequentialLogic*>(current_element);
                    current_logic->add_accessible_element(sink_element);
                }
                if (sink_element->isLogic()) {
                    SequentialLogic* current_logic = dynamic_cast<SequentialLogic*>(current_element);
                    SequentialLogic* sink_logic = dynamic_cast<SequentialLogic*>(sink_element);
                    current_logic->add_batch_elements(sink_logic->get_accessible_elements());
                }
            }
            process_data->add_accessed_instance(current_element->get_name());
        }
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
                                                            std::stack<SequentialElement*>& stack,
                                                            std::set<std::string>& accessed_instance,
                                                            ProcessData* process_data) {
    SequentialElement* sink_element;
    if (sink_pin->isPO()) {
        SequentialPO* sink_po = _sequential_base->get_sequential_po(sink_pin->get_name());
        if (!sink_po) {
            sink_po = new SequentialPO(sink_pin);
            // add skew to the sequential po.
            sink_po->add_name_to_skew(src_element->get_name(), calculateSetupSkew(sink_pin));
            _sequential_base->add_po(sink_po);
        }
        stack.push(sink_po);
        sink_element = sink_po;
        return sink_element;
    } else {
        Instance* sink_instance = sink_pin->get_instance();
        // Create sequential element.
        if (sink_instance->isFlipflop()) {
            SequentialFlipFlop* sink_flipflop = _sequential_base->get_sequential_flipflop(sink_instance->get_name());
            if (!sink_flipflop) {
                sink_flipflop = new SequentialFlipFlop(sink_instance);
                // add skew to the sequential flipflop.
                sink_flipflop->add_name_to_skew(src_element->get_name(), calculateSetupSkew(sink_pin));
                _sequential_base->add_flipflop(sink_flipflop);
                process_data->add_accessed_flipflop(sink_instance->get_name());
            }
            stack.push(sink_flipflop);
            sink_element = sink_flipflop;
        } else {
            SequentialLogic* sink_logic = process_data->get_sequential_logic(sink_instance->get_name());
            if (!sink_logic) {
                sink_logic = new SequentialLogic(sink_instance);
                process_data->add_logic_map(sink_logic);
            }
            stack.push(sink_logic);
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

    duplicate_stack.pop();  // pop the current element so that find the source element.

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

        if (existence_flag1 || existence_flag2) {
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

}  // namespace itdp