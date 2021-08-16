/*
 * @Author: ShiJian Chen
 * @Date: 2021-07-31 16:51:45
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-12 14:39:34
 * @Description:
 */

#include "../include/iccad2015Adapter.h"

namespace itdp {

Iccad2015Adapter::Iccad2015Adapter(std::shared_ptr<circuit> circuit) {
    _log = Logger::get_logger_obj("xxx", 0);
    _log->displayTitle(0);
    _circuit = circuit;
    _design_name = circuit->get_design_name();
    _core_edge_x = circuit->get_rx() - circuit->get_lx();
    _core_edge_y = circuit->get_ty() - circuit->get_by();
    _row_height = static_cast<DBU>(circuit->get_row_height());
    dataTransmit();
}

Iccad2015Adapter::~Iccad2015Adapter() {
    for (auto inst : _instance_pvec) {
        delete inst;
    }
    for (auto net : _net_pvec) {
        delete net;
    }
    for (auto pin : _pin_pvec) {
        delete pin;
    }
}

/**
 * @description: Convert iccad circuit to Iccad2015Adapter.
 * @param {*}
 * @return {*}
 * @author: ShiJian Chen
 */
void Iccad2015Adapter::dataTransmit() {
    auto& macro_vec = _circuit->getMacros();
    auto& cell_vec = _circuit->getCells();
    auto& net_vec = _circuit->getNets();
    auto& pin_vec = _circuit->getPins();

    // initialize instance pointer.
    _instance_pvec.reserve(cell_vec.size());
    for (auto& cell : cell_vec) {
        Instance* inst_pointer = new Instance(cell.name);
        // state.
        cell.isFixed ? inst_pointer->set_state(kFixed) : inst_pointer->set_state(kPlaceable);
        // type.
        if (cell.isLCB) {
            inst_pointer->set_type(kLCB);
        } else if (macro_vec[cell.type].isFlop) {
            inst_pointer->set_type(kFlipFlop);
        } else {
            inst_pointer->set_type(kLogic);
        }
        // coord.
        inst_pointer->set_center_coord(Point<DBU>(cell.x_coord, cell.y_coord));
        // replenish vector<Pin*> after creating the pin object.

        _instance_pvec.push_back(inst_pointer);
    }

    // initialize net pointer and pin pointer.
    _net_pvec.reserve(net_vec.size());
    for (auto& net : net_vec) {
        Net* net_pointer = new Net(net.name);
        // driver pin.
        auto& driver_pin = pin_vec[net.source];
        add_pin_pointer(driver_pin);
        auto cur_pin_pointer = _pin_pvec[_pin_pvec.size() - 1];
        cur_pin_pointer->set_net(net_pointer);
        // set pin type.
        if (cur_pin_pointer->isNoPinType()) {
            cur_pin_pointer->set_pin_type(kInstanceOutputPin);
        }
        net_pointer->set_driver_pin(cur_pin_pointer);  // add driver pin.
        if (isSequentialPin(driver_pin)) {
            net_pointer->set_type(kClockNet);
        } else {
            net_pointer->set_type(kSignalNet);
        }

        // load pins.
        for (auto sink_idx : net.sinks) {
            auto& sink_pin = pin_vec[sink_idx];
            add_pin_pointer(sink_pin);
            auto cur_pin_pointer = _pin_pvec[_pin_pvec.size() - 1];
            cur_pin_pointer->set_net(net_pointer);
            // set pin type.
            if (cur_pin_pointer->isNoPinType()) {
                cur_pin_pointer->set_pin_type(kInstanceInputPin);
            }
            net_pointer->add_sink_pin(cur_pin_pointer);
        }
        _net_pvec.push_back(net_pointer);
    }

    // replenish instance's pin.
    string pin_name;
    for (size_t i = 0; i < cell_vec.size(); i++) {
        auto ports = cell_vec[i].ports;
        for (auto it = ports.begin(); it != ports.end(); it++) {
            pin_name = pin_vec[(*it).second].name;
            auto map_iter = _name_to_pin_ptr.find(pin_name);
            if (map_iter == _name_to_pin_ptr.end()) {
                _log->warn("Instance: " + cell_vec[i].name + " has no port: " + pin_name + "in design", 1, 0);
            } else {
                _instance_pvec[i]->add_pin(map_iter->second);
            }
        }
    }
}

/**
 * @description: Convert iccad pin to itdp pin.
 * @param {pin} iccad_pin
 * @return {*}
 * @author: ShiJian Chen
 */
void Iccad2015Adapter::add_pin_pointer(const pin iccad_pin) {
    Pin* pin_pointer = new Pin(iccad_pin.name);
    // pin type.
    if (iccad_pin.type == 1) {
        pin_pointer->set_pin_type(kPI);
    } else if (iccad_pin.type == 2) {
        pin_pointer->set_pin_type(kPO);
    } else {
        // distinguish data input pin with data output pin.
        // pin_pointer->set_pin_type(kInstancePin);
        if (iccad_pin.owner > _instance_pvec.size() - 1) {
            _log->error("Current pin: " + iccad_pin.name + " does not belong to any instance.", 1, 0);
        }
        pin_pointer->set_instance(_instance_pvec[iccad_pin.owner]);
        // sequential type.
        if (iccad_pin.isFlopInput) {
            pin_pointer->set_sequential_type(kFlipFlopInput);
        }
        if (iccad_pin.isFlopCkPort) {
            pin_pointer->set_sequential_type(kFlipFlopClk);
        }
    }
    pin_pointer->set_coord(Point<DBU>(iccad_pin.x_coord, iccad_pin.y_coord));

    // TODO : Add LCB input & output.

    pin_pointer->set_early_slack(iccad_pin.earlySlk);
    pin_pointer->set_late_slack(iccad_pin.lateSlk);

    _pin_pvec.push_back(pin_pointer);
    _name_to_pin_ptr[iccad_pin.name] = pin_pointer;
}

/**
 * @description: Judge whether the current pin is related to the clock.
 * @param {pin} iccad_pin
 * @return {*}
 * @author: ShiJian Chen
 */
bool Iccad2015Adapter::isSequentialPin(const pin iccad_pin) {
    if (iccad_pin.name == "iccad_clk") {
        return true;
    }
    // TODO : Identify the output of LCB.

    return false;
}

}  // namespace itdp