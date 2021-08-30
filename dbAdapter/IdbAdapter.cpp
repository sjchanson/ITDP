/**
 * @file IdbAdapter.cpp
 * @author Dawn Li (dawnli619215645@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-08-25
 *
 * @copyright Copyright (c) 2021
 *
 */

// TODO : Wait iDB to help
// Check : Please check the function whether is right

#include "IdbAdapter.h"

#include "IdbEnum.h"

namespace itdp {
IdbAdapter::IdbAdapter(IdbDesign *idb_design) {
    _log = Logger::get_logger_obj("xxx", 0);
    _log->displayTitle(0);
    _idb_design = idb_design;
    _design_name = _idb_design->get_design_name();
    // TODO : wait iDB to write the core interface
    //   _core_edge_x = circuit->get_rx() - circuit->get_lx();
    //   _core_edge_y = circuit->get_ty() - circuit->get_by();
    //   _row_height = static_cast<DBU>(circuit->get_row_height());
    dataTransmit();
}

IdbAdapter::~IdbAdapter() {}

void IdbAdapter::dataTransmit() {
    // build instance
    IdbInstanceList *idb_inst_list = _idb_design->get_instance_list();
    map<IdbInstance *, size_t> idb_instance_index_map;
    size_t index = 0;
    for (auto idb_inst : idb_inst_list->get_instance_list()) {
        Instance *inst_pointer = new Instance(idb_inst->get_name());
        idb_instance_index_map[idb_inst] = index;
        ++index;
        // state.
        idb_inst->is_fixed() ? inst_pointer->set_state(kFixed) : inst_pointer->set_state(kPlaceable);
        // type.
        // Check : iDB instance whether only have FF and logic
        if (idb_inst->get_type() == PCL::iDB::IdbInstanceType::kTiming) {
            inst_pointer->set_type(kFlipFlop);
            // ignore source buffer // clock buffer // only flipflop
        } else {
            inst_pointer->set_type(kLogic);
        }

        // coord.
        inst_pointer->set_center_coord(
            Point<DBU>(idb_inst->get_coordinate()->get_x(), idb_inst->get_coordinate()->get_y()));

        _instance_pvec.push_back(inst_pointer);
    }

    // initialize net pointer and pin pointer.
    IdbNetList *idb_net_list = _idb_design->get_net_list();
    for (auto idb_net : idb_net_list->get_net_list()) {
        Net *net_pointer = new Net(idb_net->get_net_name());
        // driver pin.
        auto driver_pin = idb_net->get_driving_pin();
        add_pin_pointer(driver_pin);
        auto cur_pin_pointer = _pin_pvec.back();
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
        vector<IdbPin *> idb_load_pin_list = idb_net->get_instance_pin_list()->get_pin_list();
        for (auto iter = idb_load_pin_list.begin(); iter != idb_load_pin_list.end(); ++iter) {
            if (*iter == driver_pin) {
                idb_load_pin_list.erase(iter);
                break;
            }
        }
        for (auto idb_pin : idb_load_pin_list) {
            Pin *db_pin = add_pin_pointer(idb_pin);
            IdbInstance *belong_instance = idb_pin->get_instance();
            size_t instance_index = idb_instance_index_map[belong_instance];
            db_pin->set_instance(_instance_pvec[instance_index]);
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
    // Check : add corresponding pins to instance
    vector<IdbInstance *> idb_instance_vec = idb_inst_list->get_instance_list();
    for (size_t i = 0; i < idb_instance_vec.size(); ++i) {
        IdbPins *pin_list = idb_instance_vec[i]->get_pin_list();
        for (auto idb_pin : pin_list->get_pin_list()) {
            if (_name_to_pin_ptr.find(idb_pin->get_pin_name()) != _name_to_pin_ptr.end()) {
                _instance_pvec[i]->add_pin(_name_to_pin_ptr[idb_pin->get_pin_name()]);
            }
        }
    }
}

Pin *IdbAdapter::add_pin_pointer(IdbPin *idb_pin) {
    Pin *pin_pointer = new Pin(idb_pin->get_pin_name());
    // pin type.
    if (idb_pin->is_primary_input()) {
        pin_pointer->set_pin_type(kPI);
    } else if (idb_pin->is_primary_output()) {
        pin_pointer->set_pin_type(kPO);
    } else {
        // distinguish data input pin with data output pin.
        // pin_pointer->set_pin_type(kInstancePin);

        // TODO : wait iDB suport the pin type
        // sequential type.
        // if (idb_pin->isFlopInput()) {
        //     pin_pointer->set_sequential_type(kFlipFlopInput);
        // }
        // if (idb_pin->isFlopCkPort()) {
        //     pin_pointer->set_sequential_type(kFlipFlopClk);
        // }
    }

    pin_pointer->set_coord(Point<DBU>(idb_pin->get_coordinate()->get_x(), idb_pin->get_coordinate()->get_y()));

    // Add LCB input & output.

    // TODO : wait iDB support the pin type
    // pin_pointer->set_early_slack(idb_pin->get_early_slack());
    // pin_pointer->set_late_slack(idb_pin->get_late_slack());

    _pin_pvec.push_back(pin_pointer);
    _name_to_pin_ptr[idb_pin->get_net_name()] = pin_pointer;
    return pin_pointer;
}
bool IdbAdapter::isSequentialPin(IdbPin *idb_pin) {
    if (idb_pin->get_net()->get_connect_type() == PCL::iDB::IdbConnectType::kClock) {
        return true;
    }
    // todo : Identify the output of LCB.

    return false;
}

}  // namespace itdp