/*
 * @Author: ShiJian Chen
 * @Date: 2021-07-29 14:57:42
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-05 15:15:29
 * @Description:
 */

#ifndef ITDP_INSTANCE
#define ITDP_INSTANCE

#include <string>
#include <vector>

#include "../common/utility.h"
#include "pin.h"

namespace itdp {

enum InstanceType { kNoInstanceType, kLogic, kFlipFlop, kLCB };
enum InstanceState { kNoInstanceState, kFixed, kPlaceable };

class Instance {
public:
    Instance() = delete;
    Instance(std::string name);
    Instance(const Instance&) = delete;
    ~Instance() = default;

    // getter.
    std::string get_name() const { return _name; }
    InstanceState get_state() const { return _state; }
    // Point<DBU> get_lower_point() const { return _lower_point; }
    // Point<DBU> get_upper_point() const { return _upper_point; }
    Point<DBU> get_center_point() const { return _center_point; }
    std::vector<Pin*> get_pin_vec() const { return _pins; }
    std::vector<Pin*> get_outpin_vec() const;

    bool isLogic() { return _type == kLogic; }
    bool isFlipflop() { return _type == kFlipFlop; }
    bool isLCB() { return _type == kLCB; }

    bool isFixed() { return _state == kFixed; }
    bool isPlaceable() { return _state == kPlaceable; }

    // setter.
    void set_state(InstanceState state) { _state = state; }
    void set_type(InstanceType type) { _type = type; }
    void set_center_coord(Point<DBU> center_point) { _center_point = center_point; }
    void add_pin(Pin* pin) { _pins.push_back(pin); }

private:
    std::string _name;
    InstanceState _state;
    InstanceType _type;
    // Point<DBU> _lower_point;
    // Point<DBU> _upper_point;
    Point<DBU> _center_point;
    std::vector<Pin*> _pins;
};

inline Instance::Instance(std::string name) : _name(name) {}

inline std::vector<Pin*> Instance::get_outpin_vec() const {
    std::vector<Pin*> out_pins;
    for (auto pin : _pins) {
        if (pin->isInstanceOutputPin()) {
            out_pins.push_back(pin);
        }
    }
    return out_pins;
}

}  // namespace itdp

#endif