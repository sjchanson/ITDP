/*
 * @Author: ShiJian Chen
 * @Date: 2021-07-29 15:54:43
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-26 14:35:46
 * @Description:
 */

#ifndef ITDP_NET
#define ITDP_NET

#include <string>
#include <vector>

#include "Pin.h"
#include "common/Utility.h"

namespace itdp {

enum NET_TYPE { kNoNetType, kClockNet, kSignalNet };

class Net {
public:
    Net() = delete;
    Net(std::string name);
    ~Net() = default;

    // getter.
    std::string get_name() const { return _name; }
    Pin* get_driver_pin() const { return _driver_pin; }
    std::vector<Pin*> get_sink_pins() const { return _sink_pins; }

    bool isClockNet() { return _type == kClockNet; }
    bool isSignalNet() { return _type == kSignalNet; }

    // setter.
    void set_type(NET_TYPE type) { _type = type; }
    void set_driver_pin(Pin* pin) { _driver_pin = pin; }
    void add_sink_pin(Pin* pin) { _sink_pins.push_back(pin); }

private:
    std::string _name;
    NET_TYPE _type;
    Pin* _driver_pin;
    std::vector<Pin*> _sink_pins;
};

inline Net::Net(std::string name) : _name(name), _driver_pin(nullptr) {}

}  // namespace itdp

#endif