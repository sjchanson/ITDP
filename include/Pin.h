/*
 * @Author: ShiJian Chen
 * @Date: 2021-07-29 16:51:20
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-26 14:35:55
 * @Description:
 */

#ifndef ITDP_PIN
#define ITDP_PIN

#include <string>

#include "common/Utility.h"

// Internal association type.
namespace itdp {
class Instance;
class Net;
};  // namespace itdp

namespace itdp {

enum PIN_TYPE { kNoPinType, kPI, kPO, kInstanceInputPin, kInstanceOutputPin };
enum SEQUENTIAL_TYPE { kNoSequentialType, kFlipFlopInput, kFlipFlopClk, kLCBInput, kLCBOutput };

class Pin {
public:
    Pin() = delete;
    Pin(std::string name);
    Pin(const Pin&) = delete;
    ~Pin() = default;

    // getter.
    std::string get_name() const { return _name; }
    Net* get_net() const { return _net; }
    Instance* get_instance() const { return _instance; }
    Point<DBU> get_center_coord() const { return _center_coord; }
    double get_early_slack() const { return _early_slack; }
    double get_late_slack() const { return _late_slack; }

    bool isNoPinType() { return _pin_type == kNoPinType; }
    bool isPI() { return _pin_type == kPI; }
    bool isPO() { return _pin_type == kPO; }
    bool isInstanceInputPin() { return _pin_type == kInstanceInputPin; }
    bool isInstanceOutputPin() { return _pin_type == kInstanceOutputPin; }

    bool isFlipFlopInput() { return _sequential_type == kFlipFlopInput; }
    bool isFlipFlopClk() { return _sequential_type == kFlipFlopClk; }
    bool isLCBInput() { return _sequential_type == kLCBInput; }
    bool isLCBOutput() { return _sequential_type == kLCBOutput; }

    // setter.
    void set_net(Net* net) { _net = net; }
    void set_instance(Instance* instance) { _instance = instance; }
    void set_coord(Point<DBU> coord) { _center_coord = coord; }
    void set_pin_type(PIN_TYPE pin_type) { _pin_type = pin_type; }
    void set_sequential_type(SEQUENTIAL_TYPE seq_type) { _sequential_type = seq_type; }
    void set_early_slack(double slk) { _early_slack = slk; }
    void set_late_slack(double slk) { _late_slack = slk; }

private:
    std::string _name;
    Net* _net;
    Instance* _instance;
    Point<DBU> _center_coord;

    // type
    PIN_TYPE _pin_type;
    SEQUENTIAL_TYPE _sequential_type;

    // sequential infomation.
    double _early_slack;
    double _late_slack;
};

inline Pin::Pin(std::string name)
    : _name(name)
    , _net(nullptr)
    , _instance(nullptr)
    , _pin_type(kNoPinType)
    , _sequential_type(kNoSequentialType)
    , _early_slack(0.0)
    , _late_slack(0.0) {}

}  // namespace itdp

#endif