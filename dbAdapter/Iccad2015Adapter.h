/*
 * @Author: ShiJian Chen
 * @Date: 2021-07-30 14:20:19
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-26 14:36:30
 * @Description:
 */

#ifndef ICCAD_2015_ADAPTER
#define ICCAD_2015_ADAPTER

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "AdapterInterface.h"
#include "Instance.h"
#include "Net.h"
#include "Pin.h"
#include "common/Logger.h"
#include "common/Parameter.h"
#include "common/Utility.h"
#include "evaluate.h"

namespace itdp {

class Iccad2015Adapter : public AdapterInterface {
public:
    Iccad2015Adapter() = delete;
    Iccad2015Adapter(std::shared_ptr<circuit> circuit);
    ~Iccad2015Adapter();

    std::vector<Instance*> get_instance_pvec() const { return _instance_pvec; }
    std::vector<Net*> get_net_pvec() const { return _net_pvec; }
    std::vector<Pin*> get_pin_pvec() const { return _pin_pvec; }

    std::string get_design_name() const { return _design_name; }
    double get_core_edge_x() const { return _core_edge_x; }
    double get_core_edge_y() const { return _core_edge_y; }
    DBU get_row_height() const { return _row_height; }

private:
    Logger* _log;
    std::vector<Instance*> _instance_pvec;
    std::vector<Net*> _net_pvec;
    std::vector<Pin*> _pin_pvec;

    std::map<std::string, Pin*> _name_to_pin_ptr;

    std::string _design_name;
    double _core_edge_x;
    double _core_edge_y;
    DBU _row_height;
    // input.

    std::shared_ptr<circuit> _circuit;

    void dataTransmit();
    void add_pin_pointer(const pin iccad_pin);
    bool isSequentialPin(const pin iccad_pin);
};

}  // namespace itdp

#endif