/*
 * @Author: ShiJian Chen
 * @Date: 2021-07-30 14:20:19
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-04 16:04:09
 * @Description:
 */

#ifndef ICCAD_2015_ADAPTER
#define ICCAD_2015_ADAPTER

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "../iccadEstimator/evaluate.h"
#include "common/logger.h"
#include "common/parameter.h"
#include "common/utility.h"
#include "itdpBase/adapterInterface.h"
#include "itdpBase/instance.h"
#include "itdpBase/net.h"
#include "itdpBase/pin.h"

namespace itdp {

class Iccad2015Adapter : public AdapterInterface {
public:
    Iccad2015Adapter() = delete;
    Iccad2015Adapter(std::shared_ptr<circuit> circuit);
    ~Iccad2015Adapter();

    const std::vector<Instance*> get_instance_pvec() const { return _instance_pvec; }
    const std::vector<Net*> get_net_pvec() const { return _net_pvec; }
    const std::vector<Pin*> get_pin_pvec() const { return _pin_pvec; }

    const std::string get_design_name() const { return _design_name; }
    const double get_core_edge_x() const { return _core_edge_x; }
    const double get_core_edge_y() const { return _core_edge_y; }

private:
    Logger* _log;
    std::vector<Instance*> _instance_pvec;
    std::vector<Net*> _net_pvec;
    std::vector<Pin*> _pin_pvec;

    std::map<std::string, Pin*> _name_to_pin_ptr;

    std::string _design_name;
    double _core_edge_x;
    double _core_edge_y;
    // input.

    std::shared_ptr<circuit> _circuit;

    void dataTransmit();
    void add_pin_pointer(const pin iccad_pin);
    bool isSequentialPin(const pin iccad_pin);
};

}  // namespace itdp

#endif