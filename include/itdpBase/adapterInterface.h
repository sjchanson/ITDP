/*
 * @Author: ShiJian Chen
 * @Date: 2021-07-31 16:27:26
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-12 12:10:06
 * @Description:
 */

#ifndef ITDP_ADAPTER
#define ITDP_ADAPTER

#include <string>
#include <vector>

#include "instance.h"
#include "net.h"
#include "pin.h"

namespace itdp {

class AdapterInterface {
public:
    AdapterInterface() = default;
    AdapterInterface(const AdapterInterface&) = delete;
    AdapterInterface(AdapterInterface&&) = delete;
    virtual ~AdapterInterface() {}

    virtual std::vector<Net*> get_net_pvec() const = 0;
    virtual std::vector<Instance*> get_instance_pvec() const = 0;
    virtual std::vector<Pin*> get_pin_pvec() const = 0;
    virtual std::string get_design_name() const = 0;
    virtual double get_core_edge_x() const = 0;
    virtual double get_core_edge_y() const = 0;
    virtual long long get_row_height() const = 0;
};

}  // namespace itdp

#endif