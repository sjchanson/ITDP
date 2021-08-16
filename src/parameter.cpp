/*
 * @Author: ShiJian Chen
 * @Date: 2021-07-14 14:37:40
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-12 14:35:03
 * @Description:
 */
#include "common/parameter.h"

namespace itdp {

Parameter::Parameter()
    : _skew_flag(1.0)
    , _side_length(82080)
    , _extra_dist(0.5)
    , _clus_size(25)
    , _plot_interval(1000000)
    , _pre_clus_size(1000)
    , _lamda(1)
    , _benchmark_path("benchmark")
    , _clean_prefix("clean_")
    , _modify_prefix("modify_")
    , _core_x(0)
    , _core_y(0)
    , _max_required_skew(0.0)
    , _row_height(0) {}

}  // namespace itdp
