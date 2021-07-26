#include "common/parameter.h"

parameter::parameter()
    : skew_flag(1.0)
    , side_length(82080)
    , extra_dist(0.5)
    , clus_num(55000)
    , clus_size(25)
    , plot_interval(1000000)
    , pre_clus_size(1000)
    , lamda(1)
    , benchmark_path("benchmark")
    , clean_prefix("clean_")
    , modify_prefix("modify_")
    , _core_x(0)
    , _core_y(0)
    , _max_required_skew(0.0) {}
