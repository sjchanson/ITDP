/*
 * @Author: ShiJian Chen
 * @Date: 2021-07-14 14:37:40
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-07 14:32:24
 * @Description:
 */
/**
 * @file parameter.h
 * @author SJchan (13560469332@163.com)
 * @brief
 * @version 0.1
 * @date 2021-05-30
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef ITDP_PARAMETER_H
#define ITDP_PARAMETER_H

#include <memory>
#include <string>

namespace itdp {
class Parameter {
public:
    static std::shared_ptr<Parameter> get_parameter_pointer();

    void set_core_x(int x) { _core_x = x; }
    void set_core_y(int y) { _core_y = y; }
    void set_max_required_skew(double skew) { _max_required_skew = skew; }

    bool isCoreXLegal() const { return _core_x != 0; }
    bool isCoreYLegal() const { return _core_y != 0; }
    bool isMaxSkewLegal() const { return _max_required_skew != 0.0; }

    double get_skew_flag() const { return _skew_flag; }
    int get_side_length() const { return _side_length; }
    double get_extra_dist() const { return _extra_dist; }
    int get_clus_size() const { return _clus_size; }
    int get_plot_interval() const { return _plot_interval; }
    int get_pre_clus_size() const { return _pre_clus_size; }
    int get_lamda() const { return _lamda; }
    std::string get_benchmark_path() const { return _benchmark_path; }
    std::string get_clean_prefix() const { return _clean_prefix; }
    std::string get_modify_prefix() const { return _modify_prefix; }
    int get_core_x() const { return _core_x; }
    int get_core_y() const { return _core_y; }
    double get_max_required_skew() const { return _max_required_skew; }

private:
    static std::shared_ptr<Parameter> _para;
    static void destroyParameter(Parameter*) {}

private:
    Parameter();
    ~Parameter();

    double _skew_flag;   // Set mapping skew value in no setup violation case.
    int _side_length;    // Search window's side length.
    double _extra_dist;  // for the extra_dist
    int _clus_size;
    int _plot_interval;
    int _pre_clus_size;
    int _lamda;
    std::string _benchmark_path;
    std::string _clean_prefix;
    std::string _modify_prefix;
    int _core_x;
    int _core_y;
    double _max_required_skew;
};

inline std::shared_ptr<Parameter> Parameter::_para(new Parameter(), Parameter::destroyParameter);

inline std::shared_ptr<Parameter> Parameter::get_parameter_pointer() { return _para; }

}  // namespace itdp

#endif