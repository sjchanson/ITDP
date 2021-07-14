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

#pragma once

class parameter {
public:
    double skew_flag;   // Set mapping skew value in no setup violation case.
    int side_length;    // Search window's side length.
    double extra_dist;  // for the extra_dist
    int clus_num;
    int clus_size;
    int plot_interval;
    int pre_clus_size;
    int lamda;

    parameter();
    void reset() {}

    void set_core_x(int x) { _core_x = x; }
    void set_core_y(int y) { _core_y = y; }
    void set_max_required_skew(double skew) { _max_required_skew = skew; }

    int get_core_x() const { return _core_x; }
    int get_core_y() const { return _core_y; }
    double get_max_required_skew() const { return _max_required_skew; }

private:
    int _core_x;
    int _core_y;
    double _max_required_skew;
};