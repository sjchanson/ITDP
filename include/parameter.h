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
    double skew_flag;  // Set mapping skew value in no setup violation case.
    int side_length;   // Search window's side length.

    parameter();
    void reset() {}
};