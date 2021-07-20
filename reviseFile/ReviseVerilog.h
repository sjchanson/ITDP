/**
 * Author : SJCHANSON
 * Email : 13560469332@163.com
 * brief : modify the ICCAD2015Contest file.
 */

#pragma once

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace std;

class ReviseVerilog {
public:
    ReviseVerilog() = default;
    ~ReviseVerilog() = default;

    void cleanLCBConnection(ifstream& verilog_in_stream, ofstream& verilog_out_stream);
};
