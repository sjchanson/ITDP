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

#include "../sequentialCluster/sequentialElement.h"
#include "common/utility.h"

using namespace std;

class ReviseVerilog {
public:
    ReviseVerilog() = default;
    ~ReviseVerilog() = default;

    void addLCBConnection(ifstream& verilog_in_stream, ofstream& verilog_out_stream, int lcb_count,
                          std::vector<sequentialFlipFlop*> flipflops, std::vector<string> pin_name_vec);
    void cleanLCBConnection(ifstream& verilog_in_stream, ofstream& verilog_out_stream);
};
