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

class ReviseDEF {
public:
    ReviseDEF() = default;
    ~ReviseDEF() = default;

    void cleanLCBConnection(ifstream& def_in_stream, ofstream& def_out_stream, int component_cnt);
};
