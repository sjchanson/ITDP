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
#include <vector>

#include "common/utility.h"

using namespace std;

class ReviseDEF {
public:
    ReviseDEF() = default;
    ~ReviseDEF() = default;

    void addLCBConnection(ifstream& def_in_stream, ofstream& def_out_stream, vector<Point<DBU>*> lcb_coords);
    void cleanLCBConnection(ifstream& def_in_stream, ofstream& def_out_stream, int component_cnt);
};
