#include "ReviseVerilog.h"

#include <string>
#include <vector>

void ReviseVerilog::cleanLCBConnection(ifstream& verilog_in_stream, ofstream& verilog_out_stream) {
    string line;
    string sub_string;

    while (getline(verilog_in_stream, line)) {
        stringstream input(line);
        string token = "";
        vector<string> tokens;

        while (input >> token) {
            tokens.push_back(token);
        }

        // delete lcb wire.
        if (tokens.size() == 2) {
            sub_string = tokens[1].substr(0, 3);
            if (sub_string == "lcb") {
                continue;
            }
        }

        // modify flipflop clk port.
        if (tokens.size() > 2) {
            if (tokens[0] == "DFF_X80") {
                line.clear();
                for (int i = 0; i < tokens.size(); i++) {
                    sub_string = tokens[i].substr(0, 3);
                    if (sub_string == ".ck") {
                        tokens[i] = ".ck(iccad_clk)";
                    }
                    line += tokens[i] + " ";
                }
            }
        }

        // delete the lcb cell.
        if (tokens.size() == 6) {
            sub_string = tokens[1].substr(0, 3);
            if (sub_string == "lcb") {
                continue;
            }
        }

        verilog_out_stream << line << endl;
    }
}