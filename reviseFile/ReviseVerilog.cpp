#include "ReviseVerilog.h"

#include <string>
#include <vector>

#include "evaluate.h"

void ReviseVerilog::addLCBConnection(ifstream& verilog_in_stream, ofstream& verilog_out_stream, int lcb_count,
                                     std::vector<sequentialFlipFlop*> flipflops, std::vector<string> pin_name_vec) {
    string line;
    bool flag = false;

    // add wire.
    while (getline(verilog_in_stream, line)) {
        if (flag) {
            for (int i = 0; i < lcb_count; i++) {
                line = "wire lcb_fo" + std::to_string(i) + ";";
                verilog_out_stream << line << std::endl;
            }
            flag = false;
            break;
        }

        if (line == "// Start wires") {
            flag = true;
        }
        verilog_out_stream << line << std::endl;
    }

    // add lcb cells.

    while (getline(verilog_in_stream, line)) {
        if (flag) {
            for (int i = 0; i < lcb_count; i++) {
                line = "INV_Z80 lcb" + std::to_string(i) + " ( ";
                line += ".o(lcb_fo" + std::to_string(i) + "), ";
                line += ".a(iccad_clk) );";
                verilog_out_stream << line << std::endl;
            }

            flag = false;
            break;
        }

        if (line == "// Start cells") {
            flag = true;
        }
        verilog_out_stream << line << std::endl;
    }

    // add flipflop cells.
    string q_port;
    string d_port;
    string ck_port;

    for (int i = 0; i < flipflops.size(); i++) {
        q_port = "";
        d_port = "";
        ck_port = "";
        string lcb_connection = flipflops[i]->get_cluster()->get_name();

        // get q_port and d_port name.
        cell* origin_cell = flipflops[i]->get_flipflop();
        for (auto pin_pair : origin_cell->ports) {
            string port_name = pin_pair.first;
            if (port_name == "q") {
                q_port = pin_name_vec[pin_pair.second];
            } else if (port_name == "d") {
                d_port = pin_name_vec[pin_pair.second];
            } else if (port_name == "ck") {
                ck_port = lcb_connection;
            } else {
                std::cout << "ERROR In FlipFlop Itself." << std::endl;
            }
        }

        line = "DFF_X80 " + origin_cell->name + " ( ";
        if (!q_port.empty()) {
            line += ".q(" + q_port + "), ";
        }
        if (!d_port.empty()) {
            line += ".d(" + d_port + "), ";
        }
        if (!ck_port.empty()) {
            line += ".ck(" + ck_port + ") );";
        }
        verilog_out_stream << line << std::endl;
    }

    // delete all DFF_X80 later.
    string sub_string;
    while (getline(verilog_in_stream, line)) {
        stringstream input(line);
        string token = "";
        vector<string> tokens;

        while (input >> token) {
            tokens.push_back(token);
        }

        if (tokens.size() > 4) {
            if (tokens[0] == "DFF_X80") {
                continue;
            }
        }
        verilog_out_stream << line << std::endl;
    }
}

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