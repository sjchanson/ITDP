#include "ReviseDEF.h"

#include <string>
#include <vector>

void ReviseDEF::addLCBConnection(ifstream& def_in_stream, ofstream& def_out_stream, vector<Point<DBU>*> lcb_coords) {
    int lcb_cnt = lcb_coords.size();
    string line;
    bool flag = true;

    while (getline(def_in_stream, line)) {
        if (!flag) {
            for (int i = 0; i < lcb_cnt; i++) {
                line = "- lcb" + std::to_string(i) + " INV_Z80";
                def_out_stream << line << std::endl;
                line = "   + PLACED ( " + std::to_string(lcb_coords[i]->getX()) + " " +
                       std::to_string(lcb_coords[i]->getY()) + " ) N ;";
                def_out_stream << line << std::endl;
            }
            break;
        }

        stringstream input(line);
        string token = "";
        vector<string> tokens;

        while (input >> token) {
            tokens.push_back(token);
        }

        if (tokens.size() == 3 && tokens[0] == "COMPONENTS") {
            int components_cnt = std::stoi(tokens[1]) + lcb_cnt;
            line = tokens[0] + " " + std::to_string(components_cnt) + " " + tokens[2];
            flag = false;
        }
        def_out_stream << line << std::endl;
    }

    // copy the remain file.
    while (getline(def_in_stream, line)) {
        def_out_stream << line << std::endl;
    }
}

void ReviseDEF::cleanLCBConnection(ifstream& def_in_stream, ofstream& def_out_stream, int component_cnt) {
    string line;
    bool flag = true;
    string sub_string;

    while (getline(def_in_stream, line)) {
        if (!flag) {
            flag = true;
            continue;
        }
        stringstream input(line);
        string token = "";
        vector<string> tokens;

        while (input >> token) {
            tokens.push_back(token);
        }

        if (tokens.size() > 1) {
            if (tokens[0] == "COMPONENTS") {
                line = tokens[0] + " " + std::to_string(component_cnt) + " ;";
            }

            // delete lcb
            sub_string = tokens[1].substr(0, 3);
            if (sub_string == "lcb") {
                flag = false;
                continue;
            }
        }
        def_out_stream << line << std::endl;
    }
}
