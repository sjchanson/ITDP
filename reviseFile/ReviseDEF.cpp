#include "ReviseDEF.h"

#include <string>
#include <vector>

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
