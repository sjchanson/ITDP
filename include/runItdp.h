
#pragma once

#include <map>
#include <vector>

#include "../reviseFile/ReviseDEF.h"
#include "../reviseFile/ReviseVerilog.h"
#include "../sequentialCluster/ctsBase.h"
#include "../sequentialCluster/sequentialOperator.h"
#include "common/logger.h"
#include "common/utility.h"
#include "evaluate.h"

namespace itdp {
using namespace std;

class runItdp {
public:
    runItdp();
    runItdp(int argc, char** argv);
    ~runItdp();

    std::vector<ctsSingleClus*> get_cts_clusters() const { return _base->get_sub_clusters(); }

    void read_clean_lcb_file(int argc, char** argv);
    void read_file(int argc, char** argv);

    void measure_timing_only(int argc, char** argv);

    void write_lcb(map<string, Point<DBU>*> lcbs);

    void init(int argc, char** argv);

private:
    int _argc;
    char** _argv;
    Logger* _log;
    circuit* _circuit;
    Parameter* _para;
    sequentialOperator* _opt;
    ctsBase* _base;
    std::vector<string> _pin_name_vec;

    void cleanLCBInDEF(string path, string prefix, int component_cnt);
    void cleanLCBInVerilog(string path, string prefix);

    void addLCBInDEF(string path, string prefix, std::vector<Point<DBU>*> lcb_coords);
    void addLCBInVerilog(string path, string prefix, int lcb_cnt, std::vector<sequentialFlipFlop*> flipflops,
                         std::vector<string> pin_name_vec);

    void completePinName();
};
}  // namespace itdp