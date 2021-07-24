#include "runItdp.h"

#include <iomanip>
#include <iostream>
#include <sstream>

runItdp::runItdp() : _log(nullptr), _circuit(nullptr), _para(nullptr), _opt(nullptr), _base(nullptr) {}

runItdp::runItdp(int argc, char** argv) : runItdp() { init(argc, argv); }

runItdp::~runItdp() {
    delete _log;
    delete _circuit;
    delete _para;
    delete _opt;
    delete _base;
}

void runItdp::read_clean_lcb_file(int argc, char** argv) {
    read_file(argc, argv);

    int cell_cnt = 0;
    int lcb_cnt = 0;
    for (auto& cell : _circuit->getCells()) {
        if (cell.isLCB) {
            lcb_cnt++;
        }
        cell_cnt++;
    }
    cleanLCBInDEF(_para->benchmark_path, _para->clean_prefix, cell_cnt - lcb_cnt);
    cleanLCBInVerilog(_para->benchmark_path, _para->clean_prefix);

    // read the file after cleaning.
    string iccad_file = _para->benchmark_path + "/" + _circuit->get_design_name() + "/" + "clean_superblue18.iccad2015";
    argv[2] = (char*)iccad_file.c_str();
    init(argc, argv);

    // memory manage.
    delete _circuit;
    _circuit = nullptr;
    delete _para;
    _para = nullptr;
}

void runItdp::read_file(int argc, char** argv) {
    _circuit = new circuit();
    _circuit->read_parameters(argv[1]);
    _circuit->read_iccad2015_file(argv[2]);
    _circuit->copy_init_to_final();
    _para = new parameter();
}

void runItdp::measure_timing_only(int argc, char** argv) {
    read_file(argc, argv);
    _circuit->measure_timing();

    // memory manage.
    delete _circuit;
    _circuit = nullptr;
    delete _para;
    _para = nullptr;
}

void runItdp::write_lcb(map<string, Point<DBU>*> lcbs) {
    std::vector<Point<DBU>*> lcb_coords;
    int lcb_cnt = lcbs.size();
    string lcb_name;
    int setting_lcb_id = 0;

    auto clusters = _opt->get_clusters();

    for (auto& pair : lcbs) {
        lcb_name = pair.first;
        // find the corresponding cluster.
        sequentialCluster tmp(lcb_name);
        auto cluster = clusters.find(&tmp);
        if (cluster == clusters.end()) {
            _log->error("ERROR In Finding Cluster", 1, 1);
        } else {
            (*cluster)->set_name("lcb_fo" + std::to_string(setting_lcb_id++));
        }

        // add to the vector.
        lcb_coords.push_back(pair.second);
    }

    addLCBInDEF(_para->benchmark_path, _para->modify_prefix, lcb_coords);
    addLCBInVerilog(_para->benchmark_path, _para->modify_prefix, lcb_cnt, _opt->get_sequential_flipflop_vec(),
                    _pin_name_vec);
}

void runItdp::init(int argc, char** argv) {
    _log = new Logger("iTDP0.2", 1);
    double begin, end;

    // Print Title
    _log->displayTitle(argc, argv, 1);

    // Read File Run Time Begin
    begin = microtime();

    // Construct the placer data from .parm .v .def .lib .sdc
    _circuit = new circuit();
    _circuit->read_parameters(argv[1]);
    _circuit->read_iccad2015_file(argv[2]);
    if (argc == 4) {
        _circuit->read_ops(argv[3]);
    }
    _circuit->copy_init_to_final();

    // Read File Run Time End
    end = microtime();
    _log->printTime("[main] Read File", end - begin, 1);

    // Evaluate Timing Run Time Begin
    begin = microtime();
    _circuit->measure_timing();
    // Evaluate Timing Run Time End
    end = microtime();
    _log->printTime("[main] Evaluate Timing", end - begin, 1);

    // parameter control
    _para = new parameter();

    // Cluster FlipFlop
    begin = microtime();
    _opt = new sequentialOperator(_para, _circuit, _log);
    end = microtime();
    _log->printTime("[main] Init Flipflop Complete Topological Relations", end - begin, 1);

    // test
    _opt->cleanLCBInDEF();
    _opt->cleanLCBInVerilog();

    // Check if the graph has a ring
    // begin = microtime();
    // _opt->test();
    // end = microtime();
    // _log->printTime("Test timing", end - begin, 1);

    // Plot Init Graph.
    begin = microtime();
    _opt->plotInitGraph("origin");
    end = microtime();
    _log->printTime("[main] Init The subGraphs", end - begin, 1);

    // init the sub graph.
    begin = microtime();
    _opt->initSubGraphs();
    end = microtime();
    _log->printTime("[main] Init The subGraphs", end - begin, 1);

    // cluster the subGraph.
    begin = microtime();
    _opt->updatePreClusteres();
    end = microtime();
    _log->printTime("[main] Update preClusteres", end - begin, 1);

    // // init the sequential pair.
    // begin = microtime();
    // _opt->initSequentialPair();
    // end = microtime();
    // _log->printTime("[main] Init SequentianPair", end - begin, 1);

    // // update vertexes fusion
    // begin = microtime();
    // _opt->updateVertexFusion();
    // end = microtime();
    // _log->printTime("[main] Update Vertexes Fusion", end - begin, 1);

    begin = microtime();
    _opt->plotCurGraph("Final");
    end = microtime();
    _log->printTime("[main] Plot The Final Graph", end - begin, 1);

    // build the ctsBase.
    begin = microtime();
    _base = new ctsBase(_opt->get_clusters());
    end = microtime();
    _log->printTime("[main] Build The Perfect Binary Tree", end - begin, 1);
}

void runItdp::cleanLCBInDEF(string path, string prefix, int component_cnt) {
    string design_name = _circuit->get_design_name();
    ifstream in(path + "/" + design_name + "/" + design_name + ".def");
    if (!in.good()) {
        _log->error("Cannot open file to read", 1, 1);
    }
    ofstream out(path + "/" + design_name + "/" + prefix + design_name + ".def");
    if (!out.good()) {
        _log->error("Cannot open flie to write", 1, 1);
    }
    ReviseDEF().cleanLCBConnection(in, out, component_cnt);
    in.close();
    out.close();
}

void runItdp::cleanLCBInVerilog(string path, string prefix) {
    string design_name = _circuit->get_design_name();
    ifstream in(path + "/" + design_name + "/" + design_name + ".v");
    if (!in.good()) {
        _log->error("Cannot open file to read", 1, 1);
    }
    ofstream out(path + "/" + design_name + "/" + prefix + design_name + ".v");
    if (!out.good()) {
        _log->error("Cannot open flie to write", 1, 1);
    }
    ReviseVerilog().cleanLCBConnection(in, out);
    in.close();
    out.close();
}

void runItdp::addLCBInDEF(string path, string prefix, std::vector<Point<DBU>*> lcb_coords) {
    string design_name = _circuit->get_design_name();
    ifstream in(path + "/" + design_name + "/" + design_name + ".def");
    if (!in.good()) {
        _log->error("Cannot open file to read", 1, 1);
    }
    ofstream out(path + "/" + design_name + "/" + prefix + design_name + ".def");
    if (!out.good()) {
        _log->error("Cannot open file to write", 1, 1);
    }
    ReviseDEF().addLCBConnection(in, out, lcb_coords);
    in.close();
    out.close();
}

void runItdp::addLCBInVerilog(string path, string prefix, int lcb_cnt, std::vector<sequentialFlipFlop*> flipflops,
                              std::vector<string> pin_name_vec) {
    string design_name = _circuit->get_design_name();
    ifstream in(path + "/" + design_name + "/" + design_name + ".v");
    if (!in.good()) {
        _log->error("Cannot open file to read", 1, 1);
    }
    ofstream out(path + "/" + design_name + "/" + prefix + design_name + ".def");
    if (!out.good()) {
        _log->error("Cannot open file to write", 1, 1);
    }
    ReviseVerilog().addLCBConnection(in, out, lcb_cnt, flipflops, pin_name_vec);
    in.close();
    out.close();
}

void runItdp::completePinName() {
    auto pin_vec = _circuit->getPins();

    for (auto pin : pin_vec) {
        string name = pin.name;
        name.replace(0, name.rfind("_") + 1, "");
        _pin_name_vec.push_back(name);
    }
}