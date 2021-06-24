#include "evaluate.h"
#include "include/logger.h"
#include "include/utility.h"
#include "sequentialCluster/ctsBase.h"
#include "sequentialCluster/sequentialOperator.h"

int main(int argc, char** argv) {
    Logger* _log = new Logger("iTDP0.2", 1);
    double begin, end;

    // Print Title
    _log->displayTitle(argc, argv, 1);

    // Read File Run Time Begin
    begin = microtime();

    // Construct the placer data from .parm .v .def .lib .sdc
    circuit* _circuit = new circuit();
    _circuit->read_parameters(argv[1]);
    _circuit->read_iccad2015_file(argv[2]);
    if (argc == 4) {
        _circuit->read_ops(argv[3]);
    }
    _circuit->copy_init_to_final();

    // Read File Run Time End
    end = microtime();
    _log->printTime("Read File", end - begin, 1);

    // Evaluate Timing Run Time Begin
    begin = microtime();

    _circuit->measure_timing();

    // Evaluate Timing Run Time End
    end = microtime();
    _log->printTime("Evaluate Timing", end - begin, 1);

    // parameter control
    parameter* para = new parameter();

    // Cluster FlipFlop
    begin = microtime();
    sequentialOperator* _cluster_ff = new sequentialOperator(para, _circuit, _log);
    end = microtime();
    _log->printTime("Init Flipflop topo", end - begin, 1);

    // Check if the graph has a ring
    // begin = microtime();
    // _cluster_ff->test();
    // end = microtime();
    // _log->printTime("Test timing", end - begin, 1);

    // Plot SequentialElement
    begin = microtime();
    _cluster_ff->plot();
    end = microtime();
    _log->printTime("Plot", end - begin, 1);

    // init the sequential pair.
    _cluster_ff->initSequentialPair();

    // update vertexes fusion
    _cluster_ff->updateVertexFusion();

    // build the ctsBase.

    return 0;
}