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
    _log->printTime("[main] Read File", end - begin, 1);

    // Evaluate Timing Run Time Begin
    begin = microtime();
    _circuit->measure_timing();
    // Evaluate Timing Run Time End
    end = microtime();
    _log->printTime("[main] Evaluate Timing", end - begin, 1);

    // parameter control
    parameter* para = new parameter();

    // Cluster FlipFlop
    begin = microtime();
    sequentialOperator* _cluster_ff = new sequentialOperator(para, _circuit, _log); 
    end = microtime();
    _log->printTime("[main] Init Flipflop Complete Topological Relations", end - begin, 1);

    // Check if the graph has a ring
    // begin = microtime();
    // _cluster_ff->test();
    // end = microtime();
    // _log->printTime("Test timing", end - begin, 1);

    // Plot Init Graph.
    begin = microtime();
    _cluster_ff->plotInitGraph("origin");
    end = microtime();

    // init the sub graph.
    begin = microtime();
    _cluster_ff->initSubGraphs();
    end = microtime();
    _log->printTime("[main] Init The subGraphs", end - begin, 1);

    // cluster the subGraph.
    begin = microtime();
    _cluster_ff->updatePreClusteres();
    end = microtime();
    _log->printTime("[main] Update preClusteres", end - begin, 1);

    // // init the sequential pair.
    // begin = microtime();
    // _cluster_ff->initSequentialPair();
    // end = microtime();
    // _log->printTime("[main] Init SequentianPair", end - begin, 1);

    // // update vertexes fusion
    // begin = microtime();
    // _cluster_ff->updateVertexFusion();
    // end = microtime();
    // _log->printTime("[main] Update Vertexes Fusion", end - begin, 1);

    begin = microtime();
    _cluster_ff->plotCurGraph("Final");
    end = microtime();
    _log->printTime("[main] Plot The Final Graph", end - begin, 1);

    // build the ctsBase.
    begin = microtime();
    ctsBase* base = new ctsBase(_cluster_ff->get_clusters());
    end = microtime();
    _log->printTime("[main] Build The Perfect Binary Tree", end - begin, 1);


    return 0;
}