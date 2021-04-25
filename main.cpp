#include "evaluate.h"
#include "logger.h"
#include "operator/clusterFF.h"
#include "utility.h"

int main(int argc, char** argv) {
    Logger* _log = new Logger("iTDP0.2", 0);
    double begin, end;

    // Print Title
    _log->displayTitle(argc, argv);

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
    _log->printTime("Read File", end - begin);

    // Evaluate Timing Run Time Begin
    begin = microtime();

    _circuit->measure_timing();

    // Evaluate Timing Run Time End
    end = microtime();
    _log->printTime("Evaluate Timing", end - begin);

    // Cluster FlipFlop
    begin = microtime();
    clusterFF* _cluster_ff = new clusterFF(_circuit, _log);
    end = microtime();
    _log->printTime("Init Flipflop topo", end - begin);

    _log->printInt("NO TimingEnding Cell Count", NOTIMING_END_CELL);
    _log->printInt("NO OUTPUT PIN FlIPFLOP", NOOUTPUT_FLIPFLIP);

    return 0;
}