#include "include/logger.h"

#include <cstdlib>
#include <iostream>

using std::cout;
using std::endl;
using std::string;

#define VERBOSE_CHECK()       \
    if (_verbose < verbose) { \
        return;               \
    };

Logger::Logger(string name, int verbose) : _name(name), _verbose(verbose) {}

// Procedure title
void Logger::displayTitle(int argc, char** argv, int verbose) {
    VERBOSE_CHECK()
    cout << "================================================================================" << endl;
    cout << "    ICCAD 2015 Incremental Timing-Driven Placement Contest Improvement Script    " << endl;
    cout << "    Authors : ChenShiJian 13560469332@163.com    " << endl;
    cout << "================================================================================" << endl;

    if (argc < 3 || argc > 4) {
        cout << "Incorrect arguments. exiting .." << endl;
        cout << "Usage : iTDPlacer ICCAD15.parm [.iccad2015] (optional)[final.ops]" << endl;
    }

    cout << "Command line : " << endl;
    for (int i = 0; i < argc; i++) cout << " " << argv[i];
    cout << endl;
}

void Logger::error(std::string input, int code, int verbose) {
    VERBOSE_CHECK()
    cout << "ERROR : " << input << ". code : " << code << endl;
    exit(1);
}

void Logger::warn(std::string input, int code, int verbose) {
    VERBOSE_CHECK()
    cout << "WARN : " << input << ". code : " << code << endl;
}

void Logger::printTime(std::string input, double time, int verbose) {
    VERBOSE_CHECK()
    cout << input << " : "
         << "It took " << time << " seconds." << endl;
}

void Logger::printPair(std::string input, int x, int y, int verbose) {
    VERBOSE_CHECK()
    cout << input << "'s coordinate : "
         << "(" << x << "," << y << ")" << endl;
}

void Logger::printInt(std::string input, int num, int verbose) {
    VERBOSE_CHECK()
    cout << input << " = " << num << endl;
}