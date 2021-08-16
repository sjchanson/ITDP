/*
 * @Author: ShiJian Chen
 * @Date: 2021-07-14 14:37:40
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-09 11:57:34
 * @Description:
 */
#include "common/logger.h"

#include <cstdlib>
#include <iostream>

namespace itdp {
using std::cout;
using std::endl;
using std::string;

#define VERBOSE_CHECK()       \
    if (_verbose < verbose) { \
        return;               \
    };

Logger::Logger(string name, int verbose) : _verbose(verbose), _name(name) {}

// Procedure title
void Logger::displayTitle(int verbose) {
    VERBOSE_CHECK()
    cout << "================================================================================" << endl;
    cout << "    ICCAD 2015 Incremental Timing-Driven Placement Contest Improvement Script    " << endl;
    cout << "    Authors : ChenShiJian 13560469332@163.com    " << endl;
    cout << "================================================================================" << endl;
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

void Logger::printDouble(std::string input, double num, int verbose) {
    VERBOSE_CHECK()
    cout << input << " = " << num << endl;
}

void Logger::printString(std::string input, std::string context, int verbose) {
    VERBOSE_CHECK()
    cout << input << " : " << context << endl;
}

void Logger::printItself(std::string context, int verbose) {
    VERBOSE_CHECK()
    cout << context << endl;
}
}  // namespace itdp