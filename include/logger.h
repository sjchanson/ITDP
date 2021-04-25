/**
 * @file logger.h
 * @author SJchan (13560469332@163.com)
 * @brief
 * @version 0.1
 * @date 2021-04-04
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <string>

#pragma once

class Logger {
public:
    Logger(std::string name, int verbose);

    // Print functions
    void displayTitle(int argc, char** argv, int verbose = 0);

    void programBegin(std::string input, int verbose = 0);
    void programEnd(std::string input, int verbose = 0);

    void warn(std::string input, int code, int verbose = 0);
    void error(std::string input, int code, int verbose = 0);

    void printTime(std::string input, double time, int verbose = 0);

    void printPair(std::string input, int x, int y, int verbose = 0);

    void printInt(std::string input, int num, int verbose = 0);

private:
    int _verbose;
    std::string _name;
};