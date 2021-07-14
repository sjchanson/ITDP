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
    void displayTitle(int argc, char** argv, int verbose);

    void programBegin(std::string input, int verbose);
    void programEnd(std::string input, int verbose);

    void warn(std::string input, int code, int verbose);
    void error(std::string input, int code, int verbose);

    void printTime(std::string input, double time, int verbose);

    void printPair(std::string input, int x, int y, int verbose);

    void printInt(std::string input, int num, int verbose);
    void printDouble(std::string input, double num, int verbose);
    void printString(std::string input, std::string context, int verbose);

    void printItself(std::string context, int verbose);

private:
    int _verbose;
    std::string _name;
};