/*
 * @Author: ShiJian Chen
 * @Date: 2021-07-14 14:37:40
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-09 11:57:52
 * @Description:
 */
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

#ifndef ITDP_LOGGER
#define ITDP_LOGGER

#include <string>
namespace itdp {

class Logger {
public:
    static Logger* get_logger_obj(std::string name, int verbose) {
        static Logger _log(name, verbose);
        return &_log;
    }

    // Print functions
    void displayTitle(int verbose);

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

    // private:
    //     static Logger _log;

private:
    Logger(std::string name, int verbose);
    ~Logger() {}
    int _verbose;
    std::string _name;
};

}  // namespace itdp
#endif