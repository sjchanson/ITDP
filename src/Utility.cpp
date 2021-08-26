/*
 * @Author: ShiJian Chen
 * @Date: 2021-05-16 21:11:14
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-26 14:37:44
 * @Description:
 */
#include "common/Utility.h"

double microtime() {
    struct timeval tv;
    struct timezone tz;
    gettimeofday(&tv, &tz);
    return tv.tv_sec + tv.tv_usec / 1000000.00;
}
