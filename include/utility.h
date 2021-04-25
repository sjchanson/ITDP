/**
 * @file utility.h
 * @author SJchan (13560469332@163.com)
 * @brief
 * @version 0.1
 * @date 2021-04-04
 *
 * @copyright Copyright (c) 2021
 *
 */

// computing time

#pragma once

#include <sys/resource.h>
#include <sys/time.h>
#include <time.h>

double microtime() {
    struct timeval tv;
    struct timezone tz;
    gettimeofday(&tv, &tz);
    return tv.tv_sec + tv.tv_usec / 1000000.00;
}

template <typename T>
void deletePointer(T* pointer) {
    delete pointer;
    pointer = nullptr;
}