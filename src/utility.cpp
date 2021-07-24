#include "common/utility.h"

double microtime() {
    struct timeval tv;
    struct timezone tz;
    gettimeofday(&tv, &tz);
    return tv.tv_sec + tv.tv_usec / 1000000.00;
}
