#include <iostream>

#include "runItdp.h"

using namespace std;

int main(int argc, char** argv) {
    runItdp* itdp = new runItdp(argc, argv);
    itdp->init(argc, argv);
}