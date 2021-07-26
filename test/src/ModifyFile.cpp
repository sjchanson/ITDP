#include <gtest/gtest.h>

#include <iostream>
#include <map>
#include <string>

#include "runItdp.h"

// int argc = 3;
// string s1 = "modifyDEF";
// string s2 = "../benchmark/ICCAD15.parm";
// string s3 = "../benchmark/superblue18/superblue18.iccad2015";
// char *argv[] = {(char *)s1.c_str(), (char *)s2.c_str(), (char *)s3.c_str()};

using namespace std;

TEST(IOTest, clean_lcb) {
    int argc = 3;
    string s1 = "modifyDEF";
    string s2 = "benchmark/ICCAD15.parm";
    string s3 = "benchmark/simple/simple.iccad2015";
    char *argv[] = {(char *)s1.c_str(), (char *)s2.c_str(), (char *)s3.c_str()};

    runItdp().read_clean_lcb_file(argc, argv);
}

TEST(IOTest, rewrite_lcb) { map<string, Point<DBU> *> lcbs; }