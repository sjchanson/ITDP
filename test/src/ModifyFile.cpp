/*
 * @Author: ShiJian Chen
 * @Date: 2021-07-22 15:08:49
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-08 22:11:39
 * @Description:
 */
#include <gtest/gtest.h>

#include <iostream>
#include <map>
#include <memory>
#include <string>

#include "/root/iTDP/iccadEstimator/evaluate.h"
#include "/root/iTDP/sequentialUnitCluster/sequentialOperator.h"
#include "common/logger.h"
#include "common/parameter.h"
#include "iccad2015Adapter.h"

// int argc = 3;
// string s1 = "modifyDEF";
// string s2 = "../benchmark/ICCAD15.parm";
// string s3 = "../benchmark/superblue18/superblue18.iccad2015";
// char *argv[] = {(char *)s1.c_str(), (char *)s2.c_str(), (char *)s3.c_str()};

using namespace std;

TEST(DataInterface, constructDB) {
    string s1 = "benchmark/ICCAD15.parm";
    string s2 = "benchmark/superblue18/superblue18.iccad2015";

    auto _circuit = make_shared<circuit>();
    _circuit->read_parameters(s1.c_str());
    _circuit->read_iccad2015_file(s2.c_str());
    _circuit->copy_init_to_final();

    itdp::AdapterInterface* adapter = new itdp::Iccad2015Adapter(_circuit);

    ASSERT_STREQ(_circuit->get_design_name().c_str(), adapter->get_design_name().c_str());
    ASSERT_EQ(_circuit->getCells().size(), adapter->get_instance_pvec().size());
    ASSERT_EQ(_circuit->getNets().size(), adapter->get_net_pvec().size());
    ASSERT_EQ(_circuit->getPins().size(), adapter->get_pin_pvec().size());

    delete adapter;
}

TEST(SequentialGraph, buildGraph) {
    string s1 = "benchmark/ICCAD15.parm";
    string s2 = "benchmark/superblue18/superblue18.iccad2015";

    auto _circuit = make_shared<circuit>();
    _circuit->read_parameters(s1.c_str());
    _circuit->read_iccad2015_file(s2.c_str());
    _circuit->copy_init_to_final();

    itdp::Logger::get_logger_obj("googletest", 1);

    itdp::AdapterInterface* adapter = new itdp::Iccad2015Adapter(_circuit);

    itdp::SequentialOperator* opt = new itdp::SequentialOperator(adapter);

    opt->buildPerfectBinaryTree();

    delete opt;
    delete adapter;
}
