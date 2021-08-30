/*
 * @Author: ShiJian Chen
 * @Date: 2021-07-22 15:08:49
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-26 17:08:14
 * @Description:
 */
#include <gtest/gtest.h>

#include <iostream>
#include <map>
#include <memory>
#include <string>

#include "AdapterInterface.h"
#include "CtsBase.h"
#include "DME.h"
#include "Iccad2015Adapter.h"
#include "SequentialOperator.h"
#include "common/Logger.h"
#include "common/Parameter.h"
#include "evaluate.h"

// int argc = 3;
// string s1 = "modifyDEF";
// string s2 = "../benchmark/ICCAD15.parm";
// string s3 = "../benchmark/superblue18/superblue18.iccad2015";
// char *argv[] = {(char *)s1.c_str(), (char *)s2.c_str(), (char *)s3.c_str()};

using namespace std;

TEST(DataInterface, constructDB) {
    string s1 = "benchmark/ICCAD15.parm";
    string s2 = "benchmark/simple/simple.iccad2015";

    auto _circuit = make_shared<circuit>();
    _circuit->read_parameters(s1.c_str());
    _circuit->read_iccad2015_file(s2.c_str());
    _circuit->copy_init_to_final();
    _circuit->measure_timing();

    itdp::AdapterInterface* adapter = new itdp::Iccad2015Adapter(_circuit);

    ASSERT_STREQ(_circuit->get_design_name().c_str(), adapter->get_design_name().c_str());
    ASSERT_EQ(_circuit->getCells().size(), adapter->get_instance_pvec().size());
    ASSERT_EQ(_circuit->getNets().size(), adapter->get_net_pvec().size());
    ASSERT_EQ(_circuit->getPins().size(), adapter->get_pin_pvec().size());

    delete adapter;
}

TEST(SequentialGraph, buildGraph) {
    string s1 = "benchmark/ICCAD15.parm";
    string s2 = "benchmark/simple/simple.iccad2015";

    auto _circuit = make_shared<circuit>();
    _circuit->read_parameters(s1.c_str());
    _circuit->read_iccad2015_file(s2.c_str());
    _circuit->copy_init_to_final();
    _circuit->measure_timing();

    itdp::Logger::get_logger_obj("googletest", 1);
    itdp::AdapterInterface* adapter = new itdp::Iccad2015Adapter(_circuit);

    itdp::SequentialOperator* opt = new itdp::SequentialOperator(adapter);

    delete opt;
    delete adapter;
}

TEST(SequentialGraph, subGraphPartition) {
    string s1 = "benchmark/ICCAD15.parm";
    string s2 = "benchmark/simple/simple.iccad2015";

    auto _circuit = make_shared<circuit>();
    _circuit->read_parameters(s1.c_str());
    _circuit->read_iccad2015_file(s2.c_str());
    _circuit->copy_init_to_final();
    _circuit->measure_timing();

    itdp::Logger::get_logger_obj("googletest", 1);
    itdp::AdapterInterface* adapter = new itdp::Iccad2015Adapter(_circuit);

    itdp::SequentialOperator* opt = new itdp::SequentialOperator(adapter);

    opt->sloveInitLevelCluster();

    delete opt;
    delete adapter;
}

TEST(SequentialGraph, initDistance) {
    string s1 = "benchmark/ICCAD15.parm";
    string s2 = "benchmark/simple/simple.iccad2015";

    auto _circuit = make_shared<circuit>();
    _circuit->read_parameters(s1.c_str());
    _circuit->read_iccad2015_file(s2.c_str());
    _circuit->copy_init_to_final();
    _circuit->measure_timing();

    itdp::Logger::get_logger_obj("googletest", 1);
    itdp::AdapterInterface* adapter = new itdp::Iccad2015Adapter(_circuit);

    itdp::SequentialOperator* opt = new itdp::SequentialOperator(adapter);
    // opt->initDistanceMatrix();

    delete opt;
    delete adapter;
}

TEST(SequentialGraph, vertexFusion) {
    string s1 = "benchmark/ICCAD15.parm";
    string s2 = "benchmark/simple/simple.iccad2015";

    auto _circuit = make_shared<circuit>();
    _circuit->read_parameters(s1.c_str());
    _circuit->read_iccad2015_file(s2.c_str());
    _circuit->copy_init_to_final();
    _circuit->measure_timing();

    itdp::Logger::get_logger_obj("googletest", 1);
    itdp::AdapterInterface* adapter = new itdp::Iccad2015Adapter(_circuit);

    itdp::SequentialOperator* opt = new itdp::SequentialOperator(adapter);

    // opt->initDistanceMatrix();
    // opt->sequentialClusterSolve();

    delete opt;
    delete adapter;
}

TEST(PerfectBinaryTree, constructPerfectBinaryTree) {
    string s1 = "benchmark/ICCAD15.parm";
    string s2 = "benchmark/superblue18/superblue18.iccad2015";

    auto _circuit = make_shared<circuit>();
    _circuit->read_parameters(s1.c_str());
    _circuit->read_iccad2015_file(s2.c_str());
    _circuit->copy_init_to_final();
    _circuit->measure_timing();

    itdp::Logger::get_logger_obj("googletest", 1);
    itdp::AdapterInterface* adapter = new itdp::Iccad2015Adapter(_circuit);

    itdp::SequentialOperator* opt = new itdp::SequentialOperator(adapter);

    opt->sloveInitLevelCluster();

    // DME test.
    while (opt->get_clusters_map().size() > 1) {
        CtsBase* base = new CtsBase(opt->get_clusters_map());
        auto clusters = base->get_binary_clusters();

        std::map<std::string, Point<DBU>> buffer_coords;
        for (auto clus : clusters) {
            auto binary_tree = clus->get_perfect_binary_tree();
            Point<DBU> buffer_location = DME().computeTpClusterCenterLocation(binary_tree, clus->get_name(), 4);
            buffer_coords.emplace(clus->get_name(), buffer_location);
        }

        opt->buildNextLevelCluster(buffer_coords);
        delete base;
    }

    delete opt;
    delete adapter;
}