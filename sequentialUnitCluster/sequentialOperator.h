/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-03 11:57:29
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-16 16:41:13
 * @Description:
 */

#ifndef ITDP_SEQUENTIAL_OPERATOR
#define ITDP_SEQUENTIAL_OPERATOR

#include <map>
#include <memory>
#include <set>
#include <stack>
#include <string>
#include <vector>

#include "../include/common/logger.h"
#include "../include/common/parameter.h"
#include "../include/common/utility.h"
#include "../include/itdpBase/adapterInterface.h"
#include "clockTree.h"
#include "sequentialBase.h"
#include "sequentialElement.h"
#include "skewConstraintGraph.h"

namespace itdp {

class ProcessData;

class SequentialOperator {
public:
    SequentialOperator() = delete;
    SequentialOperator(AdapterInterface* data_interface);
    ~SequentialOperator();

    // getter.
    double get_max_required_skew() const { return _max_required_skew; }

    void buildNextLevelCluster(std::map<std::string, Point<DBU>> _coord_map) {}  // TODO : Top-level Operations.
    // std::vector<std::vector<ClusterVertex*>> obtainPerfectBinaryTree();

    void subSequentialClusterSolve();

    void initDistanceMatrix();

    void cleanClusters();

    void sequentialClusterSolve();

private:
    Logger* _log;
    std::shared_ptr<Parameter> _parameter;
    AdapterInterface* _data_interface;
    SequentialBase* _sequential_base;
    SkewConstraintGraph* _skew_constraint_graph;
    std::map<std::string, std::vector<SequentialElement*>> _clusters_map;
    double _max_required_skew;

    void init();

    void add_max_required_skew(double skew);
    void ergodicGenerateGraph(std::stack<SequentialElement*>& stack, std::set<std::string>& accessed_instance,
                              ProcessData* process_data);
    void obtainSinkPins(SequentialElement* element, std::vector<Pin*>& pins);
    SequentialElement* addSequentialElement(Pin* sink_pin, SequentialElement* src_element, ProcessData* process_data);
    double calculateSetupSkew(const Pin* pin);
    void addSequentialGraph(std::vector<SequentialElement*> sink_elements,
                            std::stack<SequentialElement*> duplicate_stack);
    void addVertexes(std::vector<SequentialElement*> sink_elements, SequentialElement* source_element);

    // perfect binary tree.
    // void makeBinaryPair(std::vector<SequentialElement*>& remain_elements,
    //                     std::vector<std::pair<ClusterVertex*, ClusterVertex*>>& binary_pairs);
};

}  // namespace itdp

#endif
