/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-03 11:57:29
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-23 10:56:36
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

    std::map<std::string, std::vector<const SequentialElement*>> get_clusters_map() const { return _clusters_map; }
    void buildNextLevelCluster(std::map<std::string, Point<DBU>> _coord_map) {}  // TODO : Top-level Operations.
    void sloveInitLevelCluster();

private:
    Logger* _log;
    std::shared_ptr<Parameter> _parameter;
    AdapterInterface* _data_interface;
    SequentialBase* _sequential_base;
    SkewConstraintGraph* _skew_constraint_graph;
    std::map<std::string, std::vector<const SequentialElement*>> _clusters_map;
    double _max_required_skew;

    void initDistanceMatrix();
    void cleanClusters();
    void sequentialClusterSolve();

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
};

}  // namespace itdp

#endif
