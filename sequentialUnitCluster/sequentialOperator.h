/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-03 11:57:29
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-08 20:19:11
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

    void buildNextLevelCluster(std::map<std::string, Point<DBU>> _coord_map) {}  // TODO : Top-level Operations.
    void buildPerfectBinaryTree() {}                                             // TODO

    void cleanClusters();

private:
    Logger* _log;
    std::shared_ptr<Parameter> _parameter;
    AdapterInterface* _data_interface;
    SequentialBase* _sequential_base;
    SkewConstraintGraph* _skew_constraint_graph;
    std::map<std::string, std::vector<SequentialElement*>> clusters_map;

    void init();
    void ergodicGenerateGraph(std::stack<SequentialElement*>& stack, std::set<std::string>& accessed_instance,
                              ProcessData* process_data);
    void obtainSinkPins(SequentialElement* element, std::vector<Pin*>& pins);
    SequentialElement* addSequentialElement(Pin* sink_pin, SequentialElement* src_element,
                                            std::stack<SequentialElement*>& stack,
                                            std::set<std::string>& accessed_instance, ProcessData* process_data);
    double calculateSetupSkew(const Pin* pin);
    void addSequentialGraph(std::vector<SequentialElement*> sink_elements,
                            std::stack<SequentialElement*> duplicate_stack);
    void addVertexes(std::vector<SequentialElement*> sink_elements, SequentialElement* source_element);
};

class ProcessData {
public:
    ProcessData() = default;
    ~ProcessData();

    // getter.
    int get_logic_size() const { return _logic_map.size(); }
    bool isInstanceAccessed(std::string name);
    bool isFlipflopAccessed(std::string name);
    SequentialLogic* get_sequential_logic(std::string name) const;

    // setter.
    void add_accessed_instance(std::string name) { _instance_visited_record.emplace(name); }
    void add_accessed_flipflop(std::string name) { _flipflop_visited_record.emplace(name); }
    void add_logic_map(SequentialLogic* logic) { _logic_map.emplace(logic->get_name(), logic); }

private:
    std::set<std::string> _instance_visited_record;
    std::set<std::string> _flipflop_visited_record;
    std::map<std::string, SequentialLogic*> _logic_map;
};
inline ProcessData::~ProcessData() {
    _instance_visited_record.clear();
    _flipflop_visited_record.clear();
    for (auto logic : _logic_map) {
        delete logic.second;
    }
    _logic_map.clear();
}
inline bool ProcessData::isInstanceAccessed(std::string name) {
    return _instance_visited_record.find(name) != _instance_visited_record.end();
}
inline bool ProcessData::isFlipflopAccessed(std::string name) {
    return _flipflop_visited_record.find(name) != _flipflop_visited_record.end();
}
inline SequentialLogic* ProcessData::get_sequential_logic(std::string name) const {
    auto iter = _logic_map.find(name);
    if (iter == _logic_map.end()) {
        return nullptr;
    } else {
        return (*iter).second;
    }
}

}  // namespace itdp

#endif
