/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-04 13:34:58
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-18 20:50:41
 * @Description:
 */

#ifndef ITDP_CLOCK_TREE
#define ITDP_CLOCK_TREE

#include <vector>

#include "sequentialElement.h"

namespace itdp {

class ClockNode {
public:
    ClockNode() = delete;
    ClockNode(const SequentialElement* instance);
    ~ClockNode() = default;

    // getter.
    const SequentialElement* get_buffer() const { return _instance; }
    const ClockNode* get_master() const { return _master; }
    std::vector<ClockNode*> get_slaves() const { return _slaves; }

    // setter.
    void set_master(ClockNode* master) { _master = master; }
    void add_slave(ClockNode* node) { _slaves.push_back(node); }

private:
    const SequentialElement* _instance;
    ClockNode* _master;
    std::vector<ClockNode*> _slaves;
};

class ClockTree {
public:
    ClockTree() = delete;
    ClockTree(ClockNode* root);
    ~ClockTree() = default;

    // getter.
    ClockNode* get_root() const { return _root; }

private:
    ClockNode* _root;
};
}  // namespace itdp

#endif