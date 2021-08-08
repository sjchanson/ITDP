/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-04 19:51:11
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-08 19:48:02
 * @Description:
 */

#include "sequentialBase.h"

namespace itdp {

SequentialBase::~SequentialBase() {
    for (auto clock_tree : _clock_tree) {
        delete clock_tree.second;
    }
    for (auto clock_node : _tree_nodes) {
        delete clock_node;
    }
    for (auto pi : _pi_map) {
        delete pi.second;
    }
    for (auto po : _po_map) {
        delete po.second;
    }
    for (auto flipflop : _flipflop_map) {
        delete flipflop.second;
    }
    for (auto buffer : _buffer_map) {
        delete buffer.second;
    }
}

std::vector<ClockTree*> SequentialBase::get_clock_tree() const {
    std::vector<ClockTree*> clock_tree_vec;
    for (auto pair : _clock_tree) {
        clock_tree_vec.push_back(pair.second);
    }
    return clock_tree_vec;
}

std::vector<SequentialPI*> SequentialBase::get_sequential_pi() const {
    std::vector<SequentialPI*> sequential_pi_vec;
    for (auto pair : _pi_map) {
        sequential_pi_vec.push_back(pair.second);
    }
    return sequential_pi_vec;
}

std::vector<SequentialPO*> SequentialBase::get_sequential_po() const {
    std::vector<SequentialPO*> sequential_po_vec;
    for (auto pair : _po_map) {
        sequential_po_vec.push_back(pair.second);
    }
    return sequential_po_vec;
}

std::vector<SequentialFlipFlop*> SequentialBase::get_sequential_flipflop() const {
    std::vector<SequentialFlipFlop*> sequential_flipflop_vec;
    for (auto pair : _flipflop_map) {
        sequential_flipflop_vec.push_back(pair.second);
    }
    return sequential_flipflop_vec;
}

std::vector<SequentialBuffer*> SequentialBase::get_sequential_buffer() const {
    std::vector<SequentialBuffer*> sequential_buffer_vec;
    for (auto pair : _buffer_map) {
        sequential_buffer_vec.push_back(pair.second);
    }
    return sequential_buffer_vec;
}

ClockTree* SequentialBase::get_clock_tree(std::string name) const {
    auto iter = _clock_tree.find(name);
    if (iter == _clock_tree.end()) {
        return nullptr;
    } else {
        return (*iter).second;
    }
}

SequentialPI* SequentialBase::get_sequential_pi(std::string name) const {
    auto iter = _pi_map.find(name);
    if (iter == _pi_map.end()) {
        return nullptr;
    } else {
        return (*iter).second;
    }
}

SequentialPO* SequentialBase::get_sequential_po(std::string name) const {
    auto iter = _po_map.find(name);
    if (iter == _po_map.end()) {
        return nullptr;
    } else {
        return (*iter).second;
    }
}

SequentialFlipFlop* SequentialBase::get_sequential_flipflop(std::string name) const {
    auto iter = _flipflop_map.find(name);
    if (iter == _flipflop_map.end()) {
        return nullptr;
    } else {
        return (*iter).second;
    }
}

SequentialBuffer* SequentialBase::get_sequential_buffer(std::string name) const {
    auto iter = _buffer_map.find(name);
    if (iter == _buffer_map.end()) {
        return nullptr;
    } else {
        return (*iter).second;
    }
}

}  // namespace itdp