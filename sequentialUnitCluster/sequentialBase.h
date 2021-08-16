/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-03 16:23:00
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-15 15:47:28
 * @Description:
 */

#ifndef ITDP_SEQUENTIAL_BASE
#define ITDP_SEQUENTIAL_BASE

#include <map>
#include <vector>

#include "clockTree.h"
#include "sequentialElement.h"

namespace itdp {

class SequentialBase {
public:
    SequentialBase() = default;
    ~SequentialBase();

    // getter.
    std::vector<ClockTree*> get_clock_tree() const;
    int get_clock_tree_size() const { return _clock_tree.size(); }
    std::vector<SequentialPI*> get_sequential_pi() const;
    int get_sequential_pi_size() const { return _pi_map.size(); }
    std::vector<SequentialPO*> get_sequential_po() const;
    int get_sequential_po_size() const { return _po_map.size(); }
    std::vector<SequentialFlipFlop*> get_sequential_flipflop() const;
    int get_sequential_flipflop_size() const { return _flipflop_map.size(); }
    std::vector<SequentialBuffer*> get_sequential_buffer() const;
    int get_sequential_buffer_size() const { return _buffer_map.size(); }

    ClockTree* get_clock_tree(std::string name) const;
    SequentialPI* get_sequential_pi(std::string name) const;
    SequentialPO* get_sequential_po(std::string name) const;
    SequentialFlipFlop* get_sequential_flipflop(std::string name) const;
    SequentialBuffer* get_sequential_buffer(std::string name) const;

    // setter.
    void add_pi(SequentialPI* pi) { _pi_map.emplace(pi->get_name(), pi); }
    void add_po(SequentialPO* po) { _po_map.emplace(po->get_name(), po); }
    void add_flipflop(SequentialFlipFlop* flipflop) { _flipflop_map.emplace(flipflop->get_name(), flipflop); }
    void add_buffer(SequentialBuffer* buffer, std::vector<SequentialElement*> slaves);  

    void resetBelonging();

private:
    std::map<std::string, ClockTree*> _clock_tree;
    std::vector<ClockNode*> _tree_nodes;
    std::map<std::string, SequentialPI*> _pi_map;
    std::map<std::string, SequentialPO*> _po_map;
    std::map<std::string, SequentialFlipFlop*> _flipflop_map;
    std::map<std::string, SequentialBuffer*> _buffer_map;
};
}  // namespace itdp

#endif