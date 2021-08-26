/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-01 20:11:31
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-26 14:41:14
 * @Description:
 */

#ifndef ITDP_SEQUENTIAL_ELEMENT
#define ITDP_SEQUENTIAL_ELEMENT

#include <float.h>

#include <map>
#include <string>
#include <vector>

#include "AdapterInterface.h"
#include "common/Utility.h"

namespace itdp {

class SequentialCluster;

class SequentialElement {
public:
    SequentialElement() = default;
    virtual ~SequentialElement() = default;

    // getter.
    std::string get_name() const { return _name; }
    const Point<DBU> get_coord() const { return _coordinate; }

    double get_max_skew() const;
    double get_avg_skew() const;
    double get_skew(std::string name) const;

    bool isPi() const { return _type == 1; }
    bool isPo() const { return _type == 2; }
    bool isFlipFlop() const { return _type == 3; }
    bool isBuffer() const { return _type == 4; }
    bool isCluster() const { return _type == 5; }
    bool isLogic() const { return _type == 6; }

    // setter.
    void set_name(std::string name) { _name = name; }
    void add_name_to_skew(std::string name, double skew) { _name_to_skew.emplace(name, skew); }

protected:
    std::string _name;
    int _type;  // 0->Not exist, 1->Pi, 2->Po, 3->FlipFlop, 4->Lcb, 5->Cluster, 6->Logic.
    std::map<std::string, double> _name_to_skew;
    Point<DBU> _coordinate;
};
inline double SequentialElement::get_skew(std::string name) const {
    auto iter = _name_to_skew.find(name);
    if (iter == _name_to_skew.end()) {
        return DBL_MAX;
    } else {
        return (*iter).second;
    }
}

struct SequentialElementPair {
    SequentialElementPair(const SequentialElement* e1, const SequentialElement* e2);
    const SequentialElement* element_1;
    const SequentialElement* element_2;
    double skew;
};
inline SequentialElementPair::SequentialElementPair(const SequentialElement* e1, const SequentialElement* e2) {
    element_1 = e1;
    element_2 = e2;
}

class SequentialPI : public SequentialElement {
public:
    SequentialPI() = delete;
    SequentialPI(Pin* pin);
    ~SequentialPI() = default;

    // getter.
    const Pin* get_pin() const { return _pin; }

private:
    Pin* _pin;
};
inline SequentialPI::SequentialPI(Pin* pin) {
    _name = pin->get_name();
    _type = 1;
    _coordinate = pin->get_center_coord();
    _pin = pin;
}

class SequentialPO : public SequentialElement {
public:
    SequentialPO() = delete;
    SequentialPO(Pin* pin);
    ~SequentialPO() = default;

    // getter.
    const Pin* get_pin() const { return _pin; }

private:
    Pin* _pin;
};
inline SequentialPO::SequentialPO(Pin* pin) {
    _name = pin->get_name();
    _type = 2;
    _coordinate = pin->get_center_coord();
    _pin = pin;
}

class SequentialFlipFlop : public SequentialElement {
public:
    SequentialFlipFlop() = delete;
    SequentialFlipFlop(Instance* fliflop_inst);
    ~SequentialFlipFlop() = default;

    // getter.
    const Instance* get_instance() const { return _flipflop_inst; }
    const Pin* get_data_input_pin() const { return _data_input_pin; }

    // be careful of get_cluster.

    // setter.
    void set_cluster(SequentialElement* cluster) { _cluster = cluster; }

    void resetCluster() { _cluster = nullptr; }

private:
    Pin* _data_input_pin;
    Pin* _clock_input_pin;
    Instance* _flipflop_inst;
    SequentialElement* _cluster;
};
inline SequentialFlipFlop::SequentialFlipFlop(Instance* flipflop_inst) : _cluster(nullptr) {
    _name = flipflop_inst->get_name();
    _type = 3;
    _coordinate = flipflop_inst->get_center_point();
    _flipflop_inst = flipflop_inst;
    for (auto pin : flipflop_inst->get_pin_vec()) {
        if (pin->isFlipFlopInput()) {
            _data_input_pin = pin;
        } else if (pin->isFlipFlopClk()) {
            _clock_input_pin = pin;
        }
    }
}

class SequentialCluster : public SequentialElement {
public:
    SequentialCluster() = delete;
    SequentialCluster(std::string name);
    ~SequentialCluster() = default;

    // getter.
    int get_sub_size() const { return _sub_element_map.size(); }
    const SequentialElement* get_sub_element(std::string name) const;
    std::vector<const SequentialElement*> get_all_element_vec() const;

    // setter.
    void addSubElement(const SequentialElement* element);
    void set_name(std::string name) { _name = name; }

private:
    std::map<std::string, const SequentialElement*> _sub_element_map;
};
inline SequentialCluster::SequentialCluster(std::string name) {
    _name = name;
    _type = 5;
}
inline const SequentialElement* SequentialCluster::get_sub_element(std::string name) const {
    auto iter = _sub_element_map.find(name);
    if (iter == _sub_element_map.end()) {
        return nullptr;
    } else {
        return (*iter).second;
    }
}
inline std::vector<const SequentialElement*> SequentialCluster::get_all_element_vec() const {
    std::vector<const SequentialElement*> element_vec;
    for (auto pair : _sub_element_map) {
        element_vec.push_back(pair.second);
    }
    return element_vec;
}
inline void SequentialCluster::addSubElement(const SequentialElement* element) {
    int x_sum = get_coord().get_x() * _sub_element_map.size();
    int y_sum = get_coord().get_y() * _sub_element_map.size();
    _sub_element_map.emplace(element->get_name(), element);

    int x_coord = (x_sum + element->get_coord().get_x()) / _sub_element_map.size();
    int y_coord = (y_sum + element->get_coord().get_y()) / _sub_element_map.size();
    _coordinate = Point<DBU>(x_coord, y_coord);
}

class SequentialBuffer : public SequentialElement {
public:
    SequentialBuffer() = delete;
    SequentialBuffer(std::string name);
    ~SequentialBuffer() = default;

    // setter.
    void set_cluster(SequentialElement* cluster) { _cluster = cluster; }
    void set_center_coord(Point<DBU> coord) { _coordinate = coord; }

    void resetCluster() { _cluster = nullptr; }

private:
    // Pin* _clock_input_pin;
    // Instance* _buffer_inst;
    SequentialElement* _cluster;
};
inline SequentialBuffer::SequentialBuffer(std::string name) {
    _name = name;
    _type = 4;
}

class SequentialLogic : public SequentialElement {
public:
    SequentialLogic() = delete;
    SequentialLogic(Instance* logic);
    ~SequentialLogic() = default;

    // getter.
    const Instance* get_logic_cell() const { return _logic_cell; }
    std::vector<SequentialElement*> get_accessible_elements() const { return _accessible_elements; }

    // setter.
    void add_accessible_element(SequentialElement* e) { _accessible_elements.push_back(e); };
    void add_batch_elements(std::vector<SequentialElement*> vec);

private:
    Instance* _logic_cell;
    std::vector<SequentialElement*> _accessible_elements;
};
inline SequentialLogic::SequentialLogic(Instance* logic) {
    _name = logic->get_name();
    _type = 6;
    // no need the coord.
    _logic_cell = logic;
}
inline void SequentialLogic::add_batch_elements(std::vector<SequentialElement*> vec) {
    _accessible_elements.insert(_accessible_elements.end(), vec.begin(), vec.end());
}

}  // namespace itdp

#endif