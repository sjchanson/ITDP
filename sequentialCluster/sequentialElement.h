

/**
 * @file sequentialElement.h
 * @author SJchan (13560469332@163.com)
 * @brief
 * @version 0.1
 * @date 2021-04-21
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <unordered_map>
#include <unordered_set>

#include "common/parameter.h"
#include "evaluate.h"

class cluster;

class sequentialCluster;

struct coordinate {
    int x;
    int y;
    coordinate(int px, int py);
    bool isLegalCoord();
};

class sequentialBase {
public:
    sequentialBase();
    virtual ~sequentialBase() = default;

    uint get_type() const { return _type; }
    uint get_id() const { return _id; }
    string get_name() const { return _name; }
    double get_avg_skew();
    double get_max_skew();
    coordinate get_coord() const { return _coord; }

    void set_name(string name) { _name = name; }
    void set_id(uint idx) { _id = idx; }
    void set_coord(coordinate coord) { _coord = coord; }
    void add_skew(sequentialBase* base, double skew) { _skews[base] = skew; }
    void resetSkew();

    std::vector<sequentialBase*> get_source();

    double get_skew(sequentialBase* base);

    virtual void update() = 0;

    // override the compare function and hash.
    bool operator<(const sequentialBase& base) { return _name < base.get_name(); }
    bool operator==(const sequentialBase& base) { return _name == base.get_name(); }

    struct basePtrHash {
        size_t operator()(const sequentialBase* base) const { return std::hash<string>()(base->get_name()); }
    };
    struct basePtrEqual {
        bool operator()(sequentialBase* base_1, sequentialBase* base_2) const { return (*base_1) == (*base_2); }
    };

protected:
    uint _type;  // 0->PI , 1->PO , 2->LogicCell 3->FlipFlop , 4->Cluster
    uint _id;    // Corresponding to the position of the real element.
    string _name;
    std::unordered_map<sequentialBase*, double, basePtrHash, basePtrEqual> _skews;
    coordinate _coord;
};

class sequentialPrimaryIO : public sequentialBase {
public:
    sequentialPrimaryIO();
    sequentialPrimaryIO(pin* pin);
    ~sequentialPrimaryIO();

    unsigned isPi() const { return _is_pi; }
    unsigned isPo() const { return _is_po; }

    void update();

    pin* get_pin() const { return _pin; }

private:
    unsigned _is_pi : 1;
    unsigned _is_po : 1;
    pin* _pin;
};

class sequentialFlipFlop : public sequentialBase {
public:
    sequentialFlipFlop();
    sequentialFlipFlop(cell* cell);  // for special case in ICCAD 2015 Contest.
    sequentialFlipFlop(cell* cell, pin* input_pin, parameter* para);

    ~sequentialFlipFlop();

    void update();

    unsigned isFFPi() const { return _is_ff_pi; }
    unsigned isFFPo() const { return _is_ff_po; }
    sequentialCluster* get_cluster() const { return _belong_cluster; }
    cell* get_flipflop() const { return _flipflop; }
    pin* get_input_pin() const { return _input_pin; }

    sequentialFlipFlop* get_max_skew_flipflop_source();

    void set_ff_pi() { _is_ff_pi = 1; }
    void set_ff_po() { _is_ff_po = 1; }
    void set_cluster(sequentialCluster* clus) { _belong_cluster = clus; }

private:
    unsigned _is_ff_pi : 1;
    unsigned _is_ff_po : 1;
    parameter* _para;
    pin* _input_pin;
    cell* _flipflop;
    sequentialCluster* _belong_cluster;
};

class sequentialLogicCell : public sequentialBase {
public:
    sequentialLogicCell();
    sequentialLogicCell(cell* cell);
    ~sequentialLogicCell();

    void update() {}

    cell* get_logic_cell() const { return _cell; }
    vector<sequentialBase*> get_arrival_bases() const { return _arrival_bases; }
    void copy_bases(vector<sequentialBase*>& from);
    void add_arrival_bases(sequentialBase* s) { _arrival_bases.push_back(s); }

private:
    cell* _cell;
    vector<sequentialBase*> _arrival_bases;
};

class sequentialCluster : public sequentialBase {
public:
    sequentialCluster() = default;
    sequentialCluster(std::string name);
    ~sequentialCluster() = default;

    void update();

    void add_flipflop(sequentialFlipFlop* ff);
    std::unordered_set<sequentialFlipFlop*, basePtrHash, basePtrEqual> get_subordinate_flipflops() const {
        return _subordinate_flipflops;
    }

private:
    std::unordered_set<sequentialFlipFlop*, basePtrHash, basePtrEqual> _subordinate_flipflops;
};
