

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

#include <unordered_set>

#include "../evaluate.h"

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

    uint get_id() const { return _id; }
    string get_name() const { return _name; }
    double get_skew() const { return _skew; }
    coordinate get_coord() const { return _coord; }

    void set_id(uint idx) { _id = idx; }
    void set_coord(coordinate coord) { _coord = coord; }
    void set_skew(double skew) { _skew = skew; }

    virtual void update() = 0;

protected:
    uint _type;  // 0->PI/PO,1->FlipFlop,2->Cluster.
    uint _id;
    string _name;
    double _skew;
    coordinate _coord;
};

class sequentialPrimaryIO : public sequentialBase {
public:
    sequentialPrimaryIO() = default;
    sequentialPrimaryIO(pin* pin);
    ~sequentialPrimaryIO();

    unsigned isPi() const { return _is_pi; }
    unsigned isPo() const { return _is_po; }

    void update();

private:
    unsigned _is_pi : 1;
    unsigned _is_po : 1;
    pin* _pin;
};

class sequentialFlipFlop : public sequentialBase {
public:
    sequentialFlipFlop() = default;
    sequentialFlipFlop(cell* cell, pin* input_pin, uint skew_flag);
    ~sequentialFlipFlop();

    void update();

    unsigned isFFPi() const { return _is_ff_pi; }
    unsigned isFFPo() const { return _is_ff_po; }
    sequentialCluster* get_cluster() const { return _belong_cluster; }

    void set_ff_pi() { _is_ff_pi = 1; }
    void set_ff_po() { _is_ff_po = 1; }
    void set_cluster(sequentialCluster* clus) { _belong_cluster = clus; }

private:
    unsigned _is_ff_pi : 1;
    unsigned _is_ff_po : 1;
    uint _skew_flag;
    pin* _input_pin;
    cell* _flipflop;
    sequentialCluster* _belong_cluster;
};

class sequentialCluster : public sequentialBase {
public:
    sequentialCluster() = default;
    sequentialCluster(std::string name);
    ~sequentialCluster() = default;

    void update();

    void add_flipflop(sequentialFlipFlop* ff) { _subordinate_flipflops.insert(ff); }
    std::unordered_set<sequentialFlipFlop*> get_subordinate_flipflops() const { return _subordinate_flipflops; }

private:
    std::unordered_set<sequentialFlipFlop*> _subordinate_flipflops;
};

class sequentialElement {
public:
    sequentialElement();
    sequentialElement(pin* p_pin);  // Only for PI/PO
    sequentialElement(cell* cell);
    ~sequentialElement();

    string get_name() const { return _name; }
    double get_skew() const { return _skew; }
    coordinate get_coord() const { return _coord; }
    std::unordered_set<sequentialElement*> get_predecessors() const { return _predecessors; }
    cluster* get_clus() const { return _belonging_clus; }

    void set_name(string name) { _name = name; }
    void set_ff_pi() { _is_ff_pi = 1; }
    void set_ff_po() { _is_ff_po = 1; }
    void set_ff() { _is_ff = 1; }
    void set_coord(coordinate coord) { _coord = coord; }
    void set_skew(double skew) { _skew = skew; }
    void add_predecessor(sequentialElement* ff) { _predecessors.insert(ff); }
    void set_clus(cluster* clus) { _belonging_clus = clus; }

    pin* get_pio_pin() const { return _pio_pin; }
    cell* get_cell() const { return _cell; }

    unsigned isPi() const { return _is_pi; }
    unsigned isFFPi() const { return _is_ff_pi; }
    unsigned isFFPo() const { return _is_ff_po; }
    unsigned isPo() const { return _is_po; }
    unsigned isFlipFlop() const { return _is_ff; }

private:
    unsigned _is_pi : 1;
    unsigned _is_ff_pi : 1;
    unsigned _is_po : 1;
    unsigned _is_ff_po : 1;
    unsigned _is_ff : 1;
    string _name;
    double _skew;
    coordinate _coord;

    pin* _pio_pin;
    cell* _cell;
    std::unordered_set<sequentialElement*> _predecessors;
    cluster* _belonging_clus;
};

class cluster {
public:
    cluster();
    ~cluster();

private:
    coordinate _center_coord;
    vector<sequentialElement*> _subordinate_flipflops;
};