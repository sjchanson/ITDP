

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

struct coordinate {
    int x;
    int y;
    coordinate(int px, int py);
    bool isLegalCoord();
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