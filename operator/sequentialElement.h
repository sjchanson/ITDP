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

#include "../evaluate.h"

class cluster;

enum FLIPFLOP_MODE { FAST, SLOW, NEUTRAL };

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
    FLIPFLOP_MODE get_mode() const { return _mode; }
    sequentialElement* get_predecessor() const { return _predecessor; }
    cluster* get_clus() const { return _belonging_clus; }

    void set_name(string name) { _name = name; }
    void set_ff() { _is_ff = 1; }
    void set_visited() { _is_visited = 1; }
    void set_coord(coordinate coord) { _coord = coord; }
    void set_mode(FLIPFLOP_MODE mode) { _mode = mode; }
    void set_skew(double skew) { _skew = skew; }
    void set_predecessor(sequentialElement* ff) { _predecessor = ff; }
    void set_clus(cluster* clus) { _belonging_clus = clus; }

    pin* get_pio_pin() const { return _pio_pin; }
    cell* get_cell() const { return _cell; }

    unsigned isPi() const { return _is_pi; }
    unsigned isPo() const { return _is_po; }
    unsigned isFlipFlop() const { return _is_ff; }
    unsigned isVisited() const { return _is_visited; }

private:
    unsigned _is_pi : 1;
    unsigned _is_po : 1;
    unsigned _is_ff : 1;
    unsigned _is_visited : 1;
    string _name;
    double _skew;
    coordinate _coord;

    FLIPFLOP_MODE _mode;
    pin* _pio_pin;
    cell* _cell;
    sequentialElement* _predecessor;
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