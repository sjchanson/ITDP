#include "sequentialElement.h"

coordinate::coordinate(int px, int py) {
    x = px;
    y = py;
}

bool coordinate::isLegalCoord() { return !(x == INT_MAX || y == INT_MAX); }

sequentialBase::sequentialBase()
    : _type(UINT_MAX), _id(UINT_MAX), _name(""), _skew(DBL_MAX), _coord(INT_MAX, INT_MAX) {}

sequentialPrimaryIO::sequentialPrimaryIO() : sequentialBase(), _is_pi(0), _is_po(0), _pin(nullptr) {}

sequentialPrimaryIO::sequentialPrimaryIO(pin* pin) : sequentialPrimaryIO() {
    _name = pin->name;
    _skew = 0.0;
    set_coord(coordinate(pin->x_coord, pin->y_coord));
    if (pin->type == 1) {
        _type = 0;
        _is_pi = 1;
    } else {
        _type = 1;
        _is_po = 1;
    }
    _pin = pin;
}

sequentialPrimaryIO::~sequentialPrimaryIO() { _pin = nullptr; }

void sequentialPrimaryIO::update() { set_coord(coordinate(_pin->x_coord, _pin->y_coord)); }

sequentialLogicCell::sequentialLogicCell() : sequentialBase(), _cell(nullptr) {}

sequentialLogicCell::sequentialLogicCell(cell* cell) {
    _type = 2;
    _name = cell->name;
    set_coord(coordinate(cell->x_coord, cell->y_coord));
    _cell = cell;
}

sequentialLogicCell::~sequentialLogicCell() { _cell = nullptr; }

sequentialFlipFlop::sequentialFlipFlop()
    : sequentialBase()
    , _is_ff_pi(0)
    , _is_ff_po(0)
    , _para(nullptr)
    , _input_pin(nullptr)
    , _flipflop(nullptr)
    , _belong_cluster(nullptr) {}

sequentialFlipFlop::sequentialFlipFlop(cell* cell) : sequentialFlipFlop() {
    _type = 3;
    _name = cell->name;
    set_coord(coordinate(cell->x_coord, cell->y_coord));
    _flipflop = cell;
}

sequentialFlipFlop::sequentialFlipFlop(cell* cell, pin* input_pin, parameter* para) : sequentialFlipFlop() {
    _type = 3;
    _name = cell->name;
    _para = para;

    set_coord(coordinate(cell->x_coord, cell->y_coord));
    _input_pin = input_pin;
    _flipflop = cell;
}

sequentialFlipFlop::~sequentialFlipFlop() {
    _para = nullptr;
    _input_pin = nullptr;
    _flipflop = nullptr;
    _belong_cluster = nullptr;
}

void sequentialFlipFlop::update() {
    if (_input_pin->lateSlk <= 0) {
        _skew = abs(_input_pin->lateSlk) + _input_pin->earlySlk;
    } else {
        _skew = _para->skew_flag;
    }
    set_coord(coordinate(_flipflop->x_coord, _flipflop->y_coord));
}

sequentialCluster::sequentialCluster(std::string name) {
    _type = 4;
    _name = name;
    _skew = 0;
}

void sequentialCluster::add_flipflop(sequentialFlipFlop* ff) {
    int x_sum = _coord.x * _subordinate_flipflops.size();  // before add the filpflop.
    int y_sum = _coord.y * _subordinate_flipflops.size();
    _subordinate_flipflops.insert(ff);

    int x_coord = (x_sum + ff->get_coord().x) / _subordinate_flipflops.size();  // after add the flipflop.
    int y_coord = (y_sum + ff->get_coord().y) / _subordinate_flipflops.size();
    set_coord(coordinate(x_coord, y_coord));
}

void sequentialCluster::update() {
    int x = 0;
    int y = 0;
    std::unordered_set<sequentialFlipFlop*>::iterator iter;
    for (iter = _subordinate_flipflops.begin(); iter != _subordinate_flipflops.end(); iter++) {
        // get center coordate.
        x += (*iter)->get_coord().x;
        y += (*iter)->get_coord().y;
    }
    set_coord(coordinate(x / _subordinate_flipflops.size(), y / _subordinate_flipflops.size()));
}
