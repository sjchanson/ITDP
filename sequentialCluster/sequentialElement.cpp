#include "sequentialElement.h"

coordinate::coordinate(int px, int py) {
    x = px;
    y = py;
}

bool coordinate::isLegalCoord() { return !(x == INT_MAX || y == INT_MAX); }

sequentialBase::sequentialBase()
    : _type(UINT_MAX), _id(UINT_MAX), _name(""), _skew(DBL_MAX), _coord(INT_MAX, INT_MAX) {}

sequentialPrimaryIO::sequentialPrimaryIO(pin* pin) : _is_pi(0), _is_po(0), _pin(nullptr) {
    _type = 0;
    _name = pin->name;
    _skew = 0.0;
    set_coord(coordinate(pin->x_coord, pin->y_coord));
    if (pin->type == 1) {
        _is_pi = 1;
    } else {
        _is_po = 1;
    }
    _pin = pin;
}

sequentialPrimaryIO::~sequentialPrimaryIO() { _pin = nullptr; }

void sequentialPrimaryIO::update() { set_coord(coordinate(_pin->x_coord, _pin->y_coord)); }

sequentialFlipFlop::sequentialFlipFlop(cell* cell, pin* input_pin, uint skew_flag)
    : _is_ff_pi(0), _is_ff_po(0), _skew_flag(0), _input_pin(nullptr), _flipflop(nullptr), _belong_cluster(nullptr) {
    _type = 1;
    _name = cell->name;
    _skew_flag = skew_flag;
    if (input_pin->lateSlk <= 0) {
        _skew = abs(input_pin->lateSlk) + input_pin->earlySlk;
    } else {
        _skew = _skew_flag;
    }
    set_coord(coordinate(cell->x_coord, cell->y_coord));
    _input_pin = input_pin;
    _flipflop = cell;
}

sequentialFlipFlop::~sequentialFlipFlop() {
    _input_pin = nullptr;
    _flipflop = nullptr;
    _belong_cluster = nullptr;
}

void sequentialFlipFlop::update() {
    if (_input_pin->lateSlk <= 0) {
        _skew = abs(_input_pin->lateSlk) + _input_pin->earlySlk;
    } else {
        _skew = _skew_flag;
    }
    set_coord(coordinate(_flipflop->x_coord, _flipflop->y_coord));
}

sequentialCluster::sequentialCluster(std::string name) {
    _type = 2;
    _name = name;
}

void sequentialCluster::update() {
    std::unordered_set<sequentialFlipFlop*>::iterator iter;
    for (iter = _subordinate_flipflops.begin(); iter != _subordinate_flipflops.end(); iter++) {
        // get center coordate.
        
    }
}

sequentialElement::sequentialElement()
    : _is_pi(0)
    , _is_ff_pi(0)
    , _is_po(0)
    , _is_ff_po(0)
    , _is_ff(0)
    , _name("")
    , _skew(DBL_MAX)
    , _coord(INT_MAX, INT_MAX)
    , _pio_pin(nullptr)
    , _cell(nullptr)
    , _belonging_clus(nullptr) {}

sequentialElement::sequentialElement(pin* p_pin) : sequentialElement() {
    if (p_pin->type == 1) {
        _is_pi = 1;
    } else {
        _is_po = 1;
    }
    _skew = 0.0;
    _pio_pin = p_pin;
    set_name(p_pin->name);
    set_coord(coordinate(p_pin->x_coord, p_pin->y_coord));
};

sequentialElement::sequentialElement(cell* cell) : sequentialElement() {
    _cell = cell;
    set_name(cell->name);
    set_coord(coordinate(cell->x_coord, cell->y_coord));
}

sequentialElement::~sequentialElement() { _belonging_clus = nullptr; }