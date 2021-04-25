#include "sequentialElement.h"

coordinate::coordinate(int px, int py) {
    x = px;
    y = py;
}

bool coordinate::isLegalCoord() { return !(x == INT_MAX || y == INT_MAX); }

sequentialElement::sequentialElement()
    : _is_pi(0)
    , _is_po(0)
    , _is_ff(0)
    , _name("")
    , _skew(DBL_MAX)
    , _coord(INT_MAX, INT_MAX)
    , _pio_pin(nullptr)
    , _cell(nullptr)
    , _predecessor(nullptr)
    , _belonging_clus(nullptr) {}

sequentialElement::sequentialElement(pin* p_pin) : sequentialElement() {
    if (p_pin->type == 1) {
        _is_pi = 1;
    } else {
        _is_po = 1;
    }
    _skew = 0;
    _pio_pin = p_pin;
    set_name(p_pin->name);
    set_coord(coordinate(p_pin->x_coord, p_pin->y_coord));
};

sequentialElement::sequentialElement(cell* cell) : sequentialElement() {
    _cell = cell;
    set_name(cell->name);
    set_coord(coordinate(cell->x_coord, cell->y_coord));
}

sequentialElement::~sequentialElement() {
    _predecessor = nullptr;
    _belonging_clus = nullptr;
}