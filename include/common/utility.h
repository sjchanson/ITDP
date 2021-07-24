/**
 * @file utility.h
 * @author SJchan (13560469332@163.com)
 * @brief
 * @version 0.1
 * @date 2021-04-04
 *
 * @copyright Copyright (c) 2021
 *
 */

// computing time

#pragma once

#include <sys/resource.h>
#include <sys/time.h>
#include <time.h>

#include <iostream>

double microtime();

typedef long long DBU;

// fun-2
template <class T>

class Point {
public:
    Point(T x, T y) : _x(x), _y(y) {}

    T getX() const { return _x; }
    T getY() const { return _y; }

    T computeDist(const Point<T> &other) const {
        T dx = (getX() > other.getX()) ? (getX() - other.getX()) : (other.getX() - getX());
        T dy = (getY() > other.getY()) ? (getY() - other.getY()) : (other.getY() - getY());

        return dx + dy;
    }

    T computeDistX(const Point<T> &other) const {
        T dx = (getX() > other.getX()) ? (getX() - other.getX()) : (other.getX() - getX());
        return dx;
    }

    T computeDistY(const Point<T> &other) const {
        T dy = (getY() > other.getY()) ? (getY() - other.getY()) : (other.getY() - getY());
        return dy;
    }

    bool operator<(const Point<T> &other) const {
        if (getX() != other.getX()) {
            return getX() < other.getX();
        } else {
            return getY() < other.getY();
        }
    }

    friend std::ostream &operator<<(std::ostream &out, const Point<T> &point) {
        out << "[(" << point.getX() << ", " << point.getY() << ")]";
        return out;
    }

    bool isUnLegal() const { return _x == -1 && _y == -1; }

private:
    T _x;
    T _y;
};