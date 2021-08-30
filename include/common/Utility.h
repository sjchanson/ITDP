/*
 * @Author: ShiJian Chen
 * @Date: 2021-05-16 21:11:14
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-05 14:25:25
 * @Description:
 */
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



namespace itdp {
double microtime();

typedef long long DBU;

// fun-2
template <class T>
class Point {
public:
    Point() : _x(-1), _y(-1) {}
    Point(T x, T y) : _x(x), _y(y) {}

    T get_x() const { return _x; }
    T get_y() const { return _y; }

    T computeDist(const Point<T> &other) const {
        T dx = (get_x() > other.get_x()) ? (get_x() - other.get_x()) : (other.get_x() - get_x());
        T dy = (get_y() > other.get_y()) ? (get_y() - other.get_y()) : (other.get_y() - get_y());

        return dx + dy;
    }

    T computeDistX(const Point<T> &other) const {
        T dx = (get_x() > other.get_x()) ? (get_x() - other.get_x()) : (other.get_x() - get_x());
        return dx;
    }

    T computeDistY(const Point<T> &other) const {
        T dy = (get_y() > other.get_y()) ? (get_y() - other.get_y()) : (other.get_y() - get_y());
        return dy;
    }

    bool operator<(const Point<T> &other) const {
        if (get_x() != other.get_x()) {
            return get_x() < other.get_x();
        } else {
            return get_y() < other.get_y();
        }
    }

    friend std::ostream &operator<<(std::ostream &out, const Point<T> &point) {
        out << "[(" << point.get_x() << ", " << point.get_y() << ")]";
        return out;
    }

    bool isUnLegal() const { return _x == -1 && _y == -1; }

private:
    T _x;
    T _y;
};
}

