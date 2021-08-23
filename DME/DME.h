/**
 * @file DME.h
 * @author Dawn Li (dawnli619215645@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-08-03
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <deque>
#include <ostream>
#include <string>
#include <vector>

#include "../include/common/utility.h"

using namespace std;

class ClusterVertex {
public:
    ClusterVertex() = default;
    ClusterVertex(Point<DBU> point, string name) : ClusterVertex() {
        _point = point;
        _name = name;
    };
    //   explicit ClusterVertex(ITDPVertex *vertex) {
    //     _point = new Point<DBU>(vertex->get_point()->get_x(),
    //                             vertex->get_point()->get_y());
    //     _name = vertex->get_name();
    //     _skew = vertex->get_skew();
    //   };
    ~ClusterVertex() = default;

    bool isLegal() { return !_point.isUnLegal(); }
    bool isSink() {
        if (_name.size() <= 2) {
            return false;
        }
        string suffix_name(_name.end() - 2, _name.end());
        return suffix_name == "CK";
    }
    bool isLCB() {
        if (_name.size() <= 3) {
            return false;
        }
        string suffix_name(_name.end() - 3, _name.end());
        return suffix_name == "LCB";
    }
    bool isBuffer() {
        if (_name.size() <= 3) {
            return false;
        }
        string suffix_name(_name.end() - 3, _name.end());
        return suffix_name == "BUF";
    }
    // get
    Point<DBU> get_point() const { return _point; }
    double get_skew() const { return _skew; }
    string get_name() const { return _name; }
    // set
    void set_point(Point<DBU> point) { _point = point; }
    void set_skew(const double skew) { _skew = skew; }
    void set_name(const string name) { _name = name; }

private:
    string _name = "";
    Point<DBU> _point = Point<DBU>(-1, -1);  // loaction.
    double _skew = 0;                        // left -> right.
};

using ClusterTopo = vector<ClusterVertex *>;
using MergeSegment = pair<Point<DBU>, Point<DBU>>;
using WireSegment = deque<Point<DBU>>;

class DME {
public:
    /**
     * @brief For a single cluster compute center location
     *
     * @param binary_tree
     * @param buffer_name
     */

    void computeTpClusterCenterLocation(ClusterTopo &binary_tree, const string &buffer_name,
                                        const unsigned &buffer_site_width);

private:
    /**
     * @brief Compute merge segment(point)
     *
     * @param ms1
     * @param ms2
     * @param cluster_center
     * @param buffer_name
     * @param is_buffer
     * @return Point<DBU>*
     */

    Point<DBU> computeMergePoint(const MergeSegment &ms1, const MergeSegment &ms2, const Point<DBU> &cluster_center,
                                 const string &buffer_name, const bool &is_buffer) const;

    /**
     * @brief Convert skew to distance(the cap and the res should be
     * changed)
     *
     * @param left_point
     * @param right_point
     * @param geometric_center
     * @param skew
     * @return DBU
     */

    DBU skewToDistance(const ClusterVertex *left_point, const ClusterVertex *right_point,
                       const Point<DBU> &geometric_center, const double &skew) const;
};
