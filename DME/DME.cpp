/**
 * @file DME.cpp
 * @author Dawn Li (dawnli619215645@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-08-03
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "DME.h"

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace std;

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

Point<DBU> DME::computeMergePoint(const MergeSegment &ms1, const MergeSegment &ms2, const Point<DBU> &cluster_center,
                                  const string &buffer_name, const bool &is_buffer) const {
    /**
     * @brief Compute merge segment
     *
     */
    vector<Point<DBU>> points = {ms1.first, ms1.second, ms2.first, ms2.second};
    sort(points.begin(), points.end(), [&](Point<DBU> p1, Point<DBU> p2) { return p1.get_x() < p2.get_x(); });
    Point<DBU> left_merge_point = points[1];
    Point<DBU> right_merge_point = points[2];
    // if (is_buffer) {
    //     WireSegment buffer_optional_position = {left_merge_point,
    //                                             right_merge_point};
    //     _clk_network->addSegOptPosition(buffer_name + "_root",
    //                                     buffer_optional_position);
    // }
    /**
     * @brief Merge normal point
     *
     */
    // return cluster_center.computeDist(left_merge_point) <
    //                cluster_center.computeDist(right_merge_point)
    //            ? left_merge_point
    //            : right_merge_point;
    /**
     * @brief Merge center point
     *
     */
    DBU dx = (cluster_center.get_x() - left_merge_point.get_x()) * (cluster_center.get_x() - right_merge_point.get_x());
    DBU dy = (cluster_center.get_y() - left_merge_point.get_y()) * (cluster_center.get_y() - right_merge_point.get_y());
    int direction = right_merge_point.get_y() > left_merge_point.get_y() ? 1 : -1;
    /**
     * @brief Return minimize center distance point
     *
     */
    if (dx < 0 && dy < 0) {
        Point<DBU> close_point =
            Point<DBU>(cluster_center.get_x(),
                       left_merge_point.get_y() + direction * (cluster_center.get_x() - left_merge_point.get_x()));
        return close_point;
    } else {
        return cluster_center.computeDist(left_merge_point) < cluster_center.computeDist(right_merge_point)
                   ? left_merge_point
                   : right_merge_point;
    }
}

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

DBU DME::skewToDistance(const ClusterVertex *left_point, const ClusterVertex *right_point,
                        const Point<DBU> &geometric_center, const double &skew) const {
    double g_cap = 0.16e-15;  // need to be global
    double g_res = 2.535;     // need to be global
    DBU left_HPWL = left_point->get_point().computeDist(geometric_center);
    DBU right_HPWL = right_point->get_point().computeDist(geometric_center);
    DBU delta_length = 2.5e-4 * skew / (g_cap * g_res * (left_HPWL + right_HPWL));
    return delta_length;
}

/**
 * @brief For a single cluster compute center location
 *
 * @param binary_tree
 * @param buffer_name
 */

void DME::computeTpClusterCenterLocation(ClusterTopo &binary_tree, const string &buffer_name,
                                         const unsigned &buffer_site_width) {
    /**
     * @brief Compute euclidean center of geometry
     *
     */
    DBU sum_x = 0;
    DBU sum_y = 0;
    int ceil_num = 0;
    for (auto vertex : binary_tree) {
        if (vertex == binary_tree[0]) {
            continue;
        }
        if (vertex->isBuffer() || vertex->isLCB() || vertex->isSink()) {
            sum_x += vertex->get_point().get_x();
            sum_y += vertex->get_point().get_y();
            ++ceil_num;
        }
    }
    if (ceil_num == 0) {
        cout << "[CTS-flow] Error: The Cluster " << buffer_name << "has not ceil." << endl;
        return;
    }
    Point<DBU> center = Point<DBU>(sum_x / ceil_num, sum_y / ceil_num);
    /**
     * @brief Reverse hierarchical traversal(from n-1 topology level to 1)
     *
     */
    int topo_level = log2(binary_tree.size() + 1);
    int index = pow(2, topo_level - 1) - 2;
    while (index >= 0) {
        ClusterVertex *left_vertex = binary_tree[index * 2 + 1];
        ClusterVertex *right_vertex = binary_tree[index * 2 + 2];
        // It shows that father should be computed
        if (left_vertex->isLegal() && right_vertex->isLegal()) {
            DBU left_x = left_vertex->get_point().get_x();
            DBU left_y = left_vertex->get_point().get_y();
            DBU right_x = right_vertex->get_point().get_x();
            DBU right_y = right_vertex->get_point().get_y();
            // DME part
            double skew = right_vertex->get_skew();
            DBU skew_dist = skewToDistance(left_vertex, right_vertex, center, skew);
            // Return type should be considered
            DBU manhattan_dist = fabs(left_x - right_x) + fabs(left_y - right_y);
            // It shows is normal DME
            if (manhattan_dist > skew_dist) {
                DBU radius_left = (manhattan_dist - skew_dist) / 2.0;
                DBU radius_right = manhattan_dist - radius_left;
                int direction_x = left_x < right_x ? 1 : -1;
                int direction_y = left_y < right_y ? 1 : -1;

                MergeSegment left_ms = make_pair(Point<DBU>(left_x + direction_x * radius_left, left_y),
                                                 Point<DBU>(left_x, left_y + direction_y * radius_left));
                MergeSegment right_ms = make_pair(Point<DBU>(right_x - direction_x * radius_right, right_y),
                                                  Point<DBU>(right_x, right_y - direction_y * radius_right));

                Point<DBU> merge_point = computeMergePoint(left_ms, right_ms, center, buffer_name, index == 0);

                binary_tree[index]->set_point(merge_point);
            } else {
                binary_tree[index]->set_point(left_vertex->get_point());
            }
        }
        --index;
    }
    /**
     * @brief Check the buffer location and move it if illegal
     *
     */
    ClusterVertex *buffer = binary_tree[0];
    // DBU row_unit = _setting->get_graph_unit().first;
    // DBU site_unit = _setting->get_graph_unit().second;
    // DBU spacing_x = _setting->get_spacing().first;
    // DBU spacing_y = _setting->get_spacing().second;
    // int row_index = (buffer->get_point().get_y() - spacing_y) / row_unit;
    // int site_index = (buffer->get_point().get_x() - spacing_x) / site_unit;
/**
 * @brief Search the closest index
 *
 */
//   if (!_setting->checkFeasible(row_index, site_index, buffer_site_width)) {
//   /**
//    * @brief In a clockwise direction
//    * @brief Direction stand for the {row_direction, site_direction}
//    * @brief {-1, 1} \   {-1, -1}  /  {1, -1} *   {1, 1} *
//    * @brief          \          /             \       /
//    * @brief           *        *               \    /
//    *
//    */
//     vector<pair<int, int>> directions = {{-1, 1}, {-1, -1}, {1, -1}, {1, 1}};
//     int radius = 1;
//     int row_limit = _setting->get_region_blockage_graph_size().first;
//     int site_limit = _setting->get_region_blockage_graph_size().second;
//     while (radius <= 100) {
//       ++row_index;
//       for (auto direction : directions) {
//         int shift = 1;
//         while (shift <= radius) {
//           row_index += direction.first;
//           site_index += direction.second;
//           if (row_index >= 0 && row_index < row_limit && site_index >= 0 &&
//             site_index < site_limit &&
//             _setting->checkFeasible(row_index, site_index,
//               buffer_site_width)) {
//             goto Embedding;
//         }
//         ++shift;
//       }
//     }
//     ++radius;
//   }
//   cout << "[CTS-flow] Error: In the current unit manhattan distance range, "
//   "embedding work can't be completed, please adjust a more larger "
//   "range."
//   << endl;
//   return;
// }
Embedding:
    /**
     * @brief Embedding the real location and reset the pointer
     *
     */
    // Point<DBU> buffer_embbed_point = Point<DBU>(site_index * site_unit + spacing_x, row_index * row_unit +
    // spacing_y); buffer->set_point(buffer_embbed_point); _setting->set_region_blockage(row_index, site_index,
    // buffer_site_width);
    /**
     * @brief Add buffer to network
     *
     */
    cout << "      + PLACED ( " << buffer->get_point().get_x() << " " << buffer->get_point().get_y() << ") N ;" << endl;
    buffer->set_name(buffer_name);
    // _clk_network->addClkBuffer(buffer_name,
    //                            "",  // should considered the master name
    //                            buffer->get_point().get_x(),
    //                            buffer->get_point().get_y(), 1);
}