/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-04 16:09:17
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-19 10:05:46
 * @Description:
 */

#include "clockTree.h"

namespace itdp {

ClockNode::ClockNode(const SequentialElement* instance) : _instance(instance), _master(nullptr) {}

ClockTree::ClockTree(ClockNode* root) { _root = root; }

}  // namespace itdp