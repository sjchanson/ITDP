/*
 * @Author: ShiJian Chen
 * @Date: 2021-08-04 16:09:17
 * @LastEditors: Shijian Chen
 * @LastEditTime: 2021-08-04 19:28:42
 * @Description:
 */

#include "clockTree.h"

namespace itdp {

ClockNode::ClockNode(SequentialElement* instance) : _master(nullptr) { _instance = instance; }

ClockTree::ClockTree(ClockNode* root) { _root = root; }

}  // namespace itdp