/**
 * @file IdbAdapter.h
 * @author Dawn Li (dawnli619215645@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-08-25
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "AdapterInterface.h"
#include "IdbDesign.h"
#include "common/Logger.h"
#include "common/Parameter.h"
#include "common/Utility.h"

using IdbDesign = PCL::iDB::IdbDesign;
using IdbInstanceList = PCL::iDB::IdbInstanceList;
using IdbInstance = PCL::iDB::IdbInstance;
using IdbNetList = PCL::iDB::IdbNetList;
using IdbNet = PCL::iDB::IdbNet;
using IdbPins = PCL::iDB::IdbPins;
using IdbPin = PCL::iDB::IdbPin;

namespace itdp {
class IdbAdapter : public AdapterInterface {
 public:
  IdbAdapter() = delete;
  IdbAdapter(IdbDesign *idb_design);
  ~IdbAdapter();

  std::vector<Instance *> get_instance_pvec() const { return _instance_pvec; }
  std::vector<Net *> get_net_pvec() const { return _net_pvec; }
  std::vector<Pin *> get_pin_pvec() const { return _pin_pvec; }

  std::string get_design_name() const { return _design_name; }
  double get_core_edge_x() const { return _core_edge_x; }
  double get_core_edge_y() const { return _core_edge_y; }
  DBU get_row_height() const { return _row_height; }

 private:
  IdbDesign *_idb_design;
  Logger *_log;
  std::vector<Instance *> _instance_pvec;
  std::vector<Net *> _net_pvec;
  std::vector<Pin *> _pin_pvec;
  std::map<std::string, Pin *> _name_to_pin_ptr;
  std::string _design_name;
  double _core_edge_x;
  double _core_edge_y;
  DBU _row_height;
  void dataTransmit();
  Pin *add_pin_pointer(IdbPin *idb_pin);
  bool isSequentialPin(IdbPin *idb_pin);
};
}  // namespace itdp