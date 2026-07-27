/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>

namespace autoviz {
namespace common {

/** rviz_common pluginlib metadata (Display / Tool / ViewController). */
struct PluginInfo {
  std::string class_id;
  std::string name;
  std::string description;
  std::string package;
};

}  // namespace common
}  // namespace autoviz
