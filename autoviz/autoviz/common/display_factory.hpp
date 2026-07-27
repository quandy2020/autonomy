/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autoviz/common/session_config.hpp"
#include "autoviz/display/display.hpp"

namespace autoviz {
namespace common {

class DisplayFactory {
 public:
  static std::vector<std::string> supportedTypes();
  static DisplayConfig defaultForType(const std::string& type);
  static std::unique_ptr<display::Display> create(const DisplayConfig& config);
};

}  // namespace common
}  // namespace autoviz
