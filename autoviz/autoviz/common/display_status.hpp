/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>

namespace autoviz {
namespace display {

enum class DisplayStatusLevel { kOk, kWarn, kError };

struct DisplayStatus {
  DisplayStatusLevel level = DisplayStatusLevel::kOk;
  std::string message;
};

}  // namespace display
}  // namespace autoviz
