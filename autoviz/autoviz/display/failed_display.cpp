/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/failed_display.hpp"

namespace autoviz {
namespace display {

FailedDisplay::FailedDisplay(std::string type, std::string reason)
    : type_(std::move(type)), reason_(std::move(reason)) {}

void FailedDisplay::onEnable() {
  setStatusError(reason_.empty() ? "Failed to load display" : reason_);
}

}  // namespace display
}  // namespace autoviz
