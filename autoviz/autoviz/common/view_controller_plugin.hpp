/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/common/view_controller_registry.hpp"

/** Export from Autoviz view-controller plugin shared libraries. */
#define AUTOVIZ_VIEW_CONTROLLER_PLUGIN_EXPORT \
  extern "C" void autoviz_register_view_controllers

namespace autoviz {
namespace common {

inline void RegisterViewControllerPlugin(
    ViewControllerRegistry* registry, const char* type,
    ViewControllerApplier applier) {
  if (registry != nullptr) {
    registry->registerType(type, std::move(applier));
  }
}

}  // namespace common
}  // namespace autoviz
