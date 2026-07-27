/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/common/display_registry.hpp"

/** Export from Autoviz display plugin shared libraries. */
#define AUTOVIZ_DISPLAY_PLUGIN_EXPORT extern "C" void autoviz_register_displays

namespace autoviz {
namespace common {

inline void RegisterDisplayPlugin(
    DisplayRegistry* registry,
    const char* type, DisplayCreator creator,
    DisplayDefaultConfig default_config) {
  if (registry != nullptr) {
    registry->registerType(type, std::move(creator), std::move(default_config));
  }
}

}  // namespace common
}  // namespace autoviz
