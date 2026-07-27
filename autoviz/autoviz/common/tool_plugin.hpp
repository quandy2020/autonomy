/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/common/tool_registry.hpp"

/** Export from Autoviz tool plugin shared libraries. */
#define AUTOVIZ_TOOL_PLUGIN_EXPORT extern "C" void autoviz_register_tools

namespace autoviz {
namespace common {

inline void RegisterToolPlugin(ToolRegistry* registry, const char* id,
                               ToolCreator creator) {
  if (registry != nullptr) {
    registry->registerTool(id, std::move(creator));
  }
}

}  // namespace common
}  // namespace autoviz
