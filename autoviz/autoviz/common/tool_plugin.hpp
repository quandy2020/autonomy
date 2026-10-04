/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file tool_plugin.hpp
 * @brief Macro and helper for Autoviz tool plugin shared libraries.
 *
 * @see ToolRegistry
 * @see AUTOVIZ_TOOL_PLUGIN_EXPORT
 */

#pragma once

#include "autoviz/common/tool_registry.hpp"

/**
 * @brief Declares the C export for tool plugins.
 *
 * Expands to @c extern "C" void autoviz_register_tools.
 */
#define AUTOVIZ_TOOL_PLUGIN_EXPORT extern "C" void autoviz_register_tools

namespace autoviz {
namespace common {

/**
 * @brief Registers one tool into @p registry (null-safe).
 *
 * @param registry Target registry (no-op if @c nullptr).
 * @param id Tool id C-string (e.g. @c "Interact").
 * @param creator Factory that constructs the @ref Tool instance.
 */
inline void RegisterToolPlugin(ToolRegistry* registry, const char* id,
                               ToolCreator creator) {
  if (registry != nullptr) {
    registry->registerTool(id, std::move(creator));
  }
}

}  // namespace common
}  // namespace autoviz
