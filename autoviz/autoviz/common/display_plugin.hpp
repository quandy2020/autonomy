/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file display_plugin.hpp
 * @brief Macro and helper for Autoviz display plugin shared libraries.
 *
 * Plugin @c .so files export @c autoviz_register_displays and call
 * @ref RegisterDisplayPlugin for each type they provide.
 *
 * @see DisplayRegistry
 * @see AUTOVIZ_DISPLAY_PLUGIN_EXPORT
 */

#pragma once

#include "autoviz/common/display_registry.hpp"

/**
 * @brief Declares the C export symbol plugins must define.
 *
 * Expands to @c extern "C" void autoviz_register_displays — implement as
 * @code
 * AUTOVIZ_DISPLAY_PLUGIN_EXPORT(DisplayRegistry* registry) { … }
 * @endcode
 */
#define AUTOVIZ_DISPLAY_PLUGIN_EXPORT extern "C" void autoviz_register_displays

namespace autoviz {
namespace common {

/**
 * @brief Registers one display type into @p registry (null-safe).
 *
 * @param registry Target registry (no-op if @c nullptr).
 * @param type Display type id C-string.
 * @param creator Instance factory.
 * @param default_config Default @ref DisplayConfig factory.
 */
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
