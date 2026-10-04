/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file view_controller_plugin.hpp
 * @brief Macro and helper for Autoviz view-controller plugin shared libraries.
 *
 * @see ViewControllerRegistry
 * @see AUTOVIZ_VIEW_CONTROLLER_PLUGIN_EXPORT
 */

#pragma once

#include "autoviz/common/view_controller_registry.hpp"

/**
 * @brief Declares the C export for view-controller plugins.
 *
 * Expands to @c extern "C" void autoviz_register_view_controllers.
 */
#define AUTOVIZ_VIEW_CONTROLLER_PLUGIN_EXPORT \
  extern "C" void autoviz_register_view_controllers

namespace autoviz {
namespace common {

/**
 * @brief Registers one view-controller type into @p registry (null-safe).
 *
 * @param registry Target registry (no-op if @c nullptr).
 * @param type Type name C-string (e.g. @c "Orbit").
 * @param applier Functor that configures a @ref rendering::ViewController.
 */
inline void RegisterViewControllerPlugin(
    ViewControllerRegistry* registry, const char* type,
    ViewControllerApplier applier) {
  if (registry != nullptr) {
    registry->registerType(type, std::move(applier));
  }
}

}  // namespace common
}  // namespace autoviz
