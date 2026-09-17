/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file plugin_ids.hpp
 * @brief Plugin id aliases + CreateInstance helper (MoveIt / planning-style).
 *
 * Concrete types register via AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN in
 * their .cpp; RegisterManipulationPlugins() additionally RegisterInProcessClass
 * as a dual-track fallback (same pattern as PlannerServer).
 */

#pragma once

#include <memory>
#include <string>

#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {

/**
 * @brief Resolve conf short id → C++ class name.
 * @param[in] id Alias or class name from config / CreatePlugin.
 * @return Registered class name; unknown ids are returned unchanged.
 */
std::string ResolvePluginAlias(const std::string& id);

/**
 * @brief Idempotent in-process RegisterInProcessClass fallback for all
 *        manipulation plugin types.
 */
void RegisterManipulationPlugins();

/**
 * @brief Create plugin by alias or class name after ensuring registration.
 * @tparam Base Plugin base interface type.
 * @param[in] id Alias (e.g. "time_parameterization") or class name.
 * @return Shared instance, or nullptr if resolution / create fails.
 */
template <typename Base>
std::shared_ptr<Base> CreatePlugin(const std::string& id) {
  RegisterManipulationPlugins();
  const std::string resolved = ResolvePluginAlias(id);
  return autolink::plugin_manager::PluginManager::Instance()
      ->CreateInstance<Base>(resolved);
}

}  // namespace manipulation
}  // namespace autonomy
