/*
 * Copyright 2026 The Openbot Authors
 *
 * Autolink PluginManager registration for manipulation backends.
 */

#pragma once

#include <memory>
#include <string>

#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {

/**
 * @brief Idempotent in-process RegisterInProcessClass for all manipulation
 *        plugin types (planners, kinematics, collision, adapters, capabilities).
 */
void RegisterManipulationPlugins();

/**
 * @brief Resolve a config id / short alias to the C++ class name.
 * @param[in] id Alias (e.g. "hybrid") or already-qualified class name.
 * @return Mapped class name, or @p id unchanged when unknown.
 */
std::string ResolvePluginAlias(const std::string& id);

/**
 * @brief Create a plugin by config id or C++ class name.
 *
 * Calls RegisterManipulationPlugins() then PluginManager::CreateInstance.
 *
 * @tparam Base Plugin base interface (PlannerBase, KinematicsBase, ...).
 * @param[in] id Alias or class name (see ResolvePluginAlias).
 * @return Shared instance, or nullptr if registration / creation fails.
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
