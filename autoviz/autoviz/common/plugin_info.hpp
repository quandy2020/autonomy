/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plugin_info.hpp
 * @brief Metadata describing a registered Autoviz plugin class.
 *
 * Analogous to RViz @c pluginlib ClassLoader metadata for Display, Tool,
 * ViewController, and FrameTransformer plugins.
 *
 * @see DisplayRegistry
 * @see ToolRegistry
 * @see ViewControllerRegistry
 * @see TransformationManager
 */

#pragma once

#include <string>

#include <QIcon>

namespace autoviz {
namespace common {

/**
 * @struct PluginInfo
 * @brief Human- and machine-readable identity for a plugin class.
 *
 * RViz @c rviz_common::PluginInfo analogue. Used by @ref factory::Factory,
 * Settings / About UIs, and when persisting the active transformer id into
 * @ref SessionConfig.
 *
 * @see factory::Factory
 */
struct PluginInfo {
  /** Fully-qualified class id (e.g. @c "autoviz/AutolinkTf"). */
  std::string class_id;

  /** Short display name shown in UI combo boxes. */
  std::string name;

  /** Longer description / help text. */
  std::string description;

  /** Owning package or library name (informational). */
  std::string package;

  /** Optional class icon (Displays tree / Add dialog). */
  QIcon icon;

  friend bool operator==(const PluginInfo& lhs, const PluginInfo& rhs) {
    return lhs.class_id == rhs.class_id;
  }

  friend bool operator<(const PluginInfo& lhs, const PluginInfo& rhs) {
    return lhs.class_id < rhs.class_id;
  }
};

}  // namespace common
}  // namespace autoviz
