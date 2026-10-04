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

namespace autoviz {
namespace common {

/**
 * @struct PluginInfo
 * @brief Human- and machine-readable identity for a plugin class.
 *
 * Used in Settings / About UIs and when persisting the active transformer id
 * into @ref SessionConfig.
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
};

}  // namespace common
}  // namespace autoviz
