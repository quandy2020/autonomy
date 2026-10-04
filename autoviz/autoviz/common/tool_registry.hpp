/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file tool_registry.hpp
 * @brief Built-in + dynamically loaded interactive tools (pluginlib-style).
 *
 * Registers @ref ToolCreator factories by tool id and can load shared
 * libraries exporting @c autoviz_register_tools.
 *
 * @see Tool
 * @see ToolManager
 * @see ToolPlugin
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace common {

/**
 * @brief Factory that constructs a @ref Tool instance.
 */
using ToolCreator = std::function<std::unique_ptr<Tool>()>;

/**
 * @class ToolRegistry
 * @brief Process-wide singleton of tool creators and labels.
 */
class ToolRegistry {
 public:
  /**
   * @brief Returns the process-wide registry singleton.
   * @return Mutable registry reference.
   */
  static ToolRegistry& instance();

  /**
   * @brief Registers all built-in tools (Interact, Move Camera, Measure, …).
   */
  void registerBuiltinTools();

  /**
   * @brief Registers (or replaces) a tool factory.
   *
   * @param id Tool id string (e.g. @c "Interact").
   * @param creator Factory constructing the tool.
   * @param label Optional UI label; when @c nullptr the tool's own
   *        @ref Tool::label is used after creation.
   */
  void registerTool(const std::string& id, ToolCreator creator,
                    const char* label = nullptr);

  /**
   * @brief Lists all registered tool ids.
   * @return Tool id vector.
   */
  std::vector<std::string> toolIds() const;

  /**
   * @brief Returns the display label for a tool id.
   *
   * @param id Tool id.
   * @return Label string (empty if unknown).
   */
  QString toolLabel(const std::string& id) const;

  /**
   * @brief Instantiates a tool by id.
   *
   * @param id Tool id.
   * @return New tool, or @c nullptr if unregistered.
   */
  std::unique_ptr<Tool> create(const std::string& id) const;

  /**
   * @brief Loads tool plugins from @c AUTOVIZ_PLUGIN_PATH.
   */
  void loadPluginsFromEnv();

  /**
   * @brief Loads tool plugins from a directory.
   * @param path Directory to scan for shared libraries.
   */
  void loadPluginsFromPath(const std::string& path);

 private:
  ToolRegistry() = default;

  /**
   * @struct Entry
   * @brief Per-tool creator and optional cached label.
   */
  struct Entry {
    ToolCreator creator; /**< Instance factory. */
    std::string label;   /**< Optional UI label override. */
  };

  std::unordered_map<std::string, Entry> entries_;
  std::vector<void*> plugin_handles_;
};

}  // namespace common
}  // namespace autoviz
