/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file display_registry.hpp
 * @brief Built-in + dynamically loaded display type registry (pluginlib-style).
 *
 * Registers creator / default-config factories for each display type and can
 * load shared libraries exporting @c autoviz_register_displays.
 *
 * @see DisplayFactory
 * @see DisplayPlugin
 * @see DisplayCatalog
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "autoviz/common/session_config.hpp"
#include "autoviz/display/display.hpp"

namespace autoviz {
namespace common {

/**
 * @brief Factory that constructs a @ref display::Display from config.
 */
using DisplayCreator =
    std::function<std::unique_ptr<display::Display>(const DisplayConfig&)>;

/**
 * @brief Factory that returns a default @ref DisplayConfig for a type.
 */
using DisplayDefaultConfig = std::function<DisplayConfig()>;

/**
 * @class DisplayRegistry
 * @brief Process-wide singleton of display type creators.
 *
 * ## Plugin loading
 *
 * @ref loadPluginsFromEnv / @ref loadPluginsFromPath open libraries whose
 * export is @c autoviz_register_displays(@ref DisplayRegistry*), typically
 * via @ref AUTOVIZ_DISPLAY_PLUGIN_EXPORT.
 */
class DisplayRegistry {
 public:
  /**
   * @brief Returns the process-wide registry singleton.
   * @return Mutable registry reference.
   */
  static DisplayRegistry& instance();

  /**
   * @brief Registers all built-in Autoviz display types.
   *
   * Safe to call once at startup; subsequent calls may no-op or re-register
   * depending on implementation.
   */
  void registerBuiltinTypes();

  /**
   * @brief Registers (or replaces) a display type.
   *
   * @param type Type id string (e.g. @c "Grid").
   * @param creator Factory that builds the display instance.
   * @param default_config Factory for a fresh default @ref DisplayConfig.
   */
  void registerType(const std::string& type, DisplayCreator creator,
                    DisplayDefaultConfig default_config);

  /**
   * @brief Lists all currently registered type ids.
   * @return Type name vector.
   */
  std::vector<std::string> supportedTypes() const;

  /**
   * @brief Returns the default config for @p type.
   *
   * @param type Display type id.
   * @return Default @ref DisplayConfig (empty if unknown).
   */
  DisplayConfig defaultForType(const std::string& type) const;

  /**
   * @brief Creates a display instance from @p config.
   *
   * @param config Display configuration.
   * @return New display, or @c nullptr if type is unregistered.
   */
  std::unique_ptr<display::Display> create(const DisplayConfig& config) const;

  /**
   * @brief Loads @c .so / @c .dylib plugins from @c AUTOVIZ_PLUGIN_PATH.
   *
   * Plugins must export @c autoviz_register_displays(DisplayRegistry*).
   * @see loadPluginsFromPath()
   */
  void loadPluginsFromEnv();

  /**
   * @brief Loads plugin libraries found under @p path.
   *
   * @param path Directory to scan for shared libraries matching
   *        @ref PluginLoader::libraryFilenameFilters.
   */
  void loadPluginsFromPath(const std::string& path);

 private:
  DisplayRegistry() = default;

  /**
   * @struct Entry
   * @brief Per-type creator pair stored in the registry map.
   */
  struct Entry {
    DisplayCreator creator;              /**< Instance factory. */
    DisplayDefaultConfig default_config; /**< Default config factory. */
  };

  std::unordered_map<std::string, Entry> entries_;
  std::vector<void*> plugin_handles_; /**< Opened plugin library handles. */
};

}  // namespace common
}  // namespace autoviz
