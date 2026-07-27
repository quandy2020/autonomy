/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

using DisplayCreator =
    std::function<std::unique_ptr<display::Display>(const DisplayConfig&)>;
using DisplayDefaultConfig = std::function<DisplayConfig()>;

/** Built-in + dynamically loaded display types (RViz pluginlib-style). */
class DisplayRegistry {
 public:
  static DisplayRegistry& instance();

  void registerBuiltinTypes();
  void registerType(const std::string& type, DisplayCreator creator,
                    DisplayDefaultConfig default_config);

  std::vector<std::string> supportedTypes() const;
  DisplayConfig defaultForType(const std::string& type) const;
  std::unique_ptr<display::Display> create(const DisplayConfig& config) const;

  /** Load `.so` plugins exporting `autoviz_register_displays(DisplayRegistry*)`. */
  void loadPluginsFromEnv();
  void loadPluginsFromPath(const std::string& path);

 private:
  DisplayRegistry() = default;

  struct Entry {
    DisplayCreator creator;
    DisplayDefaultConfig default_config;
  };

  std::unordered_map<std::string, Entry> entries_;
  std::vector<void*> plugin_handles_;
};

}  // namespace common
}  // namespace autoviz
