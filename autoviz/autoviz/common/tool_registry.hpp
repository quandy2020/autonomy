/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace common {

using ToolCreator = std::function<std::unique_ptr<Tool>()>;

/** Built-in + dynamically loaded tools (RViz pluginlib-style facade). */
class ToolRegistry {
 public:
  static ToolRegistry& instance();

  void registerBuiltinTools();
  void registerTool(const std::string& id, ToolCreator creator,
                    const char* label = nullptr);

  std::vector<std::string> toolIds() const;
  QString toolLabel(const std::string& id) const;
  std::unique_ptr<Tool> create(const std::string& id) const;

  void loadPluginsFromEnv();
  void loadPluginsFromPath(const std::string& path);

 private:
  ToolRegistry() = default;

  struct Entry {
    ToolCreator creator;
    std::string label;
  };

  std::unordered_map<std::string, Entry> entries_;
  std::vector<void*> plugin_handles_;
};

}  // namespace common
}  // namespace autoviz
