/*
 * Copyright 2026 The Openbot Authors
 *
 * Dynamic plugin host: dlopen / dlsym with isolated failure handling.
 */

#pragma once

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/orbisview/backend/common/plugins/orbisview_plugin_abi.h"
#include "autonomy/orbisview/backend/common/plugins/registry.h"

namespace autonomy {
namespace orbisview {
namespace plugins {

struct LoadedPlugin {
  std::string path;
  void* handle{nullptr};
  PluginInfo info;
  OrbisPluginUnregisterFn unregister{nullptr};
};

class PluginHost {
 public:
  explicit PluginHost(PluginRegistry* registry);
  ~PluginHost();

  PluginHost(const PluginHost&) = delete;
  PluginHost& operator=(const PluginHost&) = delete;

  /** Scan directory for *.so / *.dylib and attempt load. Failures are recorded. */
  int ScanAndLoad(const std::string& directory);

  bool Load(const std::string& path);
  bool Unload(const std::string& id);
  bool Reload(const std::string& id);

  std::string StatusJson() const;

 private:
  bool LoadLocked(const std::string& path);
  bool UnloadLocked(const std::string& id);

  PluginRegistry* registry_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, LoadedPlugin> loaded_;  // id -> plugin
  std::vector<std::string> load_failures_;
};

}  // namespace plugins
}  // namespace orbisview
}  // namespace autonomy
