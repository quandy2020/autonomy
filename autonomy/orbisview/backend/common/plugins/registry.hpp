/*
 * Copyright 2026 The Openbot Authors
 *
 * Panel / source / tool plugin registry (Phase 4 + hot-load).
 * A single plugin load failure must not prevent base app start.
 */

#pragma once

#include <functional>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/orbisview/backend/common/stream_envelope.hpp"

namespace autonomy {
namespace orbisview {
namespace plugins {

struct PluginInfo {
  std::string id;
  std::string kind;  // panel | source | tool
  std::string title;
  std::string version{"1.0"};
  bool enabled{true};
  std::string source{"builtin"};  // builtin | dynamic
  std::string path;               // shared library path when dynamic
};

class PluginRegistry {
 public:
  bool Register(PluginInfo info) {
    if (info.id.empty()) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (plugins_.count(info.id)) {
      failures_.push_back("duplicate plugin id: " + info.id);
      return false;
    }
    plugins_[info.id] = std::move(info);
    return true;
  }

  bool Unregister(const std::string& id) {
    std::lock_guard<std::mutex> lock(mutex_);
    return plugins_.erase(id) > 0;
  }

  void RegisterBuiltins() {
    Register({"inspector", "panel", "Message Inspector", "1.0", true, "builtin",
              {}});
    Register(
        {"diagnostics", "panel", "Diagnostics", "1.0", true, "builtin", {}});
    Register({"recorder", "panel", "Record / Playback", "1.0", true, "builtin",
              {}});
    Register({"stats", "panel", "Channel Stats", "1.0", true, "builtin", {}});
    Register(
        {"exploration", "panel", "Exploration", "1.0", true, "builtin", {}});
    Register({"navigation", "panel", "Navigation", "1.0", true, "builtin", {}});
    Register({"mapping", "panel", "Mapping", "1.0", true, "builtin", {}});
    Register({"plugins", "panel", "Plugin Host", "1.0", true, "builtin", {}});
    Register({"pnc", "panel", "PNC Monitor", "1.0", true, "builtin", {}});
  }

  void RecordFailure(const std::string& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    failures_.push_back(msg);
  }

  std::string ToJson() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::ostringstream oss;
    oss << "{\"op\":\"plugins\",\"plugins\":[";
    bool first = true;
    for (const auto& kv : plugins_) {
      if (!first) oss << ',';
      first = false;
      const auto& p = kv.second;
      oss << "{\"id\":" << core::JsonEscape(p.id)
          << ",\"kind\":" << core::JsonEscape(p.kind)
          << ",\"title\":" << core::JsonEscape(p.title)
          << ",\"version\":" << core::JsonEscape(p.version)
          << ",\"enabled\":" << (p.enabled ? "true" : "false")
          << ",\"source\":" << core::JsonEscape(p.source)
          << ",\"path\":" << core::JsonEscape(p.path) << '}';
    }
    oss << "],\"failures\":[";
    for (size_t i = 0; i < failures_.size(); ++i) {
      if (i) oss << ',';
      oss << core::JsonEscape(failures_[i]);
    }
    oss << "]}";
    return oss.str();
  }

 private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, PluginInfo> plugins_;
  std::vector<std::string> failures_;
};

}  // namespace plugins
}  // namespace orbisview
}  // namespace autonomy
