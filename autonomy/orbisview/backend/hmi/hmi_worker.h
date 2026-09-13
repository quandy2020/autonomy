/*
 * Copyright 2026 The Openbot Authors
 *
 * HMIWorker — mode / component registry (Dreamview hmi_worker counterpart).
 */

#pragma once

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace autonomy {
namespace orbisview {
namespace core {

struct HmiModule {
  std::string id;
  std::string title;
  bool expected{true};
  bool healthy{false};
  double delay_ms{0};
};

struct HmiMode {
  std::string id;
  std::string title;
  std::vector<std::string> module_ids;
  std::vector<std::string> channel_hints;
};

class HmiWorker {
 public:
  HmiWorker();

  // Optional: load extra modes from conf/hmi_modes (*.json, best-effort).
  void LoadModesDir(const std::string& dir);

  std::vector<HmiMode> Modes() const;
  std::string CurrentModeId() const;
  bool SetMode(const std::string& mode_id);
  bool ModuleAction(const std::string& module_id, const std::string& action);
  void UpdateChannelHealth(const std::string& channel, double delay_ms,
                           bool alive);

  std::string StatusJson() const;
  std::string ComponentsJson() const;

 private:
  void EnsureBuiltinModes();

  mutable std::mutex mutex_;
  std::string current_mode_{"default"};
  std::vector<HmiMode> modes_;
  std::unordered_map<std::string, HmiModule> modules_;
};

}  // namespace core
}  // namespace orbisview
}  // namespace autonomy
