/*
 * Copyright 2026 The Openbot Authors
 *
 * HMI facade over HmiWorker (Dreamview HMI layout).
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autonomy/orbisview/backend/hmi/hmi_worker.h"

namespace autonomy {
namespace orbisview {
namespace core {

class Hmi {
 public:
  Hmi();

  void LoadModesDir(const std::string& dir);

  std::vector<HmiMode> Modes() const;
  std::string CurrentModeId() const;
  bool SetMode(const std::string& mode_id);
  bool ModuleAction(const std::string& module_id, const std::string& action);
  void UpdateChannelHealth(const std::string& channel, double delay_ms,
                           bool alive);

  std::string StatusJson() const;
  std::string ComponentsJson() const;

  HmiWorker* Worker() { return worker_.get(); }

 private:
  std::unique_ptr<HmiWorker> worker_;
};

}  // namespace core
}  // namespace orbisview
}  // namespace autonomy
