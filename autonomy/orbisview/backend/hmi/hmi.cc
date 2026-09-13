/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/hmi/hmi.h"

namespace autonomy {
namespace orbisview {
namespace core {

Hmi::Hmi() : worker_(std::make_unique<HmiWorker>()) {}

void Hmi::LoadModesDir(const std::string& dir) { worker_->LoadModesDir(dir); }

std::vector<HmiMode> Hmi::Modes() const { return worker_->Modes(); }

std::string Hmi::CurrentModeId() const { return worker_->CurrentModeId(); }

bool Hmi::SetMode(const std::string& mode_id) {
  return worker_->SetMode(mode_id);
}

bool Hmi::ModuleAction(const std::string& module_id,
                       const std::string& action) {
  return worker_->ModuleAction(module_id, action);
}

void Hmi::UpdateChannelHealth(const std::string& channel, double delay_ms,
                              bool alive) {
  worker_->UpdateChannelHealth(channel, delay_ms, alive);
}

std::string Hmi::StatusJson() const { return worker_->StatusJson(); }

std::string Hmi::ComponentsJson() const { return worker_->ComponentsJson(); }

}  // namespace core
}  // namespace orbisview
}  // namespace autonomy
