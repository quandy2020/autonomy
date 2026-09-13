/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/teleop/teleop.h"

#include <sstream>

namespace autonomy {
namespace orbisview {
namespace backend {

void TeleopService::SetCmdVel(double vx, double wz) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    vx_ = vx;
    wz_ = wz;
  }
  if (publish_) publish_(vx, wz);
}

void TeleopService::GetCmdVel(double* vx, double* wz) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (vx) *vx = vx_;
  if (wz) *wz = wz_;
}

std::string TeleopService::StatusJson() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream oss;
  oss << "{\"vx\":" << vx_ << ",\"wz\":" << wz_ << "}";
  return oss.str();
}

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy
