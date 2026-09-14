/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/teleop/teleop.hpp"

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

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy
