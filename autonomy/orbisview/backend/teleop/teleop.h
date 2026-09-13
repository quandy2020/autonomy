/*
 * Copyright 2026 The Openbot Authors
 *
 * TeleopService — last cmd_vel command state (Dreamview teleop counterpart).
 */

#pragma once

#include <functional>
#include <mutex>
#include <string>

namespace autonomy {
namespace orbisview {
namespace backend {

class TeleopService {
 public:
  using PublishFn = std::function<void(double vx, double wz)>;

  void SetPublisher(PublishFn fn) { publish_ = std::move(fn); }
  void SetCmdVel(double vx, double wz);
  void GetCmdVel(double* vx, double* wz) const;
  std::string StatusJson() const;

 private:
  mutable std::mutex mutex_;
  double vx_{0};
  double wz_{0};
  PublishFn publish_;
};

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy
