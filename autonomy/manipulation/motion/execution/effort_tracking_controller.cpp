/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/execution/effort_tracking_controller.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy {
namespace manipulation {
namespace execution {

std::vector<double> CorrectEffortCommand(
    const std::vector<double>& desired, const std::vector<double>& actual,
    double proportional_gain, double max_abs_effort) {
  std::vector<double> cmd = desired;
  const std::size_t n = std::min(desired.size(), actual.size());
  for (std::size_t i = 0; i < n; ++i) {
    cmd[i] = desired[i] + proportional_gain * (desired[i] - actual[i]);
  }
  if (max_abs_effort > 0.0) {
    for (double& v : cmd) {
      v = std::clamp(v, -max_abs_effort, max_abs_effort);
    }
  }
  return cmd;
}

void EffortTrackingController::SetCommandPublisher(CommandPublisher pub) {
  std::lock_guard<std::mutex> lock(mutex_);
  publisher_ = std::move(pub);
}

void EffortTrackingController::SetStateProvider(StateProvider provider) {
  std::lock_guard<std::mutex> lock(mutex_);
  state_provider_ = std::move(provider);
}

void EffortTrackingController::SetDesired(const automsgs::msgs::sensor_msgs::JointState& desired) {
  std::lock_guard<std::mutex> lock(mutex_);
  desired_.assign(desired.effort().begin(), desired.effort().end());
}

void EffortTrackingController::Clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  desired_.clear();
}

bool EffortTrackingController::Publish() {
  CommandPublisher publisher;
  StateProvider provider;
  std::vector<double> desired;
  double proportional_gain = 0.0;
  double max_abs = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (desired_.empty() || !publisher_) {
      return false;
    }
    publisher = publisher_;
    provider = state_provider_;
    desired = desired_;
    proportional_gain = proportional_gain_;
    max_abs = max_abs_effort_;
  }

  std::vector<double> actual;
  if (provider) {
    const auto state = provider();
    actual.assign(state.effort().begin(), state.effort().end());
  }
  const std::vector<double> cmd =
      CorrectEffortCommand(desired, actual, proportional_gain, max_abs);
  if (cmd.empty()) {
    return false;
  }
  publisher(cmd);
  return true;
}

}  // namespace execution
}  // namespace manipulation
}  // namespace autonomy
