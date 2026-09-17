/*
 * Copyright 2026 The Openbot Authors
 *
 * Software effort tracking loop (MoveIt / ros2_control effort interface lite).
 *
 * Closes the loop above the hardware driver: desired feedforward + P on
 * measured JointState.effort → corrected effort_command. The driver still
 * applies torque; this layer only shapes the command.
 */

#pragma once

#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/manipulation/model/robot_model.hpp"

namespace autonomy {
namespace manipulation {
namespace execution {

/**
 * @brief P-corrected effort command from desired + measured efforts.
 *
 * @p cmd[i] = clamp(desired[i] + kp * (desired[i] − actual[i]), ±max).
 * Missing actual effort → pass-through desired (open-loop feedforward).
 */
std::vector<double> CorrectEffortCommand(
    const std::vector<double>& desired, const std::vector<double>& actual,
    double kp, double max_abs_effort);

/**
 * @brief Holds latest desired effort and produces corrected commands.
 *
 * Typical use: TEM EffortFeedforwardHook → SetDesired; JointState callback
 * refreshes measurement; Publish() emits Float64MultiArray to hardware.
 */
class EffortTrackingController {
 public:
  using CommandPublisher = std::function<void(const std::vector<double>&)>;
  using StateProvider = std::function<core::JointState()>;

  void SetCommandPublisher(CommandPublisher pub);
  void SetStateProvider(StateProvider provider);

  /** @brief Effort error gain (Nm per Nm). 0 = pure feedforward. */
  void SetKp(double kp) { kp_ = kp; }

  /** @brief Per-joint |cmd| clamp (Nm). 0 = no clamp. */
  void SetMaxAbsEffort(double max_abs) { max_abs_effort_ = max_abs; }

  /** @brief Cache desired feedforward efforts (Nm). */
  void SetDesired(const core::JointState& desired);

  /**
   * @brief Compute corrected command from desired + latest measurement and
   *        invoke the publisher.
   * @return true if a non-empty command was published.
   */
  bool Publish();

  /** @brief Clear desired / halt further Publish until SetDesired. */
  void Clear();

 private:
  CommandPublisher publisher_;
  StateProvider state_provider_;
  std::vector<double> desired_;
  double kp_ = 0.0;
  double max_abs_effort_ = 0.0;
  mutable std::mutex mutex_;
};

}  // namespace execution
}  // namespace manipulation
}  // namespace autonomy
