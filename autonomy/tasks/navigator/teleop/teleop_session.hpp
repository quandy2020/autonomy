/*
 * Copyright 2026 autonomy contributors
 */

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>

#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace tasks {
namespace navigator {
namespace teleop {

/** @brief In-process teleop session: velocity ingress, limits, timeout, stale-cmd watchdog. */
class TeleopSession
{
public:
  struct Limits
  {
    double max_linear_vel{0.5};
    double max_angular_vel{1.5};
    double cmd_stale_timeout_sec{0.5};
  };

  void Begin(double time_allowance_sec, Limits limits);
  void End();

  bool IsActive() const { return active_.load(); }

  void UpdateCommand(const commsgs::geometry_msgs::TwistStamped & cmd);

  /** @brief BT-assisted teleop writes the obstacle-filtered command here. */
  void SetOutputCommand(const commsgs::geometry_msgs::TwistStamped & cmd);

  commsgs::geometry_msgs::TwistStamped CurrentCommand() const;

  /** @brief Returns false when session should stop (timeout / cancel). */
  bool Tick(std::function<bool()> cancel_checker);

  double ElapsedSec() const;

  const commsgs::geometry_msgs::TwistStamped & lastRawCommand() const {
    return last_raw_cmd_;
  }

  /** @brief Operator command after limit clamp (input to assisted filter). */
  commsgs::geometry_msgs::TwistStamped RequestedCommand() const;

  double commandedLinearVel() const;
  double commandedAngularVel() const;
  double appliedLinearVel() const;
  double appliedAngularVel() const;

private:
  commsgs::geometry_msgs::TwistStamped clampCommand(
    const commsgs::geometry_msgs::TwistStamped & cmd) const;

  Limits limits_;
  double time_allowance_sec_{0.0};
  std::chrono::steady_clock::time_point start_time_{};
  std::chrono::steady_clock::time_point last_cmd_time_{};

  mutable std::mutex mutex_;
  commsgs::geometry_msgs::TwistStamped cmd_;
  commsgs::geometry_msgs::TwistStamped output_cmd_;
  commsgs::geometry_msgs::TwistStamped last_raw_cmd_;
  bool has_output_cmd_{false};

  std::atomic<bool> active_{false};
};

}  // namespace teleop
}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy
