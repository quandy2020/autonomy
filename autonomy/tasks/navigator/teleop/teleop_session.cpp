/*
 * Copyright 2026 autonomy contributors
 */

#include "autonomy/tasks/navigator/teleop/teleop_session.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy {
namespace tasks {
namespace navigator {
namespace teleop {

void TeleopSession::Begin(const double time_allowance_sec, Limits limits)
{
  std::lock_guard<std::mutex> lock(mutex_);
  limits_ = limits;
  time_allowance_sec_ = time_allowance_sec;
  start_time_ = std::chrono::steady_clock::now();
  last_cmd_time_ = start_time_;
  cmd_ = commsgs::geometry_msgs::TwistStamped{};
  output_cmd_ = commsgs::geometry_msgs::TwistStamped{};
  last_raw_cmd_ = cmd_;
  has_output_cmd_ = false;
  active_.store(true);
}

void TeleopSession::End()
{
  active_.store(false);
  std::lock_guard<std::mutex> lock(mutex_);
  cmd_ = commsgs::geometry_msgs::TwistStamped{};
  output_cmd_ = commsgs::geometry_msgs::TwistStamped{};
  has_output_cmd_ = false;
}

void TeleopSession::UpdateCommand(
  const commsgs::geometry_msgs::TwistStamped & cmd)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_raw_cmd_ = cmd;
  last_cmd_time_ = std::chrono::steady_clock::now();
  cmd_ = clampCommand(cmd);
}

void TeleopSession::SetOutputCommand(
  const commsgs::geometry_msgs::TwistStamped & cmd)
{
  std::lock_guard<std::mutex> lock(mutex_);
  output_cmd_ = cmd;
  has_output_cmd_ = true;
}

commsgs::geometry_msgs::TwistStamped TeleopSession::RequestedCommand() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return cmd_;
}

commsgs::geometry_msgs::TwistStamped TeleopSession::CurrentCommand() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (has_output_cmd_) {
    return output_cmd_;
  }
  return cmd_;
}

double TeleopSession::commandedLinearVel() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_raw_cmd_.twist.linear.x;
}

double TeleopSession::commandedAngularVel() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_raw_cmd_.twist.angular.z;
}

double TeleopSession::appliedLinearVel() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & out = has_output_cmd_ ? output_cmd_ : cmd_;
  return out.twist.linear.x;
}

double TeleopSession::appliedAngularVel() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & out = has_output_cmd_ ? output_cmd_ : cmd_;
  return out.twist.angular.z;
}

double TeleopSession::ElapsedSec() const
{
  const auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(now - start_time_).count();
}

bool TeleopSession::Tick(std::function<bool()> cancel_checker)
{
  if (!active_.load()) {
    return false;
  }
  if (cancel_checker && cancel_checker()) {
    return false;
  }
  if (time_allowance_sec_ > 0.0 && ElapsedSec() > time_allowance_sec_) {
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  if (limits_.cmd_stale_timeout_sec > 0.0) {
    const double since_cmd =
      std::chrono::duration<double>(now - last_cmd_time_).count();
    if (since_cmd > limits_.cmd_stale_timeout_sec) {
      cmd_ = commsgs::geometry_msgs::TwistStamped{};
      if (has_output_cmd_) {
        output_cmd_ = commsgs::geometry_msgs::TwistStamped{};
      }
    }
  }
  return true;
}

commsgs::geometry_msgs::TwistStamped TeleopSession::clampCommand(
  const commsgs::geometry_msgs::TwistStamped & cmd) const
{
  auto out = cmd;
  if (limits_.max_linear_vel > 0.0) {
    const float max_v = static_cast<float>(limits_.max_linear_vel);
    out.twist.linear.x = std::clamp(out.twist.linear.x, -max_v, max_v);
  }
  if (limits_.max_angular_vel > 0.0) {
    const float max_w = static_cast<float>(limits_.max_angular_vel);
    out.twist.angular.z = std::clamp(out.twist.angular.z, -max_w, max_w);
  }
  return out;
}

}  // namespace teleop
}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy
