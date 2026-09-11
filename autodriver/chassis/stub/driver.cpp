/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file driver.cpp
 * @brief Stub chassis backend (software differential odom, no hardware)
 *        (implementation).
 */

#include "chassis/stub/driver.hpp"

#include <chrono>
#include <cmath>
#include <mutex>

#include "chassis/backend_register.hpp"
#include "chassis/convert.hpp"
#include "chassis/operational_mode.hpp"
#include "chassis/tool_command.hpp"
#include "autodriver/driver_params.hpp"
#include "autolink/common/log.hpp"
#include <automsgs/msgs/vehicle_msgs/robot_event_type.pb.h>

namespace autodriver {
namespace chassis {
namespace {

std::uint64_t ReadSteadyTimeNanoseconds() {
  using clock = std::chrono::steady_clock;
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          clock::now().time_since_epoch())
          .count());
}

/**
 * @brief Planar yaw (rad) from RobotState orientation quaternion.
 * @param[in] state Chassis state carrying pose orientation.
 * @return Planar yaw in radians, or 0 when orientation is missing.
 */
double ExtractYawFromState(const ChassisState& state) {
  if (!state.has_pose() || !state.pose().has_pose() ||
      !state.pose().pose().has_orientation()) {
    return 0.0;
  }
  const auto& q = state.pose().pose().orientation();
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

/**
 * @brief Write SE(2) pose (x, y, yaw) into RobotState.pose.
 * @param[out] state Chassis state whose pose is overwritten.
 * @param[in] x Planar x position (m).
 * @param[in] y Planar y position (m).
 * @param[in] yaw Planar yaw (rad).
 */
void WritePlanarPose(ChassisState* state, double x, double y, double yaw) {
  auto* pose = state->mutable_pose()->mutable_pose();
  pose->mutable_position()->set_x(x);
  pose->mutable_position()->set_y(y);
  pose->mutable_position()->set_z(0.0);
  const double half = 0.5 * yaw;
  pose->mutable_orientation()->set_x(0.0);
  pose->mutable_orientation()->set_y(0.0);
  pose->mutable_orientation()->set_z(std::sin(half));
  pose->mutable_orientation()->set_w(std::cos(half));
  state->mutable_pose()->mutable_header()->set_frame_id(
      state->global_frame().empty() ? "odom" : state->global_frame());
}

class StubChassisDriver final : public ChassisDriver {
public:
  StubChassisDriver(ChassisId id, hardware::DriverParams params)
  : id_(std::move(id)), params_(std::move(params)) {
    state_.set_global_frame("odom");
    state_.set_motion_enabled(true);
    state_.set_battery_percent(static_cast<float>(
        hardware::ParseDouble(params_, "battery_soc", 1.0) * 100.0));
  }

  const ChassisId& GetChassisId() const override { return id_; }

  bool Start() override {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = true;
    last_integrate_ns_ = ReadSteadyTimeNanoseconds();
    state_.set_motion_enabled(true);
    AINFO << "stub chassis started id=" << id_;
    return true;
  }

  void Stop() override {
    std::lock_guard<std::mutex> lock(mutex_);
    command_.Clear();
    running_ = false;
  }

  bool IsRunning() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return running_;
  }

  bool ApplyVelocityCommand(const ChassisCommand& command) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || !state_.motion_enabled()) {
      return false;
    }
    command_ = command;
    return true;
  }

  bool ApplyLocomotionIntent(LocomotionIntent intent) override {
    std::lock_guard<std::mutex> lock(mutex_);
    intent_ = intent;
    AINFO << "stub chassis intent=" << LocomotionIntentToString(intent);
    return true;
  }

  bool ApplyToolCommand(const ToolCommand& command) override {
    std::lock_guard<std::mutex> lock(mutex_);
    last_tool_ = command.name + "=" + command.value;
    AINFO << "stub chassis tool " << last_tool_
          << " enable=" << (command.enable ? "true" : "false");
    return true;
  }

  bool ReadChassisState(ChassisState* state) override {
    if (state == nullptr) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    IntegrateOdometryLocked(ReadSteadyTimeNanoseconds());
    *state = state_;
    return true;
  }

  bool TriggerEmergencyStop() override {
    std::lock_guard<std::mutex> lock(mutex_);
    command_.Clear();
    state_.set_motion_enabled(false);
    ChassisEvent event;
    FillTimestampFromNanoseconds(event.mutable_timestamp(),
                                 ReadSteadyTimeNanoseconds());
    event.set_type(
        ::automsgs::msgs::vehicle_msgs::ROBOT_EVENT_EMERGENCY_STOP);
    event.set_severity(::automsgs::msgs::vehicle_msgs::EVENT_SEVERITY_ERROR);
    event.set_message("stub chassis emergency stop");
    EmitChassisEvent(event);
    return true;
  }

private:
  /**
   * @brief Integrate differential-drive odometry from the last velocity command.
   * @param[in] now_ns Current steady-clock time in nanoseconds.
   * @pre Caller holds mutex_.
   */
  void IntegrateOdometryLocked(std::uint64_t now_ns) {
    if (!running_ || last_integrate_ns_ == 0) {
      last_integrate_ns_ = now_ns;
      return;
    }
    const double dt =
        static_cast<double>(now_ns - last_integrate_ns_) * 1e-9;
    last_integrate_ns_ = now_ns;
    if (dt <= 0.0 || dt > 1.0) {
      return;
    }

    const double vx =
        state_.motion_enabled() && command_.has_twist()
            ? command_.twist().linear().x()
            : 0.0;
    const double wz =
        state_.motion_enabled() && command_.has_twist()
            ? command_.twist().angular().z()
            : 0.0;

    double x = 0.0;
    double y = 0.0;
    double yaw = ExtractYawFromState(state_);
    if (state_.has_pose() && state_.pose().has_pose() &&
        state_.pose().pose().has_position()) {
      x = state_.pose().pose().position().x();
      y = state_.pose().pose().position().y();
    }
    x += vx * std::cos(yaw) * dt;
    y += vx * std::sin(yaw) * dt;
    yaw += wz * dt;
    WritePlanarPose(&state_, x, y, yaw);

    auto* twist = state_.mutable_twist()->mutable_twist();
    twist->mutable_linear()->set_x(vx);
    twist->mutable_linear()->set_y(0.0);
    twist->mutable_angular()->set_z(wz);
    *state_.mutable_twist()->mutable_header() = command_.header();

    FillTimestampFromNanoseconds(state_.mutable_timestamp(), now_ns);
    state_.set_battery_percent(static_cast<float>(
        hardware::ParseDouble(params_, "battery_soc", 1.0) * 100.0));
  }

  ChassisId id_;
  hardware::DriverParams params_;
  mutable std::mutex mutex_;
  bool running_ = false;
  ChassisCommand command_;
  ChassisState state_;
  LocomotionIntent intent_{LocomotionIntent::kUnspecified};
  std::string last_tool_;
  std::uint64_t last_integrate_ns_ = 0;
};

}  // namespace

ChassisDriver* CreateStubChassisDriver(
    const ChassisId& id, const hardware::DriverParams& params) {
  return new StubChassisDriver(id, params);
}

}  // namespace chassis
}  // namespace autodriver

REGISTER_CHASSIS_BACKEND(stub, "stub",
                         autodriver::chassis::CreateStubChassisDriver,
                         "sim", "fake");
