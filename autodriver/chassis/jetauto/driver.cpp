/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file driver.cpp
 * @brief JetAuto ChassisDriver (implementation).
 */

#include "chassis/jetauto/driver.hpp"

#include <chrono>
#include <cmath>
#include <mutex>

#include "chassis/backend_register.hpp"
#include "chassis/convert.hpp"
#include "chassis/jetauto/kinematics.hpp"
#include "chassis/jetauto/rrc_protocol.hpp"
#include "autodriver/common/serial_port.hpp"
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

double ExtractYawFromState(const ChassisState& state) {
  if (!state.has_pose() || !state.pose().has_pose() ||
      !state.pose().pose().has_orientation()) {
    return 0.0;
  }
  const auto& q = state.pose().pose().orientation();
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

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

class JetAutoChassisDriver final : public ChassisDriver {
public:
  JetAutoChassisDriver(ChassisId id, hardware::DriverParams params)
  : id_(std::move(id)), params_(std::move(params)) {
    device_ = hardware::GetString(params_, "device");
    if (device_.empty()) {
      device_ = hardware::GetString(params_, "port", "/dev/ttyACM0");
    }
    baud_ = hardware::ParseInt(params_, "baud", 1000000);
    if (baud_ <= 0) {
      baud_ = hardware::ParseInt(params_, "baudrate", 1000000);
    }
    simulate_ = hardware::ParseBool(params_, "simulate", false) ||
                hardware::ParseBool(params_, "dry_run", false);

    geometry_.wheelbase =
        hardware::ParseDouble(params_, "wheelbase", geometry_.wheelbase);
    geometry_.track_width =
        hardware::ParseDouble(params_, "track_width", geometry_.track_width);
    geometry_.wheel_diameter = hardware::ParseDouble(
        params_, "wheel_diameter", geometry_.wheel_diameter);
    max_rps_ = hardware::ParseDouble(params_, "max_rps", 0.0);

    const bool prefer_diff =
        hardware::ParseBool(params_, "prefer_differential", false);
    drive_mode_ = jetauto::ParseDriveMode(
        hardware::GetString(params_, "drive_mode"), prefer_diff);

    state_.set_global_frame("odom");
    state_.set_motion_enabled(true);
    state_.set_battery_percent(static_cast<float>(
        hardware::ParseDouble(params_, "battery_soc", 1.0) * 100.0));
  }

  const ChassisId& GetChassisId() const override { return id_; }

  bool Start() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
      return true;
    }
    if (!simulate_) {
      if (!serial_.Open(device_, baud_)) {
        AERROR << "jetauto: open " << device_ << " baud=" << baud_
               << " failed: " << serial_.last_error();
        return false;
      }
    }
    running_ = true;
    last_integrate_ns_ = ReadSteadyTimeNanoseconds();
    state_.set_motion_enabled(true);
    AINFO << "jetauto chassis started id=" << id_ << " device=" << device_
          << " baud=" << baud_
          << " drive_mode="
          << (drive_mode_ == jetauto::DriveMode::kMecanum ? "mecanum"
                                                          : "differential")
          << " simulate=" << (simulate_ ? "true" : "false");
    return SendMotorRpsLocked({0.f, 0.f, 0.f, 0.f});
  }

  void Stop() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
      (void)SendMotorRpsLocked({0.f, 0.f, 0.f, 0.f});
    }
    command_.Clear();
    running_ = false;
    if (serial_.IsOpen()) {
      serial_.Close();
    }
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
    const double vx =
        command.has_twist() ? command.twist().linear().x() : 0.0;
    const double vy =
        command.has_twist() ? command.twist().linear().y() : 0.0;
    const double wz =
        command.has_twist() ? command.twist().angular().z() : 0.0;
    const auto rps = jetauto::TwistToMotorRps(vx, vy, wz, geometry_,
                                              drive_mode_, max_rps_);
    return SendMotorRpsLocked(rps);
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
    (void)SendMotorRpsLocked({0.f, 0.f, 0.f, 0.f});
    ChassisEvent event;
    FillTimestampFromNanoseconds(event.mutable_timestamp(),
                                 ReadSteadyTimeNanoseconds());
    event.set_type(
        ::automsgs::msgs::vehicle_msgs::ROBOT_EVENT_EMERGENCY_STOP);
    event.set_severity(::automsgs::msgs::vehicle_msgs::EVENT_SEVERITY_ERROR);
    event.set_message("jetauto chassis emergency stop");
    EmitChassisEvent(event);
    return true;
  }

private:
  bool SendMotorRpsLocked(const std::array<float, 4>& rps) {
    last_rps_ = rps;
    if (simulate_) {
      return true;
    }
    if (!serial_.IsOpen()) {
      return false;
    }
    const auto packet = jetauto::BuildSetFourMotorsPacket(rps.data());
    if (packet.empty()) {
      return false;
    }
    if (!serial_.Write(packet.data(), packet.size())) {
      AERROR << "jetauto: serial write failed: " << serial_.last_error();
      return false;
    }
    return true;
  }

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
    const double vy =
        state_.motion_enabled() && command_.has_twist() &&
                drive_mode_ == jetauto::DriveMode::kMecanum
            ? command_.twist().linear().y()
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
    x += (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt;
    y += (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt;
    yaw += wz * dt;
    WritePlanarPose(&state_, x, y, yaw);

    auto* twist = state_.mutable_twist()->mutable_twist();
    twist->mutable_linear()->set_x(vx);
    twist->mutable_linear()->set_y(vy);
    twist->mutable_angular()->set_z(wz);
    if (command_.has_header()) {
      *state_.mutable_twist()->mutable_header() = command_.header();
    }

    FillTimestampFromNanoseconds(state_.mutable_timestamp(), now_ns);
  }

  ChassisId id_;
  hardware::DriverParams params_;
  std::string device_;
  int baud_ = 1000000;
  bool simulate_ = false;
  double max_rps_ = 0.0;
  jetauto::ChassisGeometry geometry_;
  jetauto::DriveMode drive_mode_ = jetauto::DriveMode::kMecanum;
  io::SerialPort serial_;
  mutable std::mutex mutex_;
  bool running_ = false;
  ChassisCommand command_;
  ChassisState state_;
  std::array<float, 4> last_rps_{};
  std::uint64_t last_integrate_ns_ = 0;
};

}  // namespace

ChassisDriver* CreateJetAutoChassisDriver(
    const ChassisId& id, const hardware::DriverParams& params) {
  return new JetAutoChassisDriver(id, params);
}

}  // namespace chassis
}  // namespace autodriver

REGISTER_CHASSIS_BACKEND(jetauto, "jetauto",
                         autodriver::chassis::CreateJetAutoChassisDriver,
                         "hiwonder");
