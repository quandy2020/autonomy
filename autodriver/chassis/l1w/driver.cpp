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
 * @brief GENISOM L1-W ChassisDriver — full HighLevel API (implementation).
 */

#include "chassis/l1w/driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "chassis/backend_register.hpp"
#include "chassis/convert.hpp"
#include "chassis/tool_command.hpp"
#include "autodriver/driver_params.hpp"
#include "autolink/common/log.hpp"
#include <automsgs/msgs/vehicle_msgs/robot_event_type.pb.h>

#if defined(AUTODRIVER_HAVE_GENISOM_L1W)
#include "zsl-1w/highlevel.h"
#endif

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

double Clamp(double value, double limit) {
  if (limit <= 0.0) {
    return value;
  }
  return std::clamp(value, -limit, limit);
}

std::string ToLower(std::string text) {
  for (char& c : text) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return text;
}

bool ParseAttitudeValue(const std::string& text, float* roll, float* pitch,
                        float* yaw, float* height) {
  if (roll == nullptr || pitch == nullptr || yaw == nullptr ||
      height == nullptr) {
    return false;
  }
  *roll = *pitch = *yaw = *height = 0.0f;
  if (text.empty()) {
    return true;
  }
  std::stringstream ss(text);
  std::string part;
  float values[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  int count = 0;
  while (count < 4 && std::getline(ss, part, ',')) {
    values[count++] = static_cast<float>(std::strtod(part.c_str(), nullptr));
  }
  *roll = values[0];
  *pitch = values[1];
  *yaw = values[2];
  *height = values[3];
  return count > 0;
}

enum class GaitMode {
  kStand = 0,
  kMove = 1,   ///< HighLevel::move (wheel / normal)
  kCrawl = 2,  ///< HighLevel::crawl
};

class L1wChassisDriver final : public ChassisDriver {
public:
  L1wChassisDriver(ChassisId id, hardware::DriverParams params)
  : id_(std::move(id)), params_(std::move(params)) {
    host_ = hardware::GetString(params_, "host");
    if (host_.empty()) {
      host_ = hardware::GetString(params_, "dog_ip", "192.168.234.1");
    }
    local_ip_ = hardware::GetString(params_, "local_ip", "0.0.0.0");
    local_port_ = hardware::ParseInt(params_, "local_port", 43988);
    max_vx_ = hardware::ParseDouble(params_, "max_vx", 0.5);
    max_vy_ = hardware::ParseDouble(params_, "max_vy", 0.5);
    max_wz_ = hardware::ParseDouble(params_, "max_wz", 0.8);
    allow_lateral_ = hardware::ParseBool(params_, "allow_lateral", true);
    auto_stand_ = hardware::ParseBool(params_, "auto_stand", true);
    lie_on_stop_ = hardware::ParseBool(params_, "lie_on_stop", true);
    simulate_ = hardware::ParseBool(params_, "simulate", false) ||
                hardware::ParseBool(params_, "dry_run", false);

    const std::string default_gait =
        ToLower(hardware::GetString(params_, "default_gait", "move"));
    gait_ = (default_gait == "crawl") ? GaitMode::kCrawl : GaitMode::kMove;

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
#if defined(AUTODRIVER_HAVE_GENISOM_L1W)
    if (!simulate_) {
      sdk_ = std::make_unique<mc_sdk::zsl_1w::HighLevel>();
      sdk_->initRobot(local_ip_, local_port_, host_);
      if (!sdk_->checkConnect()) {
        AERROR << "l1w: SDK checkConnect failed host=" << host_
               << " local=" << local_ip_ << ":" << local_port_;
        sdk_.reset();
        return false;
      }
      if (auto_stand_) {
        if (!CallStandUpLocked()) {
          sdk_.reset();
          return false;
        }
      }
    }
#else
    if (!simulate_) {
      AERROR << "l1w: built without GenisomL1w SDK; set params.simulate=true "
                "or install SDK (GenisomL1w_ROOT / GENISOM_L1W_SDK_ROOT)";
      return false;
    }
#endif
    running_ = true;
    last_integrate_ns_ = ReadSteadyTimeNanoseconds();
    state_.set_motion_enabled(true);
    if (simulate_ && auto_stand_) {
      standing_ = true;
      gait_ = GaitMode::kMove;
    }
    AINFO << "l1w chassis started id=" << id_ << " host=" << host_
          << " standing=" << (standing_ ? "true" : "false")
          << " gait=" << GaitName(gait_)
          << " lateral=" << (allow_lateral_ ? "on" : "off")
          << " simulate=" << (simulate_ ? "true" : "false");
    return SendTwistLocked(0.0, 0.0, 0.0);
  }

  void Stop() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
      (void)CancelCrawlLocked();
      (void)SendTwistLocked(0.0, 0.0, 0.0);
#if defined(AUTODRIVER_HAVE_GENISOM_L1W)
      if (sdk_ && !simulate_ && lie_on_stop_) {
        (void)sdk_->lieDown();
        standing_ = false;
      }
#endif
    }
    command_.Clear();
    running_ = false;
    standing_ = false;
    crawling_ = false;
#if defined(AUTODRIVER_HAVE_GENISOM_L1W)
    sdk_.reset();
#endif
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
    double vx =
        command.has_twist() ? command.twist().linear().x() : 0.0;
    double vy =
        command.has_twist() ? command.twist().linear().y() : 0.0;
    double wz =
        command.has_twist() ? command.twist().angular().z() : 0.0;
    if (!allow_lateral_) {
      vy = 0.0;
    }
    vx = Clamp(vx, max_vx_);
    vy = Clamp(vy, max_vy_);
    wz = Clamp(wz, max_wz_);

    const bool nonzero =
        std::abs(vx) > 1e-6 || std::abs(vy) > 1e-6 || std::abs(wz) > 1e-6;
    if (nonzero && !standing_ && auto_stand_) {
      if (!CallStandUpLocked()) {
        return false;
      }
    }
    if (nonzero && !standing_) {
      AWARN << "l1w: reject twist while not standing";
      return false;
    }
    if (gait_ == GaitMode::kStand && nonzero) {
      // Stand intent: hold pose; ignore non-zero twist.
      return SendTwistLocked(0.0, 0.0, 0.0);
    }
    return SendTwistLocked(vx, vy, wz);
  }

  bool ApplyLocomotionIntent(LocomotionIntent intent) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
      return false;
    }
    switch (intent) {
      case LocomotionIntent::kStand:
        (void)CancelCrawlLocked();
        (void)SendTwistLocked(0.0, 0.0, 0.0);
        gait_ = GaitMode::kStand;
        return CallStandUpLocked();
      case LocomotionIntent::kWheel:
        (void)CancelCrawlLocked();
        gait_ = GaitMode::kMove;
        return CallStandUpLocked();
      case LocomotionIntent::kWalk:
        gait_ = GaitMode::kCrawl;
        return CallStandUpLocked();
      case LocomotionIntent::kUnspecified:
      default:
        return true;
    }
  }

  bool ApplyToolCommand(const ToolCommand& command) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
      return false;
    }
    const std::string name = ToLower(command.name);
    if (name == "lie" || name == "liedown" || name == "lie_down" ||
        name == "sit") {
      (void)CancelCrawlLocked();
      (void)SendTwistLocked(0.0, 0.0, 0.0);
#if defined(AUTODRIVER_HAVE_GENISOM_L1W)
      if (sdk_ && !simulate_) {
        const auto ret = sdk_->lieDown();
        if (ret != 0) {
          AWARN << "l1w: lieDown ret=" << ret;
          return false;
        }
      }
#endif
      standing_ = false;
      gait_ = GaitMode::kStand;
      AINFO << "l1w: lieDown";
      return true;
    }
    if (name == "passive" || name == "damp" || name == "damping") {
      (void)CancelCrawlLocked();
      (void)SendTwistLocked(0.0, 0.0, 0.0);
#if defined(AUTODRIVER_HAVE_GENISOM_L1W)
      if (sdk_ && !simulate_) {
        const auto ret = sdk_->passive();
        if (ret != 0) {
          AWARN << "l1w: passive ret=" << ret;
          return false;
        }
      }
#endif
      standing_ = false;
      state_.set_motion_enabled(false);
      AINFO << "l1w: passive";
      return true;
    }
    if (name == "stand" || name == "standup") {
      return CallStandUpLocked();
    }
    if (name == "cancel_crawl" || name == "cancelcrawl") {
      return CancelCrawlLocked();
    }
    if (name == "attitude" || name == "attitude_control") {
      float roll = 0.0f;
      float pitch = 0.0f;
      float yaw = 0.0f;
      float height = 0.0f;
      if (!ParseAttitudeValue(command.value, &roll, &pitch, &yaw, &height)) {
        return false;
      }
      if (!standing_ && !CallStandUpLocked()) {
        return false;
      }
#if defined(AUTODRIVER_HAVE_GENISOM_L1W)
      if (sdk_ && !simulate_) {
        const auto ret = sdk_->attitudeControl(roll, pitch, yaw, height);
        if (ret != 0) {
          AWARN << "l1w: attitudeControl ret=" << ret;
          return false;
        }
      }
#endif
      AINFO << "l1w: attitudeControl rpyh=(" << roll << "," << pitch << ","
            << yaw << "," << height << ")";
      return true;
    }
    AWARN << "l1w: unsupported tool " << command.name;
    return false;
  }

  bool ReadChassisState(ChassisState* state) override {
    if (state == nullptr) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    SampleHardwareLocked();
    IntegrateOdometryLocked(ReadSteadyTimeNanoseconds());
    *state = state_;
    return true;
  }

  bool TriggerEmergencyStop() override {
    std::lock_guard<std::mutex> lock(mutex_);
    command_.Clear();
    state_.set_motion_enabled(false);
    (void)CancelCrawlLocked();
    (void)SendTwistLocked(0.0, 0.0, 0.0);
#if defined(AUTODRIVER_HAVE_GENISOM_L1W)
    if (sdk_ && !simulate_) {
      (void)sdk_->passive();
    }
#endif
    standing_ = false;
    ChassisEvent event;
    FillTimestampFromNanoseconds(event.mutable_timestamp(),
                                 ReadSteadyTimeNanoseconds());
    event.set_type(
        ::automsgs::msgs::vehicle_msgs::ROBOT_EVENT_EMERGENCY_STOP);
    event.set_severity(::automsgs::msgs::vehicle_msgs::EVENT_SEVERITY_ERROR);
    event.set_message("l1w chassis emergency stop (passive)");
    EmitChassisEvent(event);
    return true;
  }

private:
  static const char* GaitName(GaitMode gait) {
    switch (gait) {
      case GaitMode::kCrawl:
        return "crawl";
      case GaitMode::kStand:
        return "stand";
      case GaitMode::kMove:
      default:
        return "move";
    }
  }

  bool CallStandUpLocked() {
#if defined(AUTODRIVER_HAVE_GENISOM_L1W)
    if (sdk_ && !simulate_) {
      const auto ret = sdk_->standUp();
      if (ret != 0) {
        AERROR << "l1w: standUp failed ret=" << ret;
        return false;
      }
    }
#endif
    standing_ = true;
    state_.set_motion_enabled(true);
    AINFO << "l1w: standUp ok";
    return true;
  }

  bool CancelCrawlLocked() {
    if (!crawling_) {
      return true;
    }
#if defined(AUTODRIVER_HAVE_GENISOM_L1W)
    if (sdk_ && !simulate_) {
      sdk_->cancelCrawl();
    }
#endif
    crawling_ = false;
    AINFO << "l1w: cancelCrawl";
    return true;
  }

  bool SendTwistLocked(double vx, double vy, double wz) {
    last_vx_ = vx;
    last_vy_ = vy;
    last_wz_ = wz;
    if (simulate_) {
      if (gait_ == GaitMode::kCrawl &&
          (std::abs(vx) > 1e-9 || std::abs(vy) > 1e-9 || std::abs(wz) > 1e-9)) {
        crawling_ = true;
      }
      return true;
    }
#if defined(AUTODRIVER_HAVE_GENISOM_L1W)
    if (!sdk_) {
      return false;
    }
    const float fvx = static_cast<float>(vx);
    const float fvy = static_cast<float>(vy);
    const float fwz = static_cast<float>(wz);
    std::uint32_t ret = 0;
    if (gait_ == GaitMode::kCrawl) {
      ret = sdk_->crawl(fvx, fvy, fwz);
      crawling_ = true;
    } else {
      if (crawling_) {
        sdk_->cancelCrawl();
        crawling_ = false;
      }
      ret = sdk_->move(fvx, fvy, fwz);
    }
    if (ret != 0) {
      AWARN << "l1w: " << GaitName(gait_) << " failed ret=" << ret
            << " v=(" << vx << "," << vy << "," << wz << ")";
      return false;
    }
    return true;
#else
    return false;
#endif
  }

  void SampleHardwareLocked() {
#if defined(AUTODRIVER_HAVE_GENISOM_L1W)
    if (!sdk_ || simulate_) {
      return;
    }
    if (!sdk_->checkConnect()) {
      AWARN << "l1w: checkConnect lost";
      ChassisEvent event;
      FillTimestampFromNanoseconds(event.mutable_timestamp(),
                                   ReadSteadyTimeNanoseconds());
      event.set_type(::automsgs::msgs::vehicle_msgs::ROBOT_EVENT_FAULT);
      event.set_severity(
          ::automsgs::msgs::vehicle_msgs::EVENT_SEVERITY_ERROR);
      event.set_message("l1w SDK connection lost");
      EmitChassisEvent(event);
    }

    const auto power = sdk_->getBatteryPower();
    if (power <= 100) {
      state_.set_battery_percent(static_cast<float>(power));
    }

    const auto quat = sdk_->getQuaternion();
    const auto pos = sdk_->getPosition();
    if (pos.size() >= 2) {
      auto* pose = state_.mutable_pose()->mutable_pose();
      pose->mutable_position()->set_x(pos[0]);
      pose->mutable_position()->set_y(pos[1]);
      pose->mutable_position()->set_z(pos.size() > 2 ? pos[2] : 0.0);
      if (quat.size() >= 4) {
        // Docs: [w, x, y, z]
        pose->mutable_orientation()->set_w(quat[0]);
        pose->mutable_orientation()->set_x(quat[1]);
        pose->mutable_orientation()->set_y(quat[2]);
        pose->mutable_orientation()->set_z(quat[3]);
      } else {
        const auto rpy = sdk_->getRPY();
        const double yaw =
            rpy.size() >= 3 ? static_cast<double>(rpy[2]) : 0.0;
        const double half = 0.5 * yaw;
        pose->mutable_orientation()->set_x(0.0);
        pose->mutable_orientation()->set_y(0.0);
        pose->mutable_orientation()->set_z(std::sin(half));
        pose->mutable_orientation()->set_w(std::cos(half));
      }
      state_.mutable_pose()->mutable_header()->set_frame_id(
          state_.global_frame().empty() ? "odom" : state_.global_frame());
      skip_integrate_ = true;
    }

    auto* twist = state_.mutable_twist()->mutable_twist();
    const auto body_vel = sdk_->getBodyVelocity();
    if (body_vel.size() >= 1) {
      twist->mutable_linear()->set_x(body_vel[0]);
      twist->mutable_linear()->set_y(body_vel.size() > 1 ? body_vel[1] : 0.0);
      twist->mutable_linear()->set_z(body_vel.size() > 2 ? body_vel[2] : 0.0);
    } else {
      const auto world_vel = sdk_->getWorldVelocity();
      if (world_vel.size() >= 2) {
        twist->mutable_linear()->set_x(world_vel[0]);
        twist->mutable_linear()->set_y(world_vel[1]);
        twist->mutable_linear()->set_z(world_vel.size() > 2 ? world_vel[2]
                                                            : 0.0);
      }
    }
    const auto gyro = sdk_->getBodyGyro();
    if (gyro.size() >= 3) {
      twist->mutable_angular()->set_x(gyro[0]);
      twist->mutable_angular()->set_y(gyro[1]);
      twist->mutable_angular()->set_z(gyro[2]);
    } else if (body_vel.size() > 2) {
      twist->mutable_angular()->set_z(body_vel[2]);
    }

    // Encode ctrl mode into map_name tag (no schema field for ctrlmode).
    const auto mode = sdk_->getCurrentCtrlmode();
    // 0 damp, 1 stand, 3 move — stash as map_name for debug without schema change.
    if (mode == 0) {
      state_.set_map_name("ctrl:passive");
    } else if (mode == 1) {
      state_.set_map_name("ctrl:stand");
      standing_ = true;
    } else if (mode == 3) {
      state_.set_map_name("ctrl:move");
      standing_ = true;
    } else {
      state_.set_map_name("ctrl:" + std::to_string(mode));
    }
#else
    (void)0;
#endif
  }

  void IntegrateOdometryLocked(std::uint64_t now_ns) {
    if (skip_integrate_) {
      skip_integrate_ = false;
      last_integrate_ns_ = now_ns;
      FillTimestampFromNanoseconds(state_.mutable_timestamp(), now_ns);
      return;
    }
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

    const double vx = state_.motion_enabled() ? last_vx_ : 0.0;
    const double vy = state_.motion_enabled() ? last_vy_ : 0.0;
    const double wz = state_.motion_enabled() ? last_wz_ : 0.0;

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    if (state_.has_pose() && state_.pose().has_pose()) {
      if (state_.pose().pose().has_position()) {
        x = state_.pose().pose().position().x();
        y = state_.pose().pose().position().y();
      }
      if (state_.pose().pose().has_orientation()) {
        const auto& q = state_.pose().pose().orientation();
        yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                         1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
      }
    }
    x += (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt;
    y += (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt;
    yaw += wz * dt;

    auto* pose = state_.mutable_pose()->mutable_pose();
    pose->mutable_position()->set_x(x);
    pose->mutable_position()->set_y(y);
    pose->mutable_position()->set_z(0.0);
    const double half = 0.5 * yaw;
    pose->mutable_orientation()->set_x(0.0);
    pose->mutable_orientation()->set_y(0.0);
    pose->mutable_orientation()->set_z(std::sin(half));
    pose->mutable_orientation()->set_w(std::cos(half));
    state_.mutable_pose()->mutable_header()->set_frame_id(
        state_.global_frame().empty() ? "odom" : state_.global_frame());

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
  std::string host_;
  std::string local_ip_;
  int local_port_ = 43988;
  double max_vx_ = 0.5;
  double max_vy_ = 0.5;
  double max_wz_ = 0.8;
  bool allow_lateral_ = true;
  bool auto_stand_ = true;
  bool lie_on_stop_ = true;
  bool simulate_ = false;
  GaitMode gait_ = GaitMode::kMove;
  mutable std::mutex mutex_;
  bool running_ = false;
  bool standing_ = false;
  bool crawling_ = false;
  bool skip_integrate_ = false;
  ChassisCommand command_;
  ChassisState state_;
  double last_vx_ = 0.0;
  double last_vy_ = 0.0;
  double last_wz_ = 0.0;
  std::uint64_t last_integrate_ns_ = 0;
#if defined(AUTODRIVER_HAVE_GENISOM_L1W)
  std::unique_ptr<mc_sdk::zsl_1w::HighLevel> sdk_;
#endif
};

}  // namespace

ChassisDriver* CreateL1wChassisDriver(const ChassisId& id,
                                      const hardware::DriverParams& params) {
  return new L1wChassisDriver(id, params);
}

}  // namespace chassis
}  // namespace autodriver

REGISTER_CHASSIS_BACKEND(l1w, "l1w", autodriver::chassis::CreateL1wChassisDriver,
                         "genisom", "zsibot", "zsl-1w", "l1-w");
