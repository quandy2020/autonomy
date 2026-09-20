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
 *
 * Maps mc_sdk::zsl_1w::HighLevel (thirdparty/zsl1w) onto ChassisDriver:
 * standUp/lieDown/passive, move/crawl/climb (+ cancel*), shakeHand,
 * rearSquat, attitudeControl, and full state sampling.
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
#include <thread>
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

std::string FormatFloatCsv(const std::vector<float>& values, std::size_t max_n) {
  std::ostringstream oss;
  const std::size_t n = std::min(values.size(), max_n);
  for (std::size_t i = 0; i < n; ++i) {
    if (i > 0) {
      oss << ',';
    }
    oss << values[i];
  }
  return oss.str();
}

enum class GaitMode {
  kStand = 0,
  kMove = 1,   ///< HighLevel::move
  kCrawl = 2,  ///< HighLevel::crawl
  kClimb = 3,  ///< HighLevel::climb
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
    sample_joints_ = hardware::ParseBool(params_, "sample_joints", true);
    simulate_ = hardware::ParseBool(params_, "simulate", false) ||
                hardware::ParseBool(params_, "dry_run", false);

    const std::string default_gait =
        ToLower(hardware::GetString(params_, "default_gait", "move"));
    if (default_gait == "crawl" || default_gait == "walk") {
      gait_ = GaitMode::kCrawl;
    } else if (default_gait == "climb") {
      gait_ = GaitMode::kClimb;
    } else if (default_gait == "stand") {
      gait_ = GaitMode::kStand;
    } else {
      gait_ = GaitMode::kMove;
    }

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
      // HighLevel UDP handshake needs a short settle; immediate checkConnect
      // often returns false even when the robot is reachable (onboard Firefly).
      const int connect_timeout_ms =
          hardware::ParseInt(params_, "connect_timeout_ms", 3000);
      const int connect_poll_ms =
          hardware::ParseInt(params_, "connect_poll_ms", 200);
      bool connected = false;
      const auto deadline =
          std::chrono::steady_clock::now() +
          std::chrono::milliseconds(std::max(connect_timeout_ms, 0));
      while (std::chrono::steady_clock::now() < deadline) {
        if (sdk_->checkConnect()) {
          connected = true;
          break;
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(std::max(connect_poll_ms, 50)));
      }
      if (!connected) {
        AERROR << "l1w: SDK checkConnect failed host=" << host_
               << " local=" << local_ip_ << ":" << local_port_
               << " timeout_ms=" << connect_timeout_ms;
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
                "or install SDK (GenisomL1w_ROOT / thirdparty/zsl1w)";
      return false;
    }
#endif
    running_ = true;
    last_integrate_ns_ = ReadSteadyTimeNanoseconds();
    state_.set_motion_enabled(true);
    if (simulate_ && auto_stand_) {
      standing_ = true;
      if (gait_ == GaitMode::kStand) {
        gait_ = GaitMode::kMove;
      }
    }
    AINFO << "l1w chassis started id=" << id_ << " host=" << host_
          << " standing=" << (standing_ ? "true" : "false")
          << " gait=" << GaitName(gait_)
          << " lateral=" << (allow_lateral_ ? "on" : "off")
          << " simulate=" << (simulate_ ? "true" : "false");
    // Zero-velocity stop; do not fail Start if the SDK still rejects move
    // right after standUp (state machine settle race).
    if (!SendTwistLocked(0.0, 0.0, 0.0)) {
      AWARN << "l1w: initial zero twist ignored (still standing)";
    }
    return true;
  }

  void Stop() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
      (void)CancelSpecialGaitsLocked();
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
    climbing_ = false;
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
        (void)CancelSpecialGaitsLocked();
        (void)SendTwistLocked(0.0, 0.0, 0.0);
        gait_ = GaitMode::kStand;
        return CallStandUpLocked();
      case LocomotionIntent::kWheel:
        (void)CancelSpecialGaitsLocked();
        gait_ = GaitMode::kMove;
        return CallStandUpLocked();
      case LocomotionIntent::kWalk:
        (void)CancelClimbLocked();
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
      (void)CancelSpecialGaitsLocked();
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
      (void)CancelSpecialGaitsLocked();
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
    if (name == "cancel_climb" || name == "cancelclimb") {
      return CancelClimbLocked();
    }
    if (name == "climb") {
      (void)CancelCrawlLocked();
      gait_ = GaitMode::kClimb;
      if (!standing_ && !CallStandUpLocked()) {
        return false;
      }
      // Optional one-shot velocity: climb=vx,vy,wz
      if (!command.value.empty()) {
        float vx = 0.0f;
        float vy = 0.0f;
        float wz = 0.0f;
        float unused = 0.0f;
        if (!ParseAttitudeValue(command.value, &vx, &vy, &wz, &unused)) {
          return false;
        }
        return SendTwistLocked(vx, vy, wz);
      }
      AINFO << "l1w: gait=climb";
      return true;
    }
    if (name == "crawl") {
      (void)CancelClimbLocked();
      gait_ = GaitMode::kCrawl;
      if (!standing_ && !CallStandUpLocked()) {
        return false;
      }
      AINFO << "l1w: gait=crawl";
      return true;
    }
    if (name == "move" || name == "wheel") {
      (void)CancelSpecialGaitsLocked();
      gait_ = GaitMode::kMove;
      if (!standing_ && !CallStandUpLocked()) {
        return false;
      }
      AINFO << "l1w: gait=move";
      return true;
    }
    if (name == "shake_hand" || name == "shakehand" || name == "handshake") {
      if (!standing_ && !CallStandUpLocked()) {
        return false;
      }
#if defined(AUTODRIVER_HAVE_GENISOM_L1W)
      if (sdk_ && !simulate_) {
        const auto ret = sdk_->shakeHand();
        if (ret != 0) {
          AWARN << "l1w: shakeHand ret=" << ret;
          return false;
        }
      }
#endif
      AINFO << "l1w: shakeHand";
      return true;
    }
    if (name == "rear_squat" || name == "rearsquat" || name == "squat") {
      if (!standing_ && !CallStandUpLocked()) {
        return false;
      }
#if defined(AUTODRIVER_HAVE_GENISOM_L1W)
      if (sdk_ && !simulate_) {
        const auto ret = sdk_->rearSquat();
        if (ret != 0) {
          AWARN << "l1w: rearSquat ret=" << ret;
          return false;
        }
      }
#endif
      AINFO << "l1w: rearSquat";
      return true;
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
    (void)CancelSpecialGaitsLocked();
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
      case GaitMode::kClimb:
        return "climb";
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
      // HighLevel rejects move() until standUp fully settles.
      const int stand_settle_ms =
          hardware::ParseInt(params_, "stand_settle_ms", 2000);
      if (stand_settle_ms > 0) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(stand_settle_ms));
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

  bool CancelClimbLocked() {
    if (!climbing_) {
      return true;
    }
#if defined(AUTODRIVER_HAVE_GENISOM_L1W)
    if (sdk_ && !simulate_) {
      sdk_->cancelClimb();
    }
#endif
    climbing_ = false;
    AINFO << "l1w: cancelClimb";
    return true;
  }

  bool CancelSpecialGaitsLocked() {
    const bool crawl_ok = CancelCrawlLocked();
    const bool climb_ok = CancelClimbLocked();
    return crawl_ok && climb_ok;
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
      if (gait_ == GaitMode::kClimb &&
          (std::abs(vx) > 1e-9 || std::abs(vy) > 1e-9 || std::abs(wz) > 1e-9)) {
        climbing_ = true;
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
      if (climbing_) {
        sdk_->cancelClimb();
        climbing_ = false;
      }
      ret = sdk_->crawl(fvx, fvy, fwz);
      crawling_ = true;
    } else if (gait_ == GaitMode::kClimb) {
      if (crawling_) {
        sdk_->cancelCrawl();
        crawling_ = false;
      }
      ret = sdk_->climb(fvx, fvy, fwz);
      climbing_ = true;
    } else {
      if (crawling_) {
        sdk_->cancelCrawl();
        crawling_ = false;
      }
      if (climbing_) {
        sdk_->cancelClimb();
        climbing_ = false;
      }
      ret = sdk_->move(fvx, fvy, fwz);
    }
    if (ret != 0) {
      // Zero-velocity "stop" is often rejected right after standUp / long move;
      // treat as soft success so Start()/Stop() still succeed.
      if (std::abs(vx) <= 1e-9 && std::abs(vy) <= 1e-9 &&
          std::abs(wz) <= 1e-9) {
        AWARN << "l1w: zero twist ignored ret=" << ret;
        return true;
      }
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
      UpdateMapNameLocked(/*ctrl_mode=*/-1, /*acc=*/{}, /*rpy=*/{},
                          /*joint_summary=*/"");
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
      // Heuristic: treat 100% + charging contact unknown → leave flags alone.
    }

    const auto quat = sdk_->getQuaternion();
    const auto pos = sdk_->getPosition();
    const auto rpy = sdk_->getRPY();
    if (pos.size() >= 2) {
      auto* pose = state_.mutable_pose()->mutable_pose();
      pose->mutable_position()->set_x(pos[0]);
      pose->mutable_position()->set_y(pos[1]);
      pose->mutable_position()->set_z(pos.size() > 2 ? pos[2] : 0.0);
      if (quat.size() >= 4) {
        // Docs / vendor: [w, x, y, z]
        pose->mutable_orientation()->set_w(quat[0]);
        pose->mutable_orientation()->set_x(quat[1]);
        pose->mutable_orientation()->set_y(quat[2]);
        pose->mutable_orientation()->set_z(quat[3]);
      } else {
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

    const auto acc = sdk_->getBodyAcc();
    // No dedicated IMU field on RobotState: fold |acc| into localization_quality
    // as a [0,1] health proxy (1 = near 1g upright), and stash raw XYZ in map_name.
    if (acc.size() >= 3) {
      const double mag =
          std::sqrt(static_cast<double>(acc[0]) * acc[0] +
                    static_cast<double>(acc[1]) * acc[1] +
                    static_cast<double>(acc[2]) * acc[2]);
      const double quality =
          std::clamp(1.0 - std::abs(mag - 9.81) / 9.81, 0.0, 1.0);
      state_.set_localization_quality(static_cast<float>(quality));
    }

    std::string joint_summary;
    if (sample_joints_) {
      const auto abad = sdk_->getLegAbadJoint();
      const auto hip = sdk_->getLegHipJoint();
      const auto knee = sdk_->getLegKneeJoint();
      const auto foot = sdk_->getLegFootJoint();
      const auto abad_v = sdk_->getLegAbadJointVel();
      const auto hip_v = sdk_->getLegHipJointVel();
      const auto knee_v = sdk_->getLegKneeJointVel();
      const auto foot_v = sdk_->getLegFootJointVel();
      const auto abad_t = sdk_->getLegAbadJointTorque();
      const auto hip_t = sdk_->getLegHipJointTorque();
      const auto knee_t = sdk_->getLegKneeJointTorque();
      const auto foot_t = sdk_->getLegFootJointTorque();
      // Compact: counts + first-leg sample (RobotState has no JointState field).
      std::ostringstream js;
      js << "n=" << abad.size() << "/" << hip.size() << "/" << knee.size()
         << "/" << foot.size();
      if (!abad.empty()) {
        js << ";abad0=" << abad[0];
      }
      if (!hip.empty()) {
        js << ";hip0=" << hip[0];
      }
      if (!knee.empty()) {
        js << ";knee0=" << knee[0];
      }
      if (!foot.empty()) {
        js << ";foot0=" << foot[0];
      }
      if (!abad_v.empty()) {
        js << ";abad_v0=" << abad_v[0];
      }
      if (!hip_v.empty()) {
        js << ";hip_v0=" << hip_v[0];
      }
      if (!knee_v.empty()) {
        js << ";knee_v0=" << knee_v[0];
      }
      if (!foot_v.empty()) {
        js << ";foot_v0=" << foot_v[0];
      }
      if (!abad_t.empty()) {
        js << ";abad_t0=" << abad_t[0];
      }
      if (!hip_t.empty()) {
        js << ";hip_t0=" << hip_t[0];
      }
      if (!knee_t.empty()) {
        js << ";knee_t0=" << knee_t[0];
      }
      if (!foot_t.empty()) {
        js << ";foot_t0=" << foot_t[0];
      }
      joint_summary = js.str();
    }

    const auto mode = static_cast<int>(sdk_->getCurrentCtrlmode());
    if (mode == 1 || mode == 3) {
      standing_ = true;
    } else if (mode == 0) {
      standing_ = false;
    }
    UpdateMapNameLocked(mode, acc, rpy, joint_summary);
#else
    UpdateMapNameLocked(/*ctrl_mode=*/-1, /*acc=*/{}, /*rpy=*/{},
                        /*joint_summary=*/"");
#endif
  }

  void UpdateMapNameLocked(int ctrl_mode, const std::vector<float>& acc,
                           const std::vector<float>& rpy,
                           const std::string& joint_summary) {
    // RobotState has no ctrl/IMU/joint fields; pack debug telemetry into
    // map_name (active_cmd_id is reserved for operational mode by Manager).
    std::ostringstream oss;
    if (ctrl_mode == 0) {
      oss << "ctrl:passive";
    } else if (ctrl_mode == 1) {
      oss << "ctrl:stand";
    } else if (ctrl_mode == 3) {
      oss << "ctrl:move";
    } else if (ctrl_mode >= 0) {
      oss << "ctrl:" << ctrl_mode;
    } else {
      oss << "ctrl:sim";
    }
    oss << ";gait:" << GaitName(gait_);
    if (acc.size() >= 3) {
      oss << ";acc:" << FormatFloatCsv(acc, 3);
    }
    if (rpy.size() >= 3) {
      oss << ";rpy:" << FormatFloatCsv(rpy, 3);
    }
    if (!joint_summary.empty()) {
      oss << ";j:" << joint_summary;
    }
    state_.set_map_name(oss.str());
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
  bool sample_joints_ = true;
  bool simulate_ = false;
  GaitMode gait_ = GaitMode::kMove;
  mutable std::mutex mutex_;
  bool running_ = false;
  bool standing_ = false;
  bool crawling_ = false;
  bool climbing_ = false;
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
