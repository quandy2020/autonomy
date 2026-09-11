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
 * @file chassis_manager.cpp
 * @brief ChassisManager + SafetyGate / capability / mode / tool (implementation).
 */

#include "chassis/chassis_manager.hpp"

#include <chrono>

#include "chassis/backend_registry.hpp"
#include "chassis/convert.hpp"
#include "chassis/motion_command.hpp"
#include "chassis/stub/driver.hpp"
#include "chassis/tool_command.hpp"
#include "autolink/common/log.hpp"
#include "autolink/time/duration.hpp"
#include <automsgs/msgs/vehicle_msgs/robot_event_type.pb.h>

namespace autodriver {
namespace chassis {
namespace {

using Odometry = automsgs::msgs::nav_msgs::Odometry;
using StringMsg = automsgs::msgs::std_msgs::String;

std::uint64_t ReadSteadyTimeNanoseconds() {
  using clock = std::chrono::steady_clock;
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          clock::now().time_since_epoch())
          .count());
}

bool ParseOptionalBool(const std::string& text, bool* out) {
  if (out == nullptr || text.empty()) {
    return false;
  }
  if (text == "1" || text == "true" || text == "True" || text == "yes") {
    *out = true;
    return true;
  }
  if (text == "0" || text == "false" || text == "False" || text == "no") {
    *out = false;
    return true;
  }
  return false;
}

}  // namespace

ChassisManager::~ChassisManager() { Stop(); }

CapabilityProfile ChassisManager::BuildCapability(
    const Config::Chassis& options) const {
  CapabilityProfile profile;
  profile.chassis_id = options.id.empty() ? "chassis/base" : options.id;
  profile.backend = options.backend;
  profile.locomotion.type = ParseLocomotionType(options.locomotion);
  ApplyLocomotionTypeDefaults(&profile.locomotion);
  profile.locomotion.max_linear_speed = options.max_linear_speed;
  profile.locomotion.max_angular_speed = options.max_angular_speed;
  profile.locomotion.max_linear_accel = options.max_linear_accel;
  profile.locomotion.min_turning_radius = options.min_turning_radius;
  bool flag = false;
  if (ParseOptionalBool(options.supports_lateral, &flag)) {
    profile.locomotion.supports_lateral = flag;
  }
  if (ParseOptionalBool(options.supports_inplace_turn, &flag)) {
    profile.locomotion.supports_inplace_turn = flag;
  }
  profile.has_dock = options.has_dock;
  profile.has_tool_channel = !options.tool_cmd_channel.empty();
  profile.has_joint_bypass = options.has_joint_bypass;
  profile.require_arm = options.require_arm;
  profile.tools = options.tools;
  return profile;
}

SafetyLimits ChassisManager::BuildSafetyLimits(
    const Config::Chassis& options) const {
  SafetyLimits limits;
  limits.model = BuildCapability(options).locomotion;
  limits.require_arm = options.require_arm;
  limits.watchdog_ms = options.watchdog_ms;
  return limits;
}

bool ChassisManager::Start(autolink::Node* node, const Config& config) {
  Stop();
  options_ = config.chassis;
  if (!options_.enable) {
    return true;
  }
  if (node == nullptr) {
    AERROR << "ChassisManager: null Autolink node";
    return false;
  }

  (void)&CreateStubChassisDriver;

  capability_ = BuildCapability(options_);
  safety_ = std::make_unique<SafetyGate>(BuildSafetyLimits(options_));

  ChassisId id = capability_.chassis_id;
  driver_ = ChassisBackendRegistry::Instance().CreateDriver(
      options_.backend, id, options_.params);
  if (!driver_) {
    AERROR << "ChassisManager: failed to create backend=" << options_.backend;
    return false;
  }
  driver_->SetEventCallback(
      [this](const ChassisEvent& event) { HandleDriverEvent(event); });
  if (!driver_->Start()) {
    AERROR << "ChassisManager: driver Start() failed";
    driver_.reset();
    return false;
  }

  node_ = node;
  state_writer_ = node_->CreateWriter<ChassisState>(options_.state_channel);
  if (!options_.event_channel.empty()) {
    event_writer_ = node_->CreateWriter<ChassisEvent>(options_.event_channel);
  }
  if (!options_.odom_channel.empty()) {
    odom_writer_ = node_->CreateWriter<Odometry>(options_.odom_channel);
  }
  if (!options_.capability_channel.empty()) {
    capability_writer_ =
        node_->CreateWriter<StringMsg>(options_.capability_channel);
  }
  if (!options_.mode_state_channel.empty()) {
    mode_state_writer_ =
        node_->CreateWriter<StringMsg>(options_.mode_state_channel);
  }

  cmd_reader_ = node_->CreateReader<ChassisCommand>(
      options_.cmd_vel_channel,
      [this](const std::shared_ptr<ChassisCommand>& msg) {
        HandleVelocityCommand(msg);
      });
  if (!options_.mode_cmd_channel.empty()) {
    mode_reader_ = node_->CreateReader<StringMsg>(
        options_.mode_cmd_channel,
        [this](const std::shared_ptr<StringMsg>& msg) {
          HandleModeCommand(msg);
        });
  }
  if (!options_.tool_cmd_channel.empty()) {
    tool_reader_ = node_->CreateReader<StringMsg>(
        options_.tool_cmd_channel,
        [this](const std::shared_ptr<StringMsg>& msg) {
          HandleToolCommand(msg);
        });
  }

  last_cmd_ns_ = ReadSteadyTimeNanoseconds();
  capability_tick_ = 0;
  running_ = true;
  publish_thread_ = std::thread([this] { RunStatePublishLoop(); });

  {
    std::lock_guard<std::mutex> lock(mutex_);
    PublishCapabilityLocked();
    PublishModeStateLocked();
  }

  AINFO << "ChassisManager started backend=" << options_.backend << " id=" << id
        << " locomotion=" << LocomotionTypeToString(capability_.locomotion.type)
        << " require_arm=" << (options_.require_arm ? "true" : "false")
        << " cmd=" << options_.cmd_vel_channel
        << " state=" << options_.state_channel;
  return true;
}

void ChassisManager::Stop() {
  const bool was_running = running_.exchange(false);
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
  cmd_reader_.reset();
  mode_reader_.reset();
  tool_reader_.reset();
  state_writer_.reset();
  event_writer_.reset();
  odom_writer_.reset();
  capability_writer_.reset();
  mode_state_writer_.reset();
  if (driver_) {
    if (was_running) {
      driver_->TriggerEmergencyStop();
      modes_.NotifyEStop();
    }
    driver_->Stop();
    driver_.reset();
  }
  safety_.reset();
  node_ = nullptr;
}

void ChassisManager::HandleVelocityCommand(
    const std::shared_ptr<ChassisCommand>& msg) {
  if (!msg || !driver_ || !safety_ || !running_.load()) {
    return;
  }

  ChassisCommand cmd = safety_->ClampVelocity(*msg);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!safety_->AllowsMotion(modes_)) {
    ADEBUG << "ChassisManager: twist rejected (mode=" << modes_.ModeString()
           << ")";
    return;
  }

  last_cmd_ns_ = ReadSteadyTimeNanoseconds();
  const bool non_zero = IsNonZeroTwist(cmd);
  if (!driver_->ApplyVelocityCommand(cmd)) {
    return;
  }
  modes_.NotifyMotionApplied(non_zero);

  MotionCommand motion;
  motion.twist = cmd;
  motion.intent = modes_.intent();
  motion.has_intent = (motion.intent != LocomotionIntent::kUnspecified);
  if (motion.has_intent) {
    driver_->ApplyLocomotionIntent(motion.intent);
  }
}

void ChassisManager::HandleModeCommand(
    const std::shared_ptr<StringMsg>& msg) {
  if (!msg || !running_.load()) {
    return;
  }
  std::string error;
  const bool ok = modes_.HandleModeCommand(msg->data(), &error);
  if (!ok) {
    AWARN << "ChassisManager: mode command rejected: " << msg->data()
          << " (" << error << ")";
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (modes_.mode() == OperationalMode::kEStop && driver_) {
    driver_->TriggerEmergencyStop();
  }
  if (modes_.intent() != LocomotionIntent::kUnspecified && driver_) {
    driver_->ApplyLocomotionIntent(modes_.intent());
  }
  PublishModeStateLocked();
  AINFO << "ChassisManager: mode=" << modes_.ModeString()
        << " intent=" << modes_.IntentString();
}

void ChassisManager::HandleToolCommand(
    const std::shared_ptr<StringMsg>& msg) {
  if (!msg || !driver_ || !running_.load()) {
    return;
  }
  ToolCommand tool;
  if (!ParseToolCommand(msg->data(), &tool)) {
    AWARN << "ChassisManager: bad tool command: " << msg->data();
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!driver_->ApplyToolCommand(tool)) {
    AWARN << "ChassisManager: tool unsupported by backend: " << tool.name;
  }
}

void ChassisManager::HandleDriverEvent(const ChassisEvent& event) {
  if (event.type() ==
          ::automsgs::msgs::vehicle_msgs::ROBOT_EVENT_EMERGENCY_STOP) {
    modes_.NotifyEStop();
  } else if (event.type() ==
             ::automsgs::msgs::vehicle_msgs::ROBOT_EVENT_FAULT) {
    modes_.NotifyFault();
  }
  if (event_writer_) {
    event_writer_->Write(event);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  PublishModeStateLocked();
}

void ChassisManager::ApplyCommandWatchdogLocked(std::uint64_t now_ns) {
  if (!safety_ || !driver_) {
    return;
  }
  if (!safety_->WatchdogExpired(now_ns, last_cmd_ns_)) {
    return;
  }
  ChassisCommand stop;
  FillTimestampFromNanoseconds(stop.mutable_header()->mutable_stamp(), now_ns);
  if (safety_->AllowsMotion(modes_)) {
    driver_->ApplyVelocityCommand(stop);
    modes_.NotifyMotionApplied(false);
  }
}

void ChassisManager::PublishCapabilityLocked() {
  if (!capability_writer_) {
    return;
  }
  StringMsg msg;
  msg.set_data(CapabilityProfileToJson(capability_));
  capability_writer_->Write(msg);
}

void ChassisManager::PublishModeStateLocked() {
  if (!mode_state_writer_) {
    return;
  }
  StringMsg msg;
  msg.set_data(modes_.ModeString() + "," + modes_.IntentString());
  mode_state_writer_->Write(msg);
}

void ChassisManager::EnrichStateWithMode(ChassisState* state) const {
  if (state == nullptr) {
    return;
  }
  // Convention: active_cmd_id carries operational mode for downstream without
  // a proto bump. Prefer mode_state_channel when available.
  state->set_active_cmd_id(modes_.ModeString());
  if (modes_.mode() == OperationalMode::kEStop ||
      modes_.mode() == OperationalMode::kFault) {
    state->set_motion_enabled(false);
  }
}

void ChassisManager::RunStatePublishLoop() {
  const int period_ms =
      options_.odom_period_ms > 0 ? options_.odom_period_ms : 20;
  const int cap_period = options_.capability_period_ticks > 0
                             ? options_.capability_period_ticks
                             : 1;
  while (running_.load()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ApplyCommandWatchdogLocked(ReadSteadyTimeNanoseconds());
      if (driver_) {
        ChassisState state;
        if (driver_->ReadChassisState(&state)) {
          if (state.global_frame().empty()) {
            state.set_global_frame(options_.odom_frame_id);
          }
          EnrichStateWithMode(&state);
          if (state_writer_) {
            state_writer_->Write(state);
          }
          if (odom_writer_) {
            Odometry odom;
            ConvertRobotStateToOdometry(state, options_.odom_frame_id,
                                        options_.base_frame_id, &odom);
            odom_writer_->Write(odom);
          }
        }
      }
      PublishModeStateLocked();
      if (++capability_tick_ >= cap_period) {
        capability_tick_ = 0;
        PublishCapabilityLocked();
      }
    }
    autolink::Duration(static_cast<int64_t>(period_ms) * 1'000'000).Sleep();
  }
}

}  // namespace chassis
}  // namespace autodriver
