/*
 * Copyright 2026 Autodriver contributors
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

#include "chassis/chassis_manager.hpp"

#include <algorithm>
#include <chrono>

#include "chassis/backend_registry.hpp"
#include "chassis/convert.hpp"
#include "chassis/stub/driver.hpp"
#include "autolink/common/log.hpp"
#include "autolink/time/duration.hpp"

namespace autodriver {
namespace chassis {
namespace {

using Odometry = automsgs::msgs::nav_msgs::Odometry;

/**
 * @brief Steady-clock epoch time in nanoseconds (watchdog / publish cadence).
 */
std::uint64_t ReadSteadyTimeNanoseconds() {
  using clock = std::chrono::steady_clock;
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          clock::now().time_since_epoch())
          .count());
}

}  // namespace

ChassisManager::~ChassisManager() { Stop(); }

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

  // Keep stub TU linked when only this manager is used from a binary.
  (void)&CreateStubChassisDriver;

  ChassisId id = options_.id.empty() ? "chassis/base" : options_.id;
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
  cmd_reader_ = node_->CreateReader<ChassisCommand>(
      options_.cmd_vel_channel,
      [this](const std::shared_ptr<ChassisCommand>& msg) {
        HandleVelocityCommand(msg);
      });

  last_cmd_ns_ = ReadSteadyTimeNanoseconds();
  running_ = true;
  publish_thread_ = std::thread([this] { RunStatePublishLoop(); });

  AINFO << "ChassisManager started backend=" << options_.backend << " id=" << id
        << " cmd=" << options_.cmd_vel_channel
        << " state=" << options_.state_channel
        << " (vehicle_msgs.RobotState)";
  return true;
}

void ChassisManager::Stop() {
  const bool was_running = running_.exchange(false);
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
  cmd_reader_.reset();
  state_writer_.reset();
  event_writer_.reset();
  odom_writer_.reset();
  if (driver_) {
    if (was_running) {
      driver_->TriggerEmergencyStop();
    }
    driver_->Stop();
    driver_.reset();
  }
  node_ = nullptr;
}

ChassisCommand ChassisManager::ClampVelocityCommand(
    const ChassisCommand& in) const {
  ChassisCommand out = in;
  if (!out.has_twist()) {
    return out;
  }
  auto* twist = out.mutable_twist();
  if (options_.max_linear_speed > 0.0) {
    twist->mutable_linear()->set_x(std::clamp(
        twist->linear().x(), -options_.max_linear_speed,
        options_.max_linear_speed));
    twist->mutable_linear()->set_y(std::clamp(
        twist->linear().y(), -options_.max_linear_speed,
        options_.max_linear_speed));
  }
  if (options_.max_angular_speed > 0.0) {
    twist->mutable_angular()->set_z(std::clamp(
        twist->angular().z(), -options_.max_angular_speed,
        options_.max_angular_speed));
  }
  return out;
}

void ChassisManager::HandleVelocityCommand(
    const std::shared_ptr<ChassisCommand>& msg) {
  if (!msg || !driver_ || !running_.load()) {
    return;
  }
  ChassisCommand cmd = ClampVelocityCommand(*msg);
  std::lock_guard<std::mutex> lock(mutex_);
  last_cmd_ns_ = ReadSteadyTimeNanoseconds();
  driver_->ApplyVelocityCommand(cmd);
}

void ChassisManager::HandleDriverEvent(const ChassisEvent& event) {
  if (event_writer_) {
    event_writer_->Write(event);
  }
}

void ChassisManager::ApplyCommandWatchdogLocked(std::uint64_t now_ns) {
  if (options_.watchdog_ms <= 0 || !driver_) {
    return;
  }
  const std::uint64_t timeout_ns =
      static_cast<std::uint64_t>(options_.watchdog_ms) * 1'000'000ULL;
  if (now_ns > last_cmd_ns_ && (now_ns - last_cmd_ns_) > timeout_ns) {
    ChassisCommand stop;
    FillTimestampFromNanoseconds(stop.mutable_header()->mutable_stamp(),
                                 now_ns);
    driver_->ApplyVelocityCommand(stop);
  }
}

void ChassisManager::RunStatePublishLoop() {
  const int period_ms =
      options_.odom_period_ms > 0 ? options_.odom_period_ms : 20;
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
    }
    autolink::Duration(static_cast<std::int64_t>(period_ms) * 1'000'000LL)
        .Sleep();
  }
}

}  // namespace chassis
}  // namespace autodriver
