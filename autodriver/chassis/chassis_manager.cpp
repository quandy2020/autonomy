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
#include <cmath>

#include "chassis/backend_registry.hpp"
#include "chassis/stub/driver.hpp"
#include "autolink/common/log.hpp"
#include "autolink/time/duration.hpp"

namespace autodriver {
namespace chassis {
namespace {

using TwistStamped = automsgs::msgs::geometry_msgs::TwistStamped;
using Odometry = automsgs::msgs::nav_msgs::Odometry;

std::uint64_t SteadyNowNs() {
  using clock = std::chrono::steady_clock;
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          clock::now().time_since_epoch())
          .count());
}

void FillOdometry(const ChassisState& state, const Config::Chassis& opt,
                  Odometry* out) {
  out->Clear();
  auto* header = out->mutable_header();
  header->set_frame_id(opt.odom_frame_id);
  header->mutable_stamp()->set_sec(
      static_cast<std::int32_t>(state.stamp_ns / 1'000'000'000ULL));
  header->mutable_stamp()->set_nanosec(
      static_cast<std::uint32_t>(state.stamp_ns % 1'000'000'000ULL));
  out->set_child_frame_id(opt.base_frame_id);

  auto* pose = out->mutable_pose()->mutable_pose();
  pose->mutable_position()->set_x(state.pose_x);
  pose->mutable_position()->set_y(state.pose_y);
  pose->mutable_position()->set_z(0.0);
  const double half = 0.5 * state.pose_yaw;
  pose->mutable_orientation()->set_z(std::sin(half));
  pose->mutable_orientation()->set_w(std::cos(half));

  auto* twist = out->mutable_twist()->mutable_twist();
  twist->mutable_linear()->set_x(state.linear_x);
  twist->mutable_linear()->set_y(state.linear_y);
  twist->mutable_angular()->set_z(state.angular_z);
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

  // Ensure stub registrar is linked even if LTO drops unused TUs.
  (void)&CreateStubChassisDriver;

  ChassisId id = options_.id.empty() ? "chassis/base" : options_.id;
  driver_ = ChassisBackendRegistry::Instance().Create(
      options_.backend, id, options_.params);
  if (!driver_) {
    AERROR << "ChassisManager: failed to create backend=" << options_.backend;
    return false;
  }
  if (!driver_->Start()) {
    AERROR << "ChassisManager: driver Start() failed";
    driver_.reset();
    return false;
  }

  node_ = node;
  odom_writer_ = node_->CreateWriter<Odometry>(options_.odom_channel);
  cmd_reader_ = node_->CreateReader<TwistStamped>(
      options_.cmd_vel_channel,
      [this](const std::shared_ptr<TwistStamped>& msg) { OnCmdVel(msg); });

  last_cmd_ns_ = SteadyNowNs();
  running_ = true;
  publish_thread_ = std::thread([this] { PublishLoop(); });

  AINFO << "ChassisManager started backend=" << options_.backend
        << " id=" << id << " cmd=" << options_.cmd_vel_channel
        << " odom=" << options_.odom_channel;
  return true;
}

void ChassisManager::Stop() {
  const bool was_running = running_.exchange(false);
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
  cmd_reader_.reset();
  odom_writer_.reset();
  if (driver_) {
    if (was_running) {
      driver_->EmergencyStop();
    }
    driver_->Stop();
    driver_.reset();
  }
  node_ = nullptr;
}

ChassisCommand ChassisManager::Clamp(const ChassisCommand& in) const {
  ChassisCommand out = in;
  if (options_.max_linear_speed > 0.0) {
    out.linear_x = std::clamp(out.linear_x, -options_.max_linear_speed,
                              options_.max_linear_speed);
    out.linear_y = std::clamp(out.linear_y, -options_.max_linear_speed,
                              options_.max_linear_speed);
  }
  if (options_.max_angular_speed > 0.0) {
    out.angular_z = std::clamp(out.angular_z, -options_.max_angular_speed,
                               options_.max_angular_speed);
  }
  return out;
}

void ChassisManager::OnCmdVel(const std::shared_ptr<TwistStamped>& msg) {
  if (!msg || !driver_ || !running_.load()) {
    return;
  }
  ChassisCommand cmd;
  cmd.stamp_ns = SteadyNowNs();
  cmd.linear_x = msg->twist().linear().x();
  cmd.linear_y = msg->twist().linear().y();
  cmd.angular_z = msg->twist().angular().z();
  cmd = Clamp(cmd);

  std::lock_guard<std::mutex> lock(mutex_);
  last_cmd_ns_ = cmd.stamp_ns;
  driver_->ApplyCommand(cmd);
}

void ChassisManager::ApplyWatchdogLocked(std::uint64_t now_ns) {
  if (options_.watchdog_ms <= 0 || !driver_) {
    return;
  }
  const std::uint64_t timeout_ns =
      static_cast<std::uint64_t>(options_.watchdog_ms) * 1'000'000ULL;
  if (now_ns > last_cmd_ns_ && (now_ns - last_cmd_ns_) > timeout_ns) {
    ChassisCommand stop;
    stop.stamp_ns = now_ns;
    driver_->ApplyCommand(stop);
  }
}

void ChassisManager::PublishLoop() {
  const int period_ms =
      options_.odom_period_ms > 0 ? options_.odom_period_ms : 20;
  while (running_.load()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ApplyWatchdogLocked(SteadyNowNs());
      if (driver_ && odom_writer_) {
        ChassisState state;
        if (driver_->GetState(&state)) {
          Odometry odom;
          FillOdometry(state, options_, &odom);
          odom_writer_->Write(odom);
        }
      }
    }
    autolink::Duration(static_cast<std::int64_t>(period_ms) * 1'000'000LL)
        .Sleep();
  }
}

}  // namespace chassis
}  // namespace autodriver
