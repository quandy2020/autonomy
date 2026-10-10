/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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
 * @file arm_manager.cpp
 * @brief ArmManager (implementation).
 */

#include "automanip/arm/arm_manager.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>

#include "automanip/arm/backend_registry.hpp"
#include "automanip/arm/mode.hpp"
#include "autolink/common/log.hpp"

namespace automanip {
namespace arm {
namespace {

std::uint64_t SteadyNs() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

}  // namespace

ArmManager::~ArmManager() { Stop(); }

bool ArmManager::Start(autolink::Node* node, const Config& config) {
  if (node == nullptr) {
    AERROR << "ArmManager: null node";
    return false;
  }
  if (!config.arm.enable) {
    AINFO << "ArmManager: arm disabled";
    return true;
  }
  options_ = config.arm;
  if (options_.id.empty()) {
    options_.id = "arm/manipulator";
  }
  driver_ = ArmBackendRegistry::Instance().Create(options_.backend, options_.id,
                                                  options_.plant);
  if (!driver_) {
    AERROR << "ArmManager: failed to create backend=" << options_.backend;
    return false;
  }
  if (!driver_->Start()) {
    AERROR << "ArmManager: driver Start() failed";
    driver_.reset();
    return false;
  }
  if (!options_.initial_mode.empty()) {
    driver_->ApplyMode(options_.initial_mode);
  }

  node_ = node;
  state_writer_ = node_->CreateWriter<JointState>(options_.joint_state_channel);
  ee_writer_ = node_->CreateWriter<PoseCommand>(options_.ee_pose_channel);
  if (!options_.mode_state_channel.empty()) {
    mode_writer_ = node_->CreateWriter<StringMsg>(options_.mode_state_channel);
  }
  if (!options_.capability_channel.empty()) {
    capability_writer_ =
        node_->CreateWriter<StringMsg>(options_.capability_channel);
  }
  if (!options_.event_channel.empty()) {
    event_writer_ = node_->CreateWriter<StringMsg>(options_.event_channel);
  }
  if (!options_.pose_channel.empty()) {
    pose_reader_ = node_->CreateReader<PoseCommand>(
        options_.pose_channel,
        [this](const std::shared_ptr<PoseCommand>& msg) { HandlePose(msg); });
  }
  if (!options_.joint_command_channel.empty()) {
    joint_reader_ = node_->CreateReader<JointCommand>(
        options_.joint_command_channel,
        [this](const std::shared_ptr<JointCommand>& msg) {
          HandleJointCommand(msg);
        });
  }
  if (!options_.mode_channel.empty()) {
    mode_reader_ = node_->CreateReader<StringMsg>(
        options_.mode_channel,
        [this](const std::shared_ptr<StringMsg>& msg) { HandleMode(msg); });
  }

  last_pose_ns_ = SteadyNs();
  watchdog_latched_ = false;
  capability_tick_ = 0;
  running_ = true;
  thread_ = std::thread([this] { RunControlLoop(); });
  AINFO << "ArmManager started backend=" << options_.backend
        << " id=" << options_.id << " dof=" << options_.plant.chain.dof()
        << " pose=" << options_.pose_channel
        << " state=" << options_.joint_state_channel;
  return true;
}

void ArmManager::Stop() {
  const bool was_running = running_.exchange(false);
  if (thread_.joinable()) {
    thread_.join();
  }
  if (was_running && driver_) {
    driver_->TriggerEmergencyStop();
    driver_->Stop();
  }
  driver_.reset();
  pose_reader_.reset();
  joint_reader_.reset();
  mode_reader_.reset();
  state_writer_.reset();
  ee_writer_.reset();
  mode_writer_.reset();
  capability_writer_.reset();
  event_writer_.reset();
  node_ = nullptr;
}

void ArmManager::HandlePose(const std::shared_ptr<PoseCommand>& msg) {
  if (!msg || !running_.load()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!driver_) {
    return;
  }
  const ArmMode mode = driver_->GetMode();
  if (mode == ArmMode::kEstop || mode == ArmMode::kFault) {
    return;
  }
  if (!driver_->ApplyPoseTarget(*msg)) {
    return;
  }
  driver_->ApplyMode("track");
  last_pose_ns_ = SteadyNs();
  watchdog_latched_ = false;
}

void ArmManager::HandleJointCommand(const std::shared_ptr<JointCommand>& msg) {
  if (!msg || !running_.load()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (driver_) {
    driver_->ApplyJointCommand(*msg);
  }
}

void ArmManager::HandleMode(const std::shared_ptr<StringMsg>& msg) {
  if (!msg || !running_.load()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (driver_) {
    driver_->ApplyMode(msg->data());
  }
}

void ArmManager::Publish(const JointState& state, const PoseCommand& pose,
                         const std::string& mode, const std::string& event) {
  if (state_writer_) {
    state_writer_->Write(std::make_shared<JointState>(state));
  }
  if (ee_writer_) {
    ee_writer_->Write(std::make_shared<PoseCommand>(pose));
  }
  if (mode_writer_) {
    auto msg = std::make_shared<StringMsg>();
    msg->set_data(mode);
    mode_writer_->Write(msg);
  }
  if (!event.empty() && event_writer_) {
    auto msg = std::make_shared<StringMsg>();
    msg->set_data(event);
    event_writer_->Write(msg);
  }
  if (capability_writer_ &&
      (capability_tick_ % std::max(options_.capability_period_ticks, 1) == 0)) {
    std::ostringstream json;
    json << "{\"id\":\"" << options_.id << "\",\"backend\":\""
         << options_.backend << "\",\"dof\":" << options_.plant.chain.dof()
         << ",\"mode\":\"" << mode << "\"}";
    auto msg = std::make_shared<StringMsg>();
    msg->set_data(json.str());
    capability_writer_->Write(msg);
  }
  ++capability_tick_;
}

void ArmManager::RunControlLoop() {
  const int period_ms = std::max(options_.control_period_ms, 1);
  const auto period = std::chrono::milliseconds(period_ms);
  auto next = std::chrono::steady_clock::now();
  while (running_.load()) {
    next += period;
    std::string event;
    JointState state;
    PoseCommand pose;
    std::string mode = "hold";
    bool have_sample = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (driver_) {
        if (options_.watchdog_ms > 0 &&
            driver_->GetMode() == ArmMode::kTrack) {
          const std::uint64_t age = SteadyNs() - last_pose_ns_;
          const std::uint64_t limit =
              static_cast<std::uint64_t>(options_.watchdog_ms) * 1000000ULL;
          if (age > limit && !watchdog_latched_) {
            driver_->ApplyMode("hold");
            watchdog_latched_ = true;
            event = "watchdog_hold";
          }
        }
        driver_->Step(static_cast<double>(period_ms) / 1000.0);
        driver_->ReadJointState(&state);
        driver_->ReadEndEffectorPose(&pose);
        mode = ArmModeName(driver_->GetMode());
        have_sample = true;
      }
    }
    if (have_sample) {
      Publish(state, pose, mode, event);
    }
    std::this_thread::sleep_until(next);
  }
}

}  // namespace arm
}  // namespace automanip
