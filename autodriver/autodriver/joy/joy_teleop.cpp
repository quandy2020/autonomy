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
 * @file joy_teleop.cpp
 * @brief JoyTeleop implementation.
 */

#include "autodriver/joy/joy_teleop.hpp"

#include <algorithm>
#include <chrono>

#include "autodriver/joy/joy_mapper.hpp"
#include "autolink/common/log.hpp"
#include "autolink/time/duration.hpp"
#include "autolink/time/time.hpp"
#include <automsgs/msgs/time_utils.hpp>

namespace autodriver {
namespace joy {
namespace {

using JoyMsg = automsgs::msgs::sensor_msgs::Joy;
using TwistStamped = automsgs::msgs::geometry_msgs::TwistStamped;

void StampHeader(automsgs::msgs::std_msgs::Header* header,
                 const std::string& frame_id) {
  if (header == nullptr) {
    return;
  }
  *header->mutable_stamp() =
      automsgs::msgs::builtin_interfaces::TimeFromNanoseconds(
          autolink::Time::Now().ToNanosecond());
  header->set_frame_id(frame_id);
}

}  // namespace

JoyTeleop::~JoyTeleop() { Stop(); }

bool JoyTeleop::Start(autolink::Node* node, const Config& config) {
  Stop();
  options_ = config.joy;
  if (!options_.enable) {
    return true;
  }
  if (node == nullptr) {
    AERROR << "JoyTeleop: null Autolink node";
    return false;
  }

  if (options_.cmd_vel_channel.empty()) {
    options_.cmd_vel_channel = config.chassis.cmd_vel_channel;
  }
  if (options_.cmd_vel_channel.empty()) {
    options_.cmd_vel_channel = "/cmd_vel";
  }
  if (options_.joy_channel.empty()) {
    options_.joy_channel = "/joy";
  }
  if (options_.device.empty()) {
    options_.device = "/dev/input/js0";
  }
  if (options_.publish_hz <= 0.0) {
    options_.publish_hz = 50.0;
  }

  node_ = node;
  joy_writer_ = node_->CreateWriter<JoyMsg>(options_.joy_channel);
  cmd_writer_ = node_->CreateWriter<TwistStamped>(options_.cmd_vel_channel);
  if (!joy_writer_ || !cmd_writer_) {
    AERROR << "JoyTeleop: CreateWriter failed joy=" << options_.joy_channel
           << " cmd=" << options_.cmd_vel_channel;
    joy_writer_.reset();
    cmd_writer_.reset();
    node_ = nullptr;
    return false;
  }

  if (!device_.Open(options_.device)) {
    AWARN << "JoyTeleop: device unavailable path=" << options_.device
          << " — teleop idle until restart with a joystick";
    // Soft-fail: keep process up; no publish thread.
    joy_writer_.reset();
    cmd_writer_.reset();
    node_ = nullptr;
    return true;
  }

  running_ = true;
  thread_ = std::thread([this] { RunLoop(); });
  AINFO << "JoyTeleop started profile=" << options_.profile
        << " device=" << options_.device
        << " joy=" << options_.joy_channel
        << " cmd=" << options_.cmd_vel_channel
        << " lin_axis=" << options_.linear_axis
        << " ang_axis=" << options_.angular_axis
        << " enable_button=" << options_.enable_button
        << " require_enable=" << (options_.require_enable ? "true" : "false")
        << " hz=" << options_.publish_hz;
  return true;
}

void JoyTeleop::Stop() {
  const bool was = running_.exchange(false);
  if (thread_.joinable()) {
    thread_.join();
  }
  if (was && cmd_writer_) {
    PublishZeroTwist();
  }
  device_.Close();
  joy_writer_.reset();
  cmd_writer_.reset();
  node_ = nullptr;
}

void JoyTeleop::PublishZeroTwist() {
  if (!cmd_writer_) {
    return;
  }
  auto msg = std::make_shared<TwistStamped>();
  StampHeader(msg->mutable_header(), options_.frame_id);
  msg->mutable_twist()->mutable_linear()->set_x(0.0);
  msg->mutable_twist()->mutable_angular()->set_z(0.0);
  cmd_writer_->Write(msg);
}

void JoyTeleop::RunLoop() {
  const double hz = std::max(1.0, options_.publish_hz);
  const int64_t period_ns =
      static_cast<int64_t>(1e9 / hz);
  while (running_.load()) {
    const auto t0 = std::chrono::steady_clock::now();
    if (!device_.IsOpen()) {
      AWARN << "JoyTeleop: device closed; publishing zero and exiting loop";
      PublishZeroTwist();
      break;
    }
    device_.Poll();

    const auto axes = device_.Axes();
    const auto buttons = device_.Buttons();

    auto joy = std::make_shared<JoyMsg>();
    StampHeader(joy->mutable_header(), options_.frame_id);
    for (float a : axes) {
      joy->add_axes(a);
    }
    for (int32_t b : buttons) {
      joy->add_buttons(b);
    }
    joy_writer_->Write(joy);

    const DiffTwist twist = MapDifferential(
        axes, buttons, options_.linear_axis, options_.angular_axis,
        options_.invert_linear, options_.invert_angular, options_.deadzone,
        options_.max_linear, options_.max_angular, options_.require_enable,
        options_.enable_button);

    auto cmd = std::make_shared<TwistStamped>();
    StampHeader(cmd->mutable_header(), options_.frame_id);
    cmd->mutable_twist()->mutable_linear()->set_x(twist.linear_x);
    cmd->mutable_twist()->mutable_angular()->set_z(twist.angular_z);
    cmd_writer_->Write(cmd);

    const auto elapsed = std::chrono::steady_clock::now() - t0;
    const int64_t elapsed_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
    const int64_t sleep_ns = period_ns - elapsed_ns;
    if (sleep_ns > 0) {
      autolink::Duration(sleep_ns).Sleep();
    }
  }
  running_ = false;
}

}  // namespace joy
}  // namespace autodriver
