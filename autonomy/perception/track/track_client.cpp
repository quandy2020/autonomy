/*
 * Copyright 2026 The Openbot Authors
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
 * @file
 * @brief Autolink TrackClient: subscribe sensors, publish velocity commands.
 */

#include "autonomy/perception/track/track_client.hpp"

#include <chrono>

#include "autonomy/common/logging.hpp"
#include "autonomy/perception/track/common/constants.hpp"
#include "autonomy/perception/track/common/types.hpp"
#include "autonomy/perception/track/track_options.hpp"

namespace autonomy::perception::track {

TrackClient::TrackClient() {
  track_options_ = DefaultOptions();
  tracker_ = std::make_shared<Tracker>(track_options_);
}

TrackClient::~TrackClient() { Shutdown(); }

std::shared_ptr<Tracker> TrackClient::AcquireTracker() const {
  std::lock_guard<std::mutex> lock(tracker_mutex_);
  return tracker_;
}

Tracker* TrackClient::tracker() { return AcquireTracker().get(); }

void TrackClient::SetOptions(const Options& options) {
  options_ = options;
  proto::TrackOptions loaded =
      options_.config_directory.empty()
          ? DefaultOptions()
          : LoadOptions(options_.config_directory, options_.track_config);
  auto replacement = std::make_shared<Tracker>(loaded);
  {
    std::lock_guard<std::mutex> lock(tracker_mutex_);
    track_options_ = std::move(loaded);
    tracker_ = std::move(replacement);
  }
}

bool TrackClient::AttachNode(const std::shared_ptr<autolink::Node>& node) {
  if (!node) {
    return false;
  }
  node_ = node;

  odometry_reader_ = node_->CreateReader<automsgs::msgs::nav_msgs::Odometry>(
      track_options_.odom_topic(),
      [this](const std::shared_ptr<automsgs::msgs::nav_msgs::Odometry>&
                 message) { HandleOdometry(message); });
  depth_reader_ = node_->CreateReader<automsgs::msgs::sensor_msgs::Image>(
      track_options_.depth_topic(),
      [this](const std::shared_ptr<automsgs::msgs::sensor_msgs::Image>&
                 message) { HandleDepth(message); });
  reset_reader_ = node_->CreateReader<automsgs::msgs::std_msgs::Bool>(
      kTrackResetTopic,
      [this](const std::shared_ptr<automsgs::msgs::std_msgs::Bool>& message) {
        HandleReset(message);
      });
  pause_reader_ = node_->CreateReader<automsgs::msgs::std_msgs::Bool>(
      kTrackPauseTopic,
      [this](const std::shared_ptr<automsgs::msgs::std_msgs::Bool>& message) {
        HandlePause(message);
      });

  velocity_command_writer_ =
      node_->CreateWriter<automsgs::msgs::geometry_msgs::TwistStamped>(
          track_options_.cmd_vel_topic());
  debug_path_writer_ = node_->CreateWriter<automsgs::msgs::nav_msgs::Path>(
      track_options_.debug_path_topic());
  return true;
}

void TrackClient::Start() {
  if (is_running_.exchange(true)) {
    return;
  }
  planning_thread_ = std::thread([this] { PlanningLoop(); });
  AINFO << "TrackClient started (human follow → "
        << track_options_.cmd_vel_topic() << ")";
}

void TrackClient::Shutdown() {
  if (!is_running_.exchange(false)) {
    return;
  }
  if (planning_thread_.joinable()) {
    planning_thread_.join();
  }
}

void TrackClient::HandleOdometry(
    const std::shared_ptr<automsgs::msgs::nav_msgs::Odometry>& message) {
  auto tracker = AcquireTracker();
  if (message && tracker) {
    tracker->UpdateOdometry(*message);
  }
}

void TrackClient::HandleDepth(
    const std::shared_ptr<automsgs::msgs::sensor_msgs::Image>& message) {
  auto tracker = AcquireTracker();
  if (message && tracker) {
    tracker->UpdateDepth(*message);
  }
}

void TrackClient::HandleReset(
    const std::shared_ptr<automsgs::msgs::std_msgs::Bool>& message) {
  auto tracker = AcquireTracker();
  if (message && message->data() && tracker) {
    tracker->Reset();
  }
}

void TrackClient::HandlePause(
    const std::shared_ptr<automsgs::msgs::std_msgs::Bool>& message) {
  auto tracker = AcquireTracker();
  if (message && tracker) {
    tracker->SetPaused(message->data());
  }
}

void TrackClient::PlanningLoop() {
  const double control_hz = track_options_.control_hz() > 0
                                ? track_options_.control_hz()
                                : defaults::kControlHz;
  const auto period = std::chrono::duration<double>(1.0 / control_hz);
  while (is_running_) {
    const auto cycle_start = std::chrono::steady_clock::now();
    auto tracker = AcquireTracker();
    TrackResult result;
    if (tracker && tracker->RunPlanningCycle(&result) &&
        velocity_command_writer_) {
      velocity_command_writer_->Write(result.velocity_command);
      if (debug_path_writer_ && result.has_target) {
        automsgs::msgs::nav_msgs::Path path;
        path.mutable_header()->set_frame_id(
            result.velocity_command.header().frame_id());
        if (!result.debug_path_xy_m.empty()) {
          for (const auto& point : result.debug_path_xy_m) {
            auto* pose = path.add_poses();
            pose->mutable_pose()->mutable_position()->set_x(point[0]);
            pose->mutable_pose()->mutable_position()->set_y(point[1]);
          }
        } else {
          auto* origin = path.add_poses();
          origin->mutable_pose()->mutable_position()->set_x(0.0);
          origin->mutable_pose()->mutable_position()->set_y(0.0);
          auto* terminal = path.add_poses();
          terminal->mutable_pose()->mutable_position()->set_x(
              result.selected.terminal_x_m);
          terminal->mutable_pose()->mutable_position()->set_y(
              result.selected.terminal_y_m);
        }
        debug_path_writer_->Write(path);
      }
    }
    const auto elapsed = std::chrono::steady_clock::now() - cycle_start;
    if (elapsed < period) {
      std::this_thread::sleep_for(period - elapsed);
    }
  }
}

}  // namespace autonomy::perception::track
