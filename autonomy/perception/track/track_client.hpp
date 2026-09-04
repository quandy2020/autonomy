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
 * @brief Autolink I/O bridge for ground-robot human following.
 */

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/std_msgs/bool.pb.h>

#include "autolink/autolink.hpp"
#include "autolink/node/reader.hpp"
#include "autolink/node/writer.hpp"
#include "autonomy/perception/proto/track_options.pb.h"
#include "autonomy/perception/track/tracker.hpp"

namespace autonomy::perception::track {

/**
 * @class autonomy::perception::track::TrackClient
 * @brief Subscribes to odometry/depth, runs Tracker, publishes velocity commands.
 */
class TrackClient {
 public:
  struct Options {
    std::string config_directory;
    std::string track_config = "perception/track_yopo.lua";
  };

  TrackClient();
  ~TrackClient();

  void SetOptions(const Options& options);
  bool AttachNode(const std::shared_ptr<autolink::Node>& node);
  void Start();
  void Shutdown();

  /**
   * @brief Non-owning access for tests; may be null briefly during SetOptions.
   */
  Tracker* tracker();

 private:
  std::shared_ptr<Tracker> AcquireTracker() const;

  void HandleOdometry(
      const std::shared_ptr<automsgs::msgs::nav_msgs::Odometry>& message);
  void HandleDepth(
      const std::shared_ptr<automsgs::msgs::sensor_msgs::Image>& message);
  void HandleReset(
      const std::shared_ptr<automsgs::msgs::std_msgs::Bool>& message);
  void HandlePause(
      const std::shared_ptr<automsgs::msgs::std_msgs::Bool>& message);
  void PlanningLoop();

  Options options_;
  proto::TrackOptions track_options_;

  mutable std::mutex tracker_mutex_;
  std::shared_ptr<Tracker> tracker_;

  std::shared_ptr<autolink::Node> node_;
  std::shared_ptr<autolink::Reader<automsgs::msgs::nav_msgs::Odometry>>
      odometry_reader_;
  std::shared_ptr<autolink::Reader<automsgs::msgs::sensor_msgs::Image>>
      depth_reader_;
  std::shared_ptr<autolink::Reader<automsgs::msgs::std_msgs::Bool>>
      reset_reader_;
  std::shared_ptr<autolink::Reader<automsgs::msgs::std_msgs::Bool>>
      pause_reader_;
  std::shared_ptr<autolink::Writer<automsgs::msgs::geometry_msgs::TwistStamped>>
      velocity_command_writer_;
  std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Path>>
      debug_path_writer_;

  std::atomic<bool> is_running_{false};
  std::thread planning_thread_;
};

}  // namespace autonomy::perception::track
