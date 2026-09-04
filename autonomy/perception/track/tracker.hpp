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
 * @brief Ground-robot human follower: depth → primitives → selection → velocity.
 */

#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>

#include "autonomy/perception/proto/track_options.pb.h"
#include "autonomy/perception/track/common/types.hpp"
#include "autonomy/perception/track/infer/track_inference_engine.hpp"
#include "autonomy/perception/track/primitive/lattice.hpp"
#include "autonomy/perception/track/utils/velocity_command_mapper.hpp"

namespace autonomy::perception::track {

/**
 * @class autonomy::perception::track::Tracker
 * @brief Thread-safe planner that turns depth + odometry into a follow command.
 *
 * Prefers ONNX inference when a model is loaded; otherwise uses a sector-depth
 * heuristic so autosim bring-up works without weights.
 */
class Tracker {
 public:
  Tracker();
  explicit Tracker(const proto::TrackOptions& options);

  void Configure(const proto::TrackOptions& options);
  void Reset();

  void SetPaused(bool paused) { is_paused_.store(paused); }
  bool IsPaused() const { return is_paused_.load(); }

  void UpdateOdometry(const automsgs::msgs::nav_msgs::Odometry& odometry);
  void UpdateDepth(const automsgs::msgs::sensor_msgs::Image& depth_image);

  /**
   * @brief Runs one planning cycle.
   * @return False when paused or no depth is available yet.
   */
  bool RunPlanningCycle(TrackResult* result);

 private:
  /**
   * @brief Immutable inputs captured under |mutex_| for one planning cycle.
   */
  struct PlanningSnapshot {
    std::vector<float> normalized_depth;
    std::vector<float> robot_state;
    proto::TrackOptions options;
    Lattice lattice;
    VelocityCommandMapper velocity_command_mapper;
    std::string robot_frame_id;
    bool use_network{false};
    bool allow_heuristic_fallback{true};
    bool has_previous_target{false};
    double previous_target_yaw_rad{0.0};
  };

  bool CapturePlanningSnapshot(PlanningSnapshot* snapshot) const;

  bool BuildHypotheses(const PlanningSnapshot& snapshot,
                       std::vector<MotionPrimitiveHypothesis>* hypotheses);

  bool EstimateHypothesesFromDepthSectors(
      const PlanningSnapshot& snapshot,
      std::vector<MotionPrimitiveHypothesis>* hypotheses) const;

  proto::TrackOptions options_;
  Lattice lattice_;
  TrackInferenceEngine inference_engine_;
  VelocityCommandMapper velocity_command_mapper_;

  mutable std::mutex mutex_;
  double robot_linear_velocity_mps_{0.0};
  double robot_yaw_rate_rps_{0.0};
  std::string robot_frame_id_{"base_link"};
  std::vector<float> normalized_depth_;
  bool has_depth_image_{false};
  bool has_odometry_{false};
  bool has_previous_target_{false};
  double previous_target_yaw_rad_{0.0};
  std::atomic<bool> is_paused_{false};
  bool is_network_loaded_{false};
};

}  // namespace autonomy::perception::track
