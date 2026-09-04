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
 * @brief Ground-robot Tracker implementation (depth → velocity command).
 */

#include "autonomy/perception/track/tracker.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/common/logging.hpp"
#include "autonomy/perception/track/common/constants.hpp"
#include "autonomy/perception/track/track_options.hpp"
#include "autonomy/perception/track/utils/depth_preprocess.hpp"
#include "autonomy/perception/track/utils/follow_hypothesis_selection.hpp"

namespace autonomy::perception::track {

Tracker::Tracker() : Tracker(DefaultOptions()) {}

Tracker::Tracker(const proto::TrackOptions& options)
    : velocity_command_mapper_(options) {
  Configure(options);
}

void Tracker::Configure(const proto::TrackOptions& options) {
  std::lock_guard<std::mutex> lock(mutex_);
  options_ = options;
  ApplyDefaults(&options_);
  if (!options_.base_frame().empty()) {
    robot_frame_id_ = options_.base_frame();
  }
  lattice_.Configure(options_);
  velocity_command_mapper_.Configure(options_);

  std::string load_error;
  is_network_loaded_ = inference_engine_.LoadModel(options_, &load_error);
  if (!is_network_loaded_) {
    if (options_.allow_heuristic_fallback()) {
      AWARN << "Tracker: ONNX unavailable (" << load_error
            << "), using heuristic fallback.";
    } else {
      AERROR << "Tracker: failed to load model: " << load_error;
    }
  }
}

void Tracker::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  has_previous_target_ = false;
  previous_target_yaw_rad_ = 0.0;
  has_depth_image_ = false;
  normalized_depth_.clear();
}

void Tracker::UpdateOdometry(
    const automsgs::msgs::nav_msgs::Odometry& odometry) {
  std::lock_guard<std::mutex> lock(mutex_);
  robot_linear_velocity_mps_ = odometry.twist().twist().linear().x();
  robot_yaw_rate_rps_ = odometry.twist().twist().angular().z();
  if (!odometry.child_frame_id().empty()) {
    robot_frame_id_ = odometry.child_frame_id();
  }
  has_odometry_ = true;
}

void Tracker::UpdateDepth(
    const automsgs::msgs::sensor_msgs::Image& depth_image) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double max_depth_m =
      options_.radio_range_m() * kDepthNormalizeHorizonFactor;
  has_depth_image_ = NormalizeDepthImage(
      depth_image, options_.image_height(), options_.image_width(),
      options_.depth_scale(), max_depth_m, &normalized_depth_);
}

bool Tracker::CapturePlanningSnapshot(PlanningSnapshot* snapshot) const {
  if (snapshot == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_depth_image_) {
    return false;
  }

  snapshot->normalized_depth = normalized_depth_;
  snapshot->options = options_;
  snapshot->lattice = lattice_;
  snapshot->velocity_command_mapper = velocity_command_mapper_;
  snapshot->robot_frame_id = robot_frame_id_;
  snapshot->use_network = is_network_loaded_;
  snapshot->allow_heuristic_fallback = options_.allow_heuristic_fallback();
  snapshot->has_previous_target = has_previous_target_;
  snapshot->previous_target_yaw_rad = previous_target_yaw_rad_;

  const double linear_velocity_normalized =
      options_.vel_max_mps() > 0
          ? robot_linear_velocity_mps_ / options_.vel_max_mps()
          : 0.0;
  const double yaw_rate_normalized =
      options_.wz_max_rps() > 0 ? robot_yaw_rate_rps_ / options_.wz_max_rps()
                               : 0.0;
  double goal_x = 1.0;
  double goal_y = 0.0;
  if (has_previous_target_) {
    goal_x = std::cos(previous_target_yaw_rad_);
    goal_y = std::sin(previous_target_yaw_rad_);
  }
  snapshot->robot_state = {
      static_cast<float>(linear_velocity_normalized),
      static_cast<float>(yaw_rate_normalized), static_cast<float>(goal_x),
      static_cast<float>(goal_y)};
  return true;
}

bool Tracker::EstimateHypothesesFromDepthSectors(
    const PlanningSnapshot& snapshot,
    std::vector<MotionPrimitiveHypothesis>* hypotheses) const {
  if (hypotheses == nullptr || snapshot.normalized_depth.empty()) {
    return false;
  }
  const int image_height = snapshot.options.image_height();
  const int image_width = snapshot.options.image_width();
  const int lattice_size = snapshot.lattice.size();
  if (image_height <= 0 || image_width <= 0 || lattice_size <= 0) {
    return false;
  }
  hypotheses->assign(static_cast<size_t>(lattice_size),
                     MotionPrimitiveHypothesis{});

  const double desired_standoff_m = snapshot.options.follow_distance_m();
  const double field_of_view_rad =
      snapshot.options.horizon_camera_fov_deg() * M_PI / 180.0;
  const int row_begin =
      static_cast<int>(image_height * kHeuristicRowBandLowFraction);
  const int row_end =
      static_cast<int>(image_height * kHeuristicRowBandHighFraction);

  for (int lattice_index = 0; lattice_index < lattice_size; ++lattice_index) {
    const auto& anchor = snapshot.lattice.anchor(lattice_index);
    const double normalized_column =
        0.5 + anchor.yaw_rad / field_of_view_rad;
    const int column = std::clamp(
        static_cast<int>(normalized_column * image_width), 0, image_width - 1);

    float nearest_normalized_depth = 1.0f;
    for (int row = row_begin; row < row_end; ++row) {
      const float depth_value = snapshot.normalized_depth[static_cast<size_t>(
          row * image_width + column)];
      if (depth_value > kHeuristicMinValidNormalizedDepth &&
          depth_value < nearest_normalized_depth) {
        nearest_normalized_depth = depth_value;
      }
    }

    const double range_m =
        static_cast<double>(nearest_normalized_depth) *
        snapshot.options.radio_range_m() * kDepthNormalizeHorizonFactor;
    MotionPrimitiveHypothesis hypothesis;
    hypothesis.lattice_index = lattice_index;
    hypothesis.terminal_x_m = std::cos(anchor.yaw_rad) * range_m;
    hypothesis.terminal_y_m = std::sin(anchor.yaw_rad) * range_m;
    const double range_error_m = std::abs(range_m - desired_standoff_m);
    hypothesis.trajectory_cost =
        range_error_m + kHeuristicYawCostWeight * std::abs(anchor.yaw_rad);
    const bool looks_like_person =
        nearest_normalized_depth < kHeuristicMaxPersonNormalizedDepth &&
        range_m > kHeuristicMinPersonRangeM &&
        range_m < desired_standoff_m * kHeuristicMaxPersonRangeStandoffFactor;
    hypothesis.objectness_score =
        looks_like_person ? std::exp(-range_error_m)
                          : kHeuristicBackgroundObjectness;
    (*hypotheses)[static_cast<size_t>(lattice_index)] = hypothesis;
  }
  return true;
}

bool Tracker::BuildHypotheses(
    const PlanningSnapshot& snapshot,
    std::vector<MotionPrimitiveHypothesis>* hypotheses) {
  std::string inference_error;
  if (snapshot.use_network) {
    if (inference_engine_.RunInference(
            snapshot.normalized_depth, snapshot.robot_state, snapshot.lattice,
            hypotheses, &inference_error)) {
      return true;
    }
    AWARN << "Tracker inference failed: " << inference_error;
    if (!snapshot.allow_heuristic_fallback) {
      return false;
    }
  } else if (!snapshot.allow_heuristic_fallback) {
    return false;
  }
  return EstimateHypothesesFromDepthSectors(snapshot, hypotheses);
}

bool Tracker::RunPlanningCycle(TrackResult* result) {
  if (result == nullptr) {
    return false;
  }
  *result = TrackResult{};
  if (is_paused_.load()) {
    return false;
  }

  PlanningSnapshot snapshot;
  if (!CapturePlanningSnapshot(&snapshot)) {
    return false;
  }

  std::vector<MotionPrimitiveHypothesis> hypotheses;
  if (!BuildHypotheses(snapshot, &hypotheses)) {
    return false;
  }

  FollowSelectionOptions selection_options;
  selection_options.objectness_threshold =
      snapshot.options.objectness_threshold();
  selection_options.suppression_angle_rad =
      snapshot.options.nms_angle_deg() * M_PI / 180.0;

  const int selected_index = SelectFollowHypothesis(
      hypotheses, selection_options, snapshot.has_previous_target,
      snapshot.previous_target_yaw_rad);
  result->hypotheses = std::move(hypotheses);
  result->velocity_command.mutable_header()->set_frame_id(
      snapshot.robot_frame_id);

  if (selected_index < 0) {
    result->has_target = false;
    return true;
  }

  result->has_target = true;
  result->selected_lattice_index = selected_index;
  result->selected = result->hypotheses[static_cast<size_t>(selected_index)];
  result->velocity_command =
      snapshot.velocity_command_mapper.ToVelocityCommand(
          result->selected, snapshot.robot_frame_id,
          &result->debug_path_xy_m);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    previous_target_yaw_rad_ = std::atan2(result->selected.terminal_y_m,
                                          result->selected.terminal_x_m);
    has_previous_target_ = true;
  }
  return true;
}

}  // namespace autonomy::perception::track
