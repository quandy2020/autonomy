/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/control/controller/mppi_controller/tools/trajectory_visualizer.hpp"

#include <algorithm>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/visualization_msgs/marker.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>
#include "autonomy/transform/tf2/LinearMath/Quaternion.h"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {
namespace tools {

using Marker = automsgs::msgs::visualization_msgs::Marker;

namespace {

void HorizonColor(float t, float shade, float* r, float* g, float* b,
                  float* a) {
  t = std::clamp(t, 0.f, 1.f);
  shade = std::clamp(shade, 0.f, 1.f);
  // Cool family aligned with GlobalPlan: #6BA8FF → #3DD6F5 → #5EF0B0
  if (t < 0.5f) {
    const float u = t * 2.f;
    *r = 0.42f + (0.24f - 0.42f) * u;
    *g = 0.66f + (0.84f - 0.66f) * u;
    *b = 1.00f + (0.96f - 1.00f) * u;
  } else {
    const float u = (t - 0.5f) * 2.f;
    *r = 0.24f + (0.37f - 0.24f) * u;
    *g = 0.84f + (0.94f - 0.84f) * u;
    *b = 0.96f + (0.69f - 0.96f) * u;
  }
  *r *= shade;
  *g *= shade;
  *b *= shade;
  *a = 0.28f + 0.45f * t;
}

}  // namespace

TrajectoryVisualizer::TrajectoryVisualizer(
    std::shared_ptr<autolink::Node> parent, const std::string& name,
    const std::string& frame_id, const proto::MPPIControllerOptions* options) {
  frame_id_ = frame_id;
  if (!parent) {
    owned_node_ = autolink::CreateNode(name + "_mppi_viz");
    parent = owned_node_;
  }
  const std::string prefix =
      (!name.empty() && name.front() == '/') ? name : ("/" + name);
  trajectories_publisher_ =
      parent->CreateWriter<automsgs::msgs::visualization_msgs::MarkerArray>(
          prefix + "/candidate_trajectories");
  transformed_path_pub_ = parent->CreateWriter<automsgs::msgs::nav_msgs::Path>(
      prefix + "/transformed_global_plan");
  optimal_path_pub_ = parent->CreateWriter<automsgs::msgs::nav_msgs::Path>(
      prefix + "/optimal_path");
  options_ = options;

  if (options && options->has_trajectory_visualizer()) {
    trajectory_step_ = std::max<size_t>(
        1, options->trajectory_visualizer().trajectory_step());
    time_step_ =
        std::max<size_t>(1, options->trajectory_visualizer().time_step());
  } else {
    trajectory_step_ = 5;
    time_step_ = 3;
  }

  reset();
}

bool TrajectoryVisualizer::hasSubscribers() const {
  return (trajectories_publisher_ && trajectories_publisher_->HasReader()) ||
         (optimal_path_pub_ && optimal_path_pub_->HasReader()) ||
         (transformed_path_pub_ && transformed_path_pub_->HasReader());
}

void TrajectoryVisualizer::add(
    const Eigen::ArrayXXf& trajectory, const std::string& /*marker_namespace*/,
    const automsgs::msgs::builtin_interfaces::Time& cmd_stamp) {
  if (!optimal_path_pub_ || !optimal_path_pub_->HasReader()) {
    return;
  }
  const size_t size = static_cast<size_t>(trajectory.rows());
  if (size == 0) {
    return;
  }

  *optimal_path_->mutable_header()->mutable_stamp() = cmd_stamp;
  optimal_path_->mutable_header()->set_frame_id(frame_id_);
  optimal_path_->mutable_poses()->Clear();
  optimal_path_->mutable_poses()->Reserve(static_cast<int>(size));

  for (size_t i = 0; i < size; ++i) {
    automsgs::msgs::geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.mutable_header()->set_frame_id(frame_id_);
    *pose_stamped.mutable_header()->mutable_stamp() = cmd_stamp;
    pose_stamped.mutable_pose()->mutable_position()->set_x(trajectory(i, 0));
    pose_stamped.mutable_pose()->mutable_position()->set_y(trajectory(i, 1));
    pose_stamped.mutable_pose()->mutable_position()->set_z(0.06);

    autonomy::transform::tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(0., 0., trajectory(i, 2));
    pose_stamped.mutable_pose()->mutable_orientation()->set_x(
        quaternion_tf2.x());
    pose_stamped.mutable_pose()->mutable_orientation()->set_y(
        quaternion_tf2.y());
    pose_stamped.mutable_pose()->mutable_orientation()->set_z(
        quaternion_tf2.z());
    pose_stamped.mutable_pose()->mutable_orientation()->set_w(
        quaternion_tf2.w());

    *optimal_path_->mutable_poses()->Add() = std::move(pose_stamped);
  }
}

void TrajectoryVisualizer::add(const models::Trajectories& trajectories,
                               const std::string& marker_namespace) {
  if (!trajectories_publisher_ || !trajectories_publisher_->HasReader()) {
    return;
  }

  const size_t n_rows = static_cast<size_t>(trajectories.x.rows());
  const size_t n_cols = static_cast<size_t>(trajectories.x.cols());
  if (n_rows == 0 || n_cols == 0) {
    return;
  }

  const size_t traj_count = (n_rows + trajectory_step_ - 1) / trajectory_step_;
  const size_t time_count = (n_cols + time_step_ - 1) / time_step_;
  const int point_budget = static_cast<int>(traj_count * time_count);

  points_->mutable_markers()->Clear();

  // Dense POINTS swarm (scoring metaphor via horizon colour).
  auto* points_marker = points_->mutable_markers()->Add();
  points_marker->mutable_header()->set_frame_id(frame_id_);
  points_marker->set_ns(marker_namespace);
  points_marker->set_id(1);
  points_marker->set_type(Marker::POINTS);
  points_marker->set_action(Marker::ADD);
  points_marker->mutable_scale()->set_x(0.045);
  points_marker->mutable_scale()->set_y(0.045);
  points_marker->mutable_scale()->set_z(0.045);
  *points_marker->mutable_color() = tools::createColor(0.24f, 0.84f, 0.96f, 0.55f);
  points_marker->mutable_points()->Reserve(point_budget);
  points_marker->mutable_colors()->Reserve(point_budget);

  const float shape_1 = static_cast<float>(std::max<size_t>(1, n_cols - 1));
  const float row_denom =
      static_cast<float>(std::max<size_t>(1, (n_rows - 1) / trajectory_step_));

  // Continuous LINE_STRIP ribbons for a subset — reads as "system thinking".
  constexpr size_t kMaxRibbons = 18;
  const size_t ribbon_stride = std::max<size_t>(
      trajectory_step_,
      (n_rows + kMaxRibbons - 1) / kMaxRibbons);
  int ribbon_id = 10;

  size_t traj_idx = 0;
  for (size_t i = 0; i < n_rows; i += trajectory_step_, ++traj_idx) {
    const float row_t =
        std::clamp(static_cast<float>(traj_idx) / row_denom, 0.f, 1.f);
    const bool emit_ribbon = (i % ribbon_stride) == 0;

    Marker* ribbon = nullptr;
    if (emit_ribbon) {
      ribbon = points_->mutable_markers()->Add();
      ribbon->mutable_header()->set_frame_id(frame_id_);
      ribbon->set_ns(marker_namespace + "/ribbons");
      ribbon->set_id(ribbon_id++);
      ribbon->set_type(Marker::LINE_STRIP);
      ribbon->set_action(Marker::ADD);
      ribbon->mutable_scale()->set_x(0.025);
      *ribbon->mutable_color() = tools::createColor(0.37f, 0.94f, 0.69f, 0.40f);
      ribbon->mutable_points()->Reserve(static_cast<int>(time_count));
      ribbon->mutable_colors()->Reserve(static_cast<int>(time_count));
    }

    for (size_t j = 0; j < n_cols; j += time_step_) {
      const float t =
          std::clamp(static_cast<float>(j) / shape_1, 0.f, 1.f);
      float r = 0.f;
      float g = 0.f;
      float b = 0.f;
      float a = 0.f;
      HorizonColor(t, 0.75f + 0.25f * row_t, &r, &g, &b, &a);

      auto* point = points_marker->mutable_points()->Add();
      point->set_x(trajectories.x(i, j));
      point->set_y(trajectories.y(i, j));
      point->set_z(0.04f);
      auto* color = points_marker->mutable_colors()->Add();
      color->set_r(r);
      color->set_g(g);
      color->set_b(b);
      color->set_a(a);

      if (ribbon != nullptr) {
        auto* rp = ribbon->mutable_points()->Add();
        rp->set_x(trajectories.x(i, j));
        rp->set_y(trajectories.y(i, j));
        rp->set_z(0.035f);
        auto* rc = ribbon->mutable_colors()->Add();
        rc->set_r(r);
        rc->set_g(g);
        rc->set_b(b);
        rc->set_a(std::min(0.85f, a + 0.15f));
      }
    }
  }
}

void TrajectoryVisualizer::reset() {
  marker_id_ = 0;
  if (!points_) {
    points_ =
        std::make_unique<automsgs::msgs::visualization_msgs::MarkerArray>();
  } else {
    points_->mutable_markers()->Clear();
  }
  if (!optimal_path_) {
    optimal_path_ = std::make_unique<automsgs::msgs::nav_msgs::Path>();
  } else {
    optimal_path_->mutable_poses()->Clear();
  }
}

void TrajectoryVisualizer::clearPublished() {
  reset();

  if (trajectories_publisher_ && trajectories_publisher_->HasReader()) {
    auto clear =
        std::make_shared<automsgs::msgs::visualization_msgs::MarkerArray>();
    auto* marker = clear->mutable_markers()->Add();
    marker->mutable_header()->set_frame_id(frame_id_);
    marker->set_ns("");
    marker->set_id(0);
    marker->set_action(Marker::DELETEALL);
    trajectories_publisher_->Write(clear);
  }

  auto empty_path = [this]() {
    auto path = std::make_shared<automsgs::msgs::nav_msgs::Path>();
    path->mutable_header()->set_frame_id(frame_id_);
    return path;
  };
  if (optimal_path_pub_ && optimal_path_pub_->HasReader()) {
    optimal_path_pub_->Write(empty_path());
  }
  if (transformed_path_pub_ && transformed_path_pub_->HasReader()) {
    transformed_path_pub_->Write(empty_path());
  }
}

void TrajectoryVisualizer::visualize(
    const automsgs::msgs::nav_msgs::Path& plan) {
  if (trajectories_publisher_ && trajectories_publisher_->HasReader() &&
      points_ && !points_->markers().empty()) {
    trajectories_publisher_->Write(
        std::shared_ptr<automsgs::msgs::visualization_msgs::MarkerArray>(
            std::move(points_)));
    points_ =
        std::make_unique<automsgs::msgs::visualization_msgs::MarkerArray>();
  } else if (points_) {
    points_->mutable_markers()->Clear();
  }

  if (optimal_path_pub_ && optimal_path_pub_->HasReader() && optimal_path_ &&
      !optimal_path_->poses().empty()) {
    optimal_path_pub_->Write(
        std::shared_ptr<automsgs::msgs::nav_msgs::Path>(std::move(optimal_path_)));
    optimal_path_ = std::make_unique<automsgs::msgs::nav_msgs::Path>();
  } else if (optimal_path_) {
    optimal_path_->mutable_poses()->Clear();
  }

  if (transformed_path_pub_ && transformed_path_pub_->HasReader()) {
    auto plan_ptr = std::make_shared<automsgs::msgs::nav_msgs::Path>(plan);
    transformed_path_pub_->Write(plan_ptr);
  }
}

TrajectoryVisualizer::~TrajectoryVisualizer() = default;

}  // namespace tools
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
