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

#include "autonomy/control/controller/mppi_controller/tools/path_handler.hpp"

#include <algorithm>
#include "autolink/common/log.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/control/controller/mppi_controller/tools/utils.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {
namespace tools {

namespace {

bool isClosedLoopPath(const commsgs::planning_msgs::Path& plan) {
  if (plan.poses.size() < 4) {
    return false;
  }
  const auto& a = plan.poses.front().pose.position;
  const auto& b = plan.poses.back().pose.position;
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  constexpr double kClosureTolerance = 0.25;
  return (dx * dx + dy * dy) < kClosureTolerance * kClosureTolerance;
}

PathIterator nextPathIterator(PathIterator current, PathIterator begin, PathIterator end,
                              const bool wrap) {
  auto next = std::next(current);
  if (next == end && wrap) {
    return begin;
  }
  return next;
}

double integratedPathLength(PathIterator begin, PathIterator end) {
  using map::costmap_2d::utils::euclidean_distance;
  if (begin == end) {
    return 0.0;
  }
  double length = 0.0;
  for (auto it = begin; std::next(it) != end; ++it) {
    length += euclidean_distance(*it, *std::next(it));
  }
  return length;
}

PathIterator advanceAlongPath(PathIterator current, PathIterator begin, PathIterator end,
                              const double distance, const bool wrap) {
  if (begin == end || distance <= 0.0) {
    return current;
  }

  using map::costmap_2d::utils::euclidean_distance;
  double remaining = distance;
  while (remaining > 1e-6) {
    PathIterator next = nextPathIterator(current, begin, end, wrap);
    if (next == end || next == current) {
      return current;
    }

    const double segment = euclidean_distance(*current, *next);
    if (segment >= remaining) {
      return next;
    }
    remaining -= segment;
    current = next;
    if (!wrap && std::next(current) == end) {
      return current;
    }
  }
  return current;
}

}  // namespace

void PathHandler::initialize(std::shared_ptr<autolink::Node> parent, const std::string& name,
                             std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
                             std::shared_ptr<autonomy::transform::Buffer> buffer,
                             const proto::MPPIControllerOptions* options) {
  name_ = name;
  costmap_ = costmap;
  tf_buffer_ = buffer;
  options_ = options;

  constexpr double kDefaultPruneDistance = 1.5;
  constexpr double kDefaultTransformTolerance = 0.1;
  constexpr double kDefaultInversionXyTolerance = 0.2;
  constexpr double kDefaultInversionYawTolerance = 0.4;

  if (options_ && options_->has_path_handler()) {
    const auto& ph = options_->path_handler();
    max_robot_pose_search_dist_ = ph.max_robot_pose_search_dist() > 0.0
                                      ? ph.max_robot_pose_search_dist()
                                      : getMaxCostmapDist();
    prune_distance_ =
        ph.prune_distance() > 0.0 ? ph.prune_distance() : kDefaultPruneDistance;
    transform_tolerance_ = ph.transform_tolerance() > 0.0
                               ? ph.transform_tolerance()
                               : kDefaultTransformTolerance;
    enforce_path_inversion_ = ph.enforce_path_inversion();
    inversion_xy_tolerance_ =
        ph.inversion_xy_tolerance() > 0.0 ? ph.inversion_xy_tolerance()
                                          : kDefaultInversionXyTolerance;
    inversion_yaw_tolerance =
        ph.inversion_yaw_tolerance() > 0.0 ? ph.inversion_yaw_tolerance()
                                           : kDefaultInversionYawTolerance;
  } else {
    max_robot_pose_search_dist_ = getMaxCostmapDist();
    prune_distance_ = kDefaultPruneDistance;
    transform_tolerance_ = kDefaultTransformTolerance;
    enforce_path_inversion_ = false;
    inversion_xy_tolerance_ = kDefaultInversionXyTolerance;
    inversion_yaw_tolerance = kDefaultInversionYawTolerance;
  }

  if (enforce_path_inversion_) {
    inversion_locale_ = 0u;
  }

  AINFO << "PathHandler: max_robot_pose_search_dist=" << max_robot_pose_search_dist_
        << " prune_distance=" << prune_distance_
        << " transform_tolerance=" << transform_tolerance_;
}

std::pair<commsgs::planning_msgs::Path, PathIterator> PathHandler::getGlobalPlanConsideringBoundsInCostmapFrame(
    const commsgs::geometry_msgs::PoseStamped& global_pose) {
  using map::costmap_2d::utils::euclidean_distance;

  auto begin = global_plan_up_to_inversion_.poses.begin();
  auto end = global_plan_up_to_inversion_.poses.end();

  auto closest_point = begin;
  if (closed_loop_path_) {
    auto search_begin =
        (last_closest_point_ >= begin && last_closest_point_ < end) ? last_closest_point_ : begin;
    closest_point = search_begin;
    double best_dist = euclidean_distance(global_pose, *search_begin);
    double accumulated = 0.0;
    auto current = search_begin;

    while (true) {
      const double dist = euclidean_distance(global_pose, *current);
      if (dist < best_dist) {
        best_dist = dist;
        closest_point = current;
      }

      auto next = nextPathIterator(current, begin, end, true);
      if (next == current) {
        break;
      }

      accumulated += euclidean_distance(*current, *next);
      if (accumulated > max_robot_pose_search_dist_ && current != search_begin) {
        break;
      }
      current = next;
      if (current == search_begin) {
        break;
      }
    }
  } else {
    // Limit the search for the closest pose up to max_robot_pose_search_dist on
    // the path
    auto closest_pose_upper_bound = map::costmap_2d::utils::first_after_integrated_distance(
        begin, end, max_robot_pose_search_dist_);

    // Find closest point to the robot
    closest_point = map::costmap_2d::utils::min_by(
        begin, closest_pose_upper_bound, [&global_pose](const commsgs::geometry_msgs::PoseStamped& ps) {
          return euclidean_distance(global_pose, ps);
        });

    // Closed loops have start/end poses that are spatially close. When the robot
    // returns to the loop start, min_by may pick a terminal pose and leave only a
    // tiny tail segment for pruning / transformation, preventing the next lap
    // from continuing smoothly. Prefer the first pose if both ends are similarly
    // close and the chosen closest point is biased toward the tail.
    if (global_plan_up_to_inversion_.poses.size() > 3) {
      const double closure_eps = 0.2;
      const double d_start =
          euclidean_distance(global_pose, global_plan_up_to_inversion_.poses.front());
      const double d_end =
          euclidean_distance(global_pose, global_plan_up_to_inversion_.poses.back());
      if (d_start < closure_eps && d_end < closure_eps) {
        const auto dist_from_begin = static_cast<size_t>(
            std::distance(global_plan_up_to_inversion_.poses.begin(), closest_point));
        const auto dist_from_end = static_cast<size_t>(
            std::distance(closest_point, global_plan_up_to_inversion_.poses.end()));
        if (dist_from_end <= dist_from_begin) {
          closest_point = global_plan_up_to_inversion_.poses.begin();
        }
      }
    }
  }

  commsgs::planning_msgs::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_->getGlobalFrameID();
  transformed_plan.header.stamp = global_pose.header.stamp;

  unsigned int mx, my;
  // Find the furthest relevant pose on the path to consider within costmap
  // bounds
  // Transforming it to the costmap frame in the same loop
  double accumulated = 0.0;
  auto global_plan_pose = closest_point;
  while (global_plan_pose != end) {
    // Transform from global plan frame to costmap frame
    commsgs::geometry_msgs::PoseStamped costmap_plan_pose;
    global_plan_pose->header.stamp = global_pose.header.stamp;
    global_plan_pose->header.frame_id = global_plan_.header.frame_id;
    transformPose(costmap_->getGlobalFrameID(), *global_plan_pose, costmap_plan_pose);

    // Check if pose is inside the costmap
    if (!costmap_->getCostmap()->worldToMap(costmap_plan_pose.pose.position.x, costmap_plan_pose.pose.position.y, mx,
                                            my)) {
      return std::make_pair(transformed_plan, closest_point);
    }

    // Filling the transformed plan to return with the transformed pose
    transformed_plan.poses.push_back(costmap_plan_pose);

    auto next = nextPathIterator(global_plan_pose, begin, end, closed_loop_path_);
    if (next == end || next == closest_point) {
      break;
    }

    accumulated += euclidean_distance(*global_plan_pose, *next);
    const double local_prune_distance =
        closed_loop_path_ ? std::max(prune_distance_, 3.0) : prune_distance_;
    if (accumulated > local_prune_distance) {
      break;
    }
    global_plan_pose = next;
  }

  return {transformed_plan, closest_point};
}

commsgs::geometry_msgs::PoseStamped PathHandler::transformToGlobalPlanFrame(
    const commsgs::geometry_msgs::PoseStamped& pose) {
  if (global_plan_up_to_inversion_.poses.empty()) {
    throw common::InvalidPath("Received plan with zero length");
  }

  commsgs::geometry_msgs::PoseStamped robot_pose;
  if (!transformPose(global_plan_up_to_inversion_.header.frame_id, pose, robot_pose)) {
    throw common::ControllerTFError("Unable to transform robot pose into global plan's frame");
  }

  return robot_pose;
}

commsgs::planning_msgs::Path PathHandler::transformPath(const commsgs::geometry_msgs::PoseStamped& robot_pose) {
  // Find relevant bounds of path to use
  commsgs::geometry_msgs::PoseStamped global_pose = transformToGlobalPlanFrame(robot_pose);
  auto [transformed_plan, lower_bound] = getGlobalPlanConsideringBoundsInCostmapFrame(global_pose);
  last_closest_point_ = lower_bound;

  if (!closed_loop_path_) {
    prunePlan(global_plan_up_to_inversion_, lower_bound);
  }

  if (enforce_path_inversion_ && inversion_locale_ != 0u) {
    if (isWithinInversionTolerances(global_pose)) {
      prunePlan(global_plan_, global_plan_.poses.begin() + inversion_locale_);
      global_plan_up_to_inversion_ = global_plan_;
      inversion_locale_ = tools::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
    }
  }

  if (transformed_plan.poses.empty()) {
    throw common::InvalidPath("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool PathHandler::transformPose(const std::string& frame, const commsgs::geometry_msgs::PoseStamped& in_pose,
                                commsgs::geometry_msgs::PoseStamped& out_pose) const {
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_buffer_->transform(in_pose, out_pose, frame, static_cast<float>(transform_tolerance_));
    out_pose.header.frame_id = frame;
    return true;
  } catch (autonomy::transform::tf2::TransformException& ex) {
    AERROR << "Exception in transformPose: " << ex.what();
  }
  return false;
}

double PathHandler::getMaxCostmapDist() {
  const auto& costmap = costmap_->getCostmap();
  return static_cast<double>(std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY())) *
         costmap->getResolution() * 0.50;
}

void PathHandler::setPath(const commsgs::planning_msgs::Path& plan) {
  global_plan_ = plan;
  global_plan_up_to_inversion_ = global_plan_;
  closed_loop_path_ = isClosedLoopPath(global_plan_);
  last_closest_point_ = global_plan_up_to_inversion_.poses.begin();
  if (closed_loop_path_) {
    const double perimeter = integratedPathLength(global_plan_up_to_inversion_.poses.begin(),
                                                  global_plan_up_to_inversion_.poses.end());
    // Keep virtual goal far enough along the path so near-goal critics do not
    // treat the spatially coincident start/end as a terminal stop condition.
    closed_loop_goal_lookahead_ =
        std::clamp(perimeter * 0.35, 3.0, std::max(3.0, perimeter - 1.0));
  }
  if (enforce_path_inversion_) {
    inversion_locale_ = tools::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
  }
  if (closed_loop_path_) {
    AINFO << "PathHandler: closed-loop path detected (" << global_plan_.poses.size()
          << " poses), pruning disabled, goal lookahead="
          << closed_loop_goal_lookahead_ << " m";
  }
}

commsgs::planning_msgs::Path& PathHandler::getPath() { return global_plan_; }

void PathHandler::prunePlan(commsgs::planning_msgs::Path& plan, const PathIterator end) {
  plan.poses.erase(plan.poses.begin(), end);
}

commsgs::geometry_msgs::PoseStamped PathHandler::getTransformedGoal(const commsgs::builtin_interfaces::Time& stamp) {
  commsgs::geometry_msgs::PoseStamped goal;
  if (closed_loop_path_ && !global_plan_up_to_inversion_.poses.empty()) {
    const auto begin = global_plan_up_to_inversion_.poses.begin();
    const auto end = global_plan_up_to_inversion_.poses.end();
    const auto anchor =
        (last_closest_point_ >= begin && last_closest_point_ < end) ? last_closest_point_ : begin;
    const auto goal_it =
        advanceAlongPath(anchor, begin, end, closed_loop_goal_lookahead_, true);
    goal = *goal_it;
  } else {
    goal = global_plan_.poses.back();
  }
  goal.header.frame_id = global_plan_.header.frame_id;
  goal.header.stamp = stamp;
  if (goal.header.frame_id.empty()) {
    throw common::ControllerTFError("Goal pose has an empty frame_id");
  }
  commsgs::geometry_msgs::PoseStamped transformed_goal;
  if (!transformPose(costmap_->getGlobalFrameID(), goal, transformed_goal)) {
    throw common::ControllerTFError("Unable to transform goal pose into costmap frame");
  }
  return transformed_goal;
}

bool PathHandler::isWithinInversionTolerances(const commsgs::geometry_msgs::PoseStamped& robot_pose) {
  // Keep full path if we are within tolerance of the inversion pose
  const auto last_pose = global_plan_up_to_inversion_.poses.back();
  float distance = hypotf(robot_pose.pose.position.x - last_pose.pose.position.x,
                          robot_pose.pose.position.y - last_pose.pose.position.y);

  float angle_distance = tools::shortest_angular_distance(autonomy::transform::tf2::getYaw(robot_pose.pose.orientation),
                                                          autonomy::transform::tf2::getYaw(last_pose.pose.orientation));

  return distance <= inversion_xy_tolerance_ && fabs(angle_distance) <= inversion_yaw_tolerance;
}

}  // namespace tools
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy