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

#include "autonomy/control/plugins/pure_pursuit_controller/path_handler.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"

namespace autonomy {
namespace control {
namespace plugins {

PathHandler::PathHandler(
  transform::tf2::Duration transform_tolerance,
  std::shared_ptr<TfBuffer> tf,
  std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper)
: transform_tolerance_(transform_tolerance), tf_(tf), costmap_wrapper_(costmap_wrapper)
{
}

double PathHandler::getCostmapMaxExtent() const
{
    const double max_costmap_dim_meters = std::max(
        costmap_wrapper_->getCostmap()->getSizeInMetersX(),
        costmap_wrapper_->getCostmap()->getSizeInMetersY());
    return max_costmap_dim_meters / 2.0;
}

commsgs::planning_msgs::Path PathHandler::transformGlobalPlan(
  const commsgs::geometry_msgs::PoseStamped & pose,
  double max_robot_pose_search_dist,
  bool reject_unit_path)
{
    if (global_plan_.poses.empty()) {
        throw common::InvalidPath("Received plan with zero length");
    }

    if (reject_unit_path && global_plan_.poses.size() == 1) {
        throw common::InvalidPath("Received plan with length of one");
    }

    // let's get the pose of the robot in the frame of the plan
    commsgs::geometry_msgs::PoseStamped robot_pose;
    if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
        throw common::ControllerTFError("Unable to transform robot pose into global plan's frame");
    }

    auto closest_pose_upper_bound =
        map::costmap_2d::utils::first_after_integrated_distance(
        global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist);

    // First find the closest pose on the path to the robot
    // bounded by when the path turns around (if it does) so we don't get a pose from a later
    // portion of the path
    auto transformation_begin =
        map::costmap_2d::utils::min_by(
        global_plan_.poses.begin(), closest_pose_upper_bound,
        [&robot_pose](const commsgs::geometry_msgs::PoseStamped & ps) {
            return map::costmap_2d::utils::euclidean_distance(robot_pose, ps);
        });

    // Make sure we always have at least 2 points on the transformed plan and that we don't prune
    // the global plan below 2 points in order to have always enough point to interpolate the
    // end of path direction
    if (global_plan_.poses.begin() != closest_pose_upper_bound && global_plan_.poses.size() > 1 &&
        transformation_begin == std::prev(closest_pose_upper_bound))
    {
        transformation_begin = std::prev(std::prev(closest_pose_upper_bound));
    }

    // We'll discard points on the plan that are outside the local costmap
    const double max_costmap_extent = getCostmapMaxExtent();
    auto transformation_end = std::find_if(
        transformation_begin, global_plan_.poses.end(),
        [&](const auto & global_plan_pose) {
        return map::costmap_2d::utils::euclidean_distance(global_plan_pose, robot_pose) > max_costmap_extent;
        });

    // Lambda to transform a PoseStamped from global frame to local
    auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
        commsgs::geometry_msgs::PoseStamped stamped_pose, transformed_pose;
        stamped_pose.header.frame_id = global_plan_.header.frame_id;
        stamped_pose.header.stamp = robot_pose.header.stamp;
        stamped_pose.pose = global_plan_pose.pose;
        if (!transformPose(costmap_wrapper_->getBaseFrameID(), stamped_pose, transformed_pose)) {
            throw common::ControllerTFError("Unable to transform plan pose into local frame");
        }
        transformed_pose.pose.position.z = 0.0;
        return transformed_pose;
    };

    // Transform the near part of the global plan into the robot's frame of reference.
    commsgs::planning_msgs::Path transformed_plan;
    std::transform(
        transformation_begin, transformation_end,
        std::back_inserter(transformed_plan.poses),
        transformGlobalPoseToLocal);
    transformed_plan.header.frame_id = costmap_wrapper_->getBaseFrameID();
    transformed_plan.header.stamp = robot_pose.header.stamp;

    // Remove the portion of the global plan that we've already passed so we don't
    // process it on the next iteration (this is called path pruning)
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);

    if (transformed_plan.poses.empty()) {
        throw common::InvalidPath("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
}

bool PathHandler::transformPose(
  const std::string frame,
  const commsgs::geometry_msgs::PoseStamped& in_pose,
  commsgs::geometry_msgs::PoseStamped& out_pose) const
{
    if (in_pose.header.frame_id == frame) {
        out_pose = in_pose;
        return true;
    }

    try {
        // tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
        out_pose.header.frame_id = frame;
        return true;
    } catch (transform::tf2::TransformException & ex) {
        LOG(ERROR) << "Exception in transformPose: " << ex.what();
    }
    return false;
}

}  // namespace plugins
}  // namespace control
}  // namespace autonomy