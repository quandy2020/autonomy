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

void PathHandler::initialize(
    std::shared_ptr<autolink::Node> parent, const std::string& name,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
    std::shared_ptr<autonomy::transform::Buffer> buffer,
    const proto::MPPIControllerOptions* options) {
    name_ = name;
    costmap_ = costmap;
    tf_buffer_ = buffer;
    options_ = options;

    // Load parameters from proto options if needed
    // Most path handler parameters are not in proto yet, use defaults
    // getParam(max_robot_pose_search_dist_, "max_robot_pose_search_dist",
    // getMaxCostmapDist()); getParam(prune_distance_, "prune_distance", 1.5);
    // getParam(transform_tolerance_, "transform_tolerance", 0.1);
    // getParam(enforce_path_inversion_, "enforce_path_inversion", false);
    // if (enforce_path_inversion_) {
    //     getParam(inversion_xy_tolerance_, "inversion_xy_tolerance", 0.2);
    //     getParam(inversion_yaw_tolerance, "inversion_yaw_tolerance", 0.4);
    //     inversion_locale_ = 0u;
    // }
}

std::pair<commsgs::planning_msgs::Path, PathIterator>
PathHandler::getGlobalPlanConsideringBoundsInCostmapFrame(
    const commsgs::geometry_msgs::PoseStamped& global_pose) {
    using map::costmap_2d::utils::euclidean_distance;

    auto begin = global_plan_up_to_inversion_.poses.begin();

    // Limit the search for the closest pose up to max_robot_pose_search_dist on
    // the path
    auto closest_pose_upper_bound =
        map::costmap_2d::utils::first_after_integrated_distance(
            global_plan_up_to_inversion_.poses.begin(),
            global_plan_up_to_inversion_.poses.end(),
            max_robot_pose_search_dist_);

    // Find closest point to the robot
    auto closest_point = map::costmap_2d::utils::min_by(
        begin, closest_pose_upper_bound,
        [&global_pose](const commsgs::geometry_msgs::PoseStamped& ps) {
            return euclidean_distance(global_pose, ps);
        });

    commsgs::planning_msgs::Path transformed_plan;
    transformed_plan.header.frame_id = costmap_->getGlobalFrameID();
    transformed_plan.header.stamp = global_pose.header.stamp;

    auto pruned_plan_end =
        map::costmap_2d::utils::first_after_integrated_distance(
            closest_point, global_plan_up_to_inversion_.poses.end(),
            prune_distance_);

    unsigned int mx, my;
    // Find the furthest relevant pose on the path to consider within costmap
    // bounds
    // Transforming it to the costmap frame in the same loop
    for (auto global_plan_pose = closest_point;
         global_plan_pose != pruned_plan_end; ++global_plan_pose) {
        // Transform from global plan frame to costmap frame
        commsgs::geometry_msgs::PoseStamped costmap_plan_pose;
        global_plan_pose->header.stamp = global_pose.header.stamp;
        global_plan_pose->header.frame_id = global_plan_.header.frame_id;
        transformPose(costmap_->getGlobalFrameID(), *global_plan_pose,
                      costmap_plan_pose);

        // Check if pose is inside the costmap
        if (!costmap_->getCostmap()->worldToMap(
                costmap_plan_pose.pose.position.x,
                costmap_plan_pose.pose.position.y, mx, my)) {
            return std::make_pair(transformed_plan, closest_point);
        }

        // Filling the transformed plan to return with the transformed pose
        transformed_plan.poses.push_back(costmap_plan_pose);
    }

    return {transformed_plan, closest_point};
}

commsgs::geometry_msgs::PoseStamped PathHandler::transformToGlobalPlanFrame(
    const commsgs::geometry_msgs::PoseStamped& pose) {
    if (global_plan_up_to_inversion_.poses.empty()) {
        throw common::InvalidPath("Received plan with zero length");
    }

    commsgs::geometry_msgs::PoseStamped robot_pose;
    if (!transformPose(global_plan_up_to_inversion_.header.frame_id, pose,
                       robot_pose)) {
        throw common::ControllerTFError(
            "Unable to transform robot pose into global plan's frame");
    }

    return robot_pose;
}

commsgs::planning_msgs::Path PathHandler::transformPath(
    const commsgs::geometry_msgs::PoseStamped& robot_pose) {
    // Find relevant bounds of path to use
    commsgs::geometry_msgs::PoseStamped global_pose =
        transformToGlobalPlanFrame(robot_pose);
    auto [transformed_plan, lower_bound] =
        getGlobalPlanConsideringBoundsInCostmapFrame(global_pose);

    prunePlan(global_plan_up_to_inversion_, lower_bound);

    if (enforce_path_inversion_ && inversion_locale_ != 0u) {
        if (isWithinInversionTolerances(global_pose)) {
            prunePlan(global_plan_,
                      global_plan_.poses.begin() + inversion_locale_);
            global_plan_up_to_inversion_ = global_plan_;
            inversion_locale_ = tools::removePosesAfterFirstInversion(
                global_plan_up_to_inversion_);
        }
    }

    if (transformed_plan.poses.empty()) {
        throw common::InvalidPath("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
}

bool PathHandler::transformPose(
    const std::string& frame,
    const commsgs::geometry_msgs::PoseStamped& in_pose,
    commsgs::geometry_msgs::PoseStamped& out_pose) const {
    if (in_pose.header.frame_id == frame) {
        out_pose = in_pose;
        return true;
    }

    try {
        tf_buffer_->transform(in_pose, out_pose, frame,
                              static_cast<float>(transform_tolerance_));
        out_pose.header.frame_id = frame;
        return true;
    } catch (autonomy::transform::tf2::TransformException& ex) {
        AERROR << "Exception in transformPose: " << ex.what();
    }
    return false;
}

double PathHandler::getMaxCostmapDist() {
    const auto& costmap = costmap_->getCostmap();
    return static_cast<double>(std::max(costmap->getSizeInCellsX(),
                                        costmap->getSizeInCellsY())) *
           costmap->getResolution() * 0.50;
}

void PathHandler::setPath(const commsgs::planning_msgs::Path& plan) {
    global_plan_ = plan;
    global_plan_up_to_inversion_ = global_plan_;
    if (enforce_path_inversion_) {
        inversion_locale_ =
            tools::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
    }
}

commsgs::planning_msgs::Path& PathHandler::getPath() {
    return global_plan_;
}

void PathHandler::prunePlan(commsgs::planning_msgs::Path& plan,
                            const PathIterator end) {
    plan.poses.erase(plan.poses.begin(), end);
}

commsgs::geometry_msgs::PoseStamped PathHandler::getTransformedGoal(
    const commsgs::builtin_interfaces::Time& stamp) {
    auto goal = global_plan_.poses.back();
    goal.header.frame_id = global_plan_.header.frame_id;
    goal.header.stamp = stamp;
    if (goal.header.frame_id.empty()) {
        throw common::ControllerTFError("Goal pose has an empty frame_id");
    }
    commsgs::geometry_msgs::PoseStamped transformed_goal;
    if (!transformPose(costmap_->getGlobalFrameID(), goal, transformed_goal)) {
        throw common::ControllerTFError(
            "Unable to transform goal pose into costmap frame");
    }
    return transformed_goal;
}

bool PathHandler::isWithinInversionTolerances(
    const commsgs::geometry_msgs::PoseStamped& robot_pose) {
    // Keep full path if we are within tolerance of the inversion pose
    const auto last_pose = global_plan_up_to_inversion_.poses.back();
    float distance =
        hypotf(robot_pose.pose.position.x - last_pose.pose.position.x,
               robot_pose.pose.position.y - last_pose.pose.position.y);

    float angle_distance = tools::shortest_angular_distance(
        autonomy::transform::tf2::getYaw(robot_pose.pose.orientation),
        autonomy::transform::tf2::getYaw(last_pose.pose.orientation));

    return distance <= inversion_xy_tolerance_ &&
           fabs(angle_distance) <= inversion_yaw_tolerance;
}

}  // namespace tools
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy