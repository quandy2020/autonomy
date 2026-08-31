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

/**
 * @file path_transformer.cpp
 * @brief Implementation of nmpc_controller::PathTransformer
 */

#include "autonomy/control/controller/nmpc_controller/path_transformer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "autolink/common/log.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {

namespace {

/**
 * @brief Euclidean distance between two stamped poses.
 */
double PoseDistance(const automsgs::msgs::geometry_msgs::PoseStamped& a,
                    const automsgs::msgs::geometry_msgs::PoseStamped& b) {
    return map::costmap_2d::utils::euclidean_distance(a, b);
}
}  // namespace

void PathTransformer::Configure(
    std::shared_ptr<transform::Buffer> tf_buffer,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper,
    const proto::NMPCControllerOptions& options) {
    tf_buffer_ = std::move(tf_buffer);
    costmap_wrapper_ = std::move(costmap_wrapper);
    options_ = options;
}

void PathTransformer::SetPlan(const automsgs::msgs::nav_msgs::Path& plan) {
    global_plan_ = plan;
}

automsgs::msgs::nav_msgs::Path PathTransformer::Transform(
    const automsgs::msgs::geometry_msgs::PoseStamped& robot_pose) const {
    automsgs::msgs::nav_msgs::Path output;
    if (global_plan_.poses().empty() || !costmap_wrapper_ || !tf_buffer_) {
        return output;
    }

    const std::string target_frame = costmap_wrapper_->getGlobalFrameID();
    automsgs::msgs::geometry_msgs::PoseStamped robot_in_plan_frame =
        robot_pose;
    if (!robot_pose.header().frame_id().empty() &&
        robot_pose.header().frame_id() != global_plan_.header().frame_id()) {
        try {
            tf_buffer_->transform(robot_pose, robot_in_plan_frame,
                                  global_plan_.header().frame_id(),
                                  static_cast<float>(options_.transform_tolerance()));
        } catch (const autonomy::transform::tf2::TransformException& ex) {
            AWARN << "NMPC path transform (robot pose): " << ex.what();
            return output;
        }
    }

    const double max_search = options_.max_plan_search_dist() > 0.0
                                  ? options_.max_plan_search_dist()
                                  : 5.0;
    const double prune_distance = options_.prune_distance() > 0.0
                                      ? options_.prune_distance()
                                      : 1.5;

    int closest_idx = 0;
    double best_dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < global_plan_.poses_size(); ++i) {
        const double dist =
            PoseDistance(robot_in_plan_frame, global_plan_.poses(i));
        if (dist < best_dist) {
            best_dist = dist;
            closest_idx = i;
        }
    }

    double accumulated = 0.0;
    int end_idx = closest_idx;
    while (end_idx + 1 < global_plan_.poses_size() && accumulated < max_search) {
        accumulated += PoseDistance(global_plan_.poses(end_idx),
                                    global_plan_.poses(end_idx + 1));
        ++end_idx;
    }

    int start_idx = closest_idx;
    accumulated = 0.0;
    while (start_idx > 0 && accumulated < prune_distance) {
        accumulated += PoseDistance(global_plan_.poses(start_idx - 1),
                                    global_plan_.poses(start_idx));
        --start_idx;
    }

    *output.mutable_header() = global_plan_.header();
    output.mutable_header()->set_frame_id(target_frame);
    for (int i = start_idx; i <= end_idx; ++i) {
        automsgs::msgs::geometry_msgs::PoseStamped transformed;
        const auto& source = global_plan_.poses(i);
        if (source.header().frame_id() == target_frame) {
            transformed = source;
        } else {
            try {
                tf_buffer_->transform(source, transformed, target_frame,
                                      static_cast<float>(
                                          options_.transform_tolerance()));
            } catch (const autonomy::transform::tf2::TransformException& ex) {
                AWARN << "NMPC path transform (waypoint): " << ex.what();
                continue;
            }
        }
        *output.add_poses() = transformed;
    }
    return output;
}

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
