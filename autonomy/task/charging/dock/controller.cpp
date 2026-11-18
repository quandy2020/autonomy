/*
 * Copyright 2026 The Openbot Authors
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
 * @file controller.cpp
 * @brief Park–Kuipers dock controller with optional local-costmap collision
 *        (implementation).
 */

#include "autonomy/task/charging/dock/controller.hpp"

#include <cmath>
#include <limits>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/task/charging/dock/geometry.hpp"

namespace autonomy {
namespace task {
namespace charging {

void DockController::Configure(
    const std::shared_ptr<autolink::Node>& node, const std::string& fixed_frame,
    const std::string& base_frame,
    const ::autonomy::task::proto::DockControllerOptions& options) {
    node_ = node;
    fixed_frame_ = fixed_frame.empty() ? "odom" : fixed_frame;
    base_frame_ = base_frame.empty() ? "base_link" : base_frame;
    const double k_phi = options.k_phi() > 0.0 ? options.k_phi() : 3.0;
    const double k_delta = options.k_delta() > 0.0 ? options.k_delta() : 2.0;
    const double beta = options.beta() > 0.0 ? options.beta() : 0.4;
    const double lambda = options.lambda() > 0.0 ? options.lambda() : 2.0;
    const double slowdown =
        options.slowdown_radius() > 0.0 ? options.slowdown_radius() : 0.25;
    const double decel =
        options.deceleration_max() > 0.0 ? options.deceleration_max() : 0.5;
    const double vmin =
        options.v_linear_min() > 0.0 ? options.v_linear_min() : 0.1;
    const double vmax =
        options.v_linear_max() > 0.0 ? options.v_linear_max() : 0.25;
    const double wmax =
        options.v_angular_max() > 0.0 ? options.v_angular_max() : 0.75;
    law_ = std::make_unique<SmoothControlLaw>(k_phi, k_delta, beta, lambda,
                                              slowdown, decel, vmin, vmax, wmax);
    use_collision_detection_ = options.has_use_collision_detection()
                                   ? options.use_collision_detection()
                                   : true;
    projection_time_ =
        options.projection_time() > 0.0 ? options.projection_time() : 5.0;
    simulation_time_step_ = options.simulation_time_step() > 0.0
                                ? options.simulation_time_step()
                                : 0.1;
    dock_collision_threshold_ = options.dock_collision_threshold() > 0.0
                                    ? options.dock_collision_threshold()
                                    : 0.3;
    robot_radius_ =
        options.robot_radius() > 0.0 ? options.robot_radius() : 0.22;
    footprint_ = ::autonomy::map::costmap_2d::makeFootprintFromRadius(robot_radius_);

    if (node_ && use_collision_detection_) {
        const std::string cmap = options.costmap_topic().empty()
                                     ? "/local_costmap"
                                     : options.costmap_topic();
        costmap_reader_ = node_->CreateReader<
            ::automsgs::msgs::map_msgs::OccupancyGrid>(
            cmap, [this](const auto& msg) { OnCostmap(msg); });
        const std::string fprint = options.footprint_topic().empty()
                                       ? "/local_costmap/published_footprint"
                                       : options.footprint_topic();
        footprint_reader_ = node_->CreateReader<
            ::automsgs::msgs::geometry_msgs::PolygonStamped>(
            fprint, [this](const auto& msg) { OnFootprint(msg); });
    }
    if (node_) {
        const std::string traj = options.trajectory_topic().empty()
                                     ? "/docking_trajectory"
                                     : options.trajectory_topic();
        trajectory_writer_ =
            node_->CreateWriter<::automsgs::msgs::nav_msgs::Path>(traj);
    }
}

void DockController::OnCostmap(
    const std::shared_ptr<::automsgs::msgs::map_msgs::OccupancyGrid>& msg) {
    if (!msg || msg->info().width() == 0 || msg->info().height() == 0) {
        return;
    }
    auto next =
        std::make_shared<::autonomy::map::costmap_2d::Costmap2D>(*msg);
    std::lock_guard<std::mutex> lock(mutex_);
    costmap_ = std::move(next);
    checker_.setCostmap(costmap_.get());
}

void DockController::OnFootprint(
    const std::shared_ptr<::automsgs::msgs::geometry_msgs::PolygonStamped>&
        msg) {
    if (!msg || msg->polygon().points_size() < 3) {
        return;
    }
    ::autonomy::map::costmap_2d::Footprint fp;
    fp.reserve(static_cast<size_t>(msg->polygon().points_size()));
    for (const auto& p : msg->polygon().points()) {
        automsgs::msgs::geometry_msgs::Point pt;
        pt.set_x(p.x());
        pt.set_y(p.y());
        pt.set_z(p.z());
        fp.push_back(pt);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    footprint_ = std::move(fp);
}

bool DockController::ComputeVelocity(
    const ::automsgs::msgs::geometry_msgs::Pose& pose,
    ::automsgs::msgs::geometry_msgs::Twist* cmd, bool is_docking,
    bool backward) {
    if (cmd == nullptr) {
        return false;
    }
    if (!law_) {
        Configure(nullptr, fixed_frame_, base_frame_, {});
    }
    *cmd = law_->Calculate(pose, backward);
    if (!std::isfinite(cmd->linear().x()) ||
        !std::isfinite(cmd->angular().z())) {
        return false;
    }
    return IsTrajectoryCollisionFree(pose, is_docking, backward);
}

bool DockController::IsTrajectoryCollisionFree(
    const ::automsgs::msgs::geometry_msgs::Pose& target_pose, bool is_docking,
    bool backward) {
    ::automsgs::msgs::nav_msgs::Path trajectory;
    trajectory.mutable_header()->set_frame_id(base_frame_);
    ::automsgs::msgs::geometry_msgs::PoseStamped next;
    next.mutable_header()->set_frame_id(base_frame_);
    next.mutable_pose()->mutable_orientation()->set_w(1.0);
    *trajectory.add_poses() = next;

    const unsigned int max_iter = static_cast<unsigned int>(
        std::ceil(projection_time_ / std::max(simulation_time_step_, 1e-3)));
    double distance = std::numeric_limits<double>::max();
    bool collision_ok = true;

    std::lock_guard<std::mutex> lock(mutex_);
    if (use_collision_detection_ && !costmap_) {
        AWARN_EVERY(50) << "dock collision: no local costmap yet";
        return false;
    }

    do {
        *next.mutable_pose() =
            law_->NextPose(simulation_time_step_, target_pose, next.pose(),
                           backward);
        *trajectory.add_poses() = next;

        ::automsgs::msgs::geometry_msgs::PoseStamped in_fixed;
        if (!geom::TransformPose(next, fixed_frame_, &in_fixed)) {
            AERROR << "dock collision: failed to transform projected pose";
            collision_ok = false;
            break;
        }
        const double dock_collision_distance =
            is_docking ? geom::Euclidean2(target_pose, next.pose())
                       : std::hypot(next.pose().position().x(),
                                    next.pose().position().y());
        if (use_collision_detection_ &&
            dock_collision_distance > dock_collision_threshold_ && costmap_) {
            const double cost = checker_.footprintCostAtPose(
                in_fixed.pose().position().x(), in_fixed.pose().position().y(),
                geom::Yaw(in_fixed.pose()), footprint_);
            if (cost >= ::autonomy::map::costmap_2d::LETHAL_OBSTACLE) {
                AWARN << "dock collision at ("
                      << in_fixed.pose().position().x() << ", "
                      << in_fixed.pose().position().y() << ")";
                collision_ok = false;
                break;
            }
        }
        distance = geom::Euclidean2(target_pose, next.pose());
    } while (distance > 1e-2 &&
             static_cast<unsigned int>(trajectory.poses_size()) < max_iter);

    if (trajectory_writer_) {
        trajectory_writer_->Write(trajectory);
    }
    return collision_ok;
}

}  // namespace charging
}  // namespace task
}  // namespace autonomy
