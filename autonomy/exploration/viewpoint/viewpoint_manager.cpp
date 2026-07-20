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

#include "autonomy/exploration/viewpoint/viewpoint_manager.hpp"

#include <algorithm>
#include <cmath>

#include "Eigen/Geometry"

#include "autonomy/common/math/vec2d.hpp"
#include "autonomy/common/transform/rigid_transform.hpp"
#include "autonomy/common/transform/transform.hpp"
#include "autonomy/exploration/planner/los_checker.hpp"

namespace autonomy {
namespace exploration {
namespace {

commsgs::geometry_msgs::Quaternion OrientationFromYaw(double yaw)
{
    const Eigen::Quaterniond q =
        ::autonomy::common::transform::RollPitchYaw(0.0, 0.0, yaw);
    commsgs::geometry_msgs::Quaternion out;
    out.w = q.w();
    out.x = q.x();
    out.y = q.y();
    out.z = q.z();
    return out;
}

}  // namespace

commsgs::geometry_msgs::PoseStamped Viewpoint::ToPoseStamped(
    const std::string& frame_id) const
{
    commsgs::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation = OrientationFromYaw(yaw);
    return pose;
}

ViewpointManager::ViewpointManager(const proto::ExplorationOptions& options)
    : options_(options), camera_(options)
{
}

void ViewpointManager::SetOptions(const proto::ExplorationOptions& options)
{
    options_ = options;
    camera_.SetOptions(options);
}

void ViewpointManager::UpdateRobotPosition(double x, double y, double z,
                                           double yaw)
{
    robot_x_ = x;
    robot_y_ = y;
    robot_z_ = z;
    robot_yaw_ = yaw;
    grid_origin_x_ = x;
    grid_origin_y_ = y;
    RebuildGrid();
}

void ViewpointManager::RebuildGrid()
{
    viewpoints_.clear();
    next_id_ = 0;
    const int nx = options_.viewpoint().number_x();
    const int ny = options_.viewpoint().number_y();
    const double res = options_.viewpoint().resolution();
    const double yaw_step = 2.0 * M_PI / std::max(1, options_.viewpoint().yaw_samples());

    for (int iy = 0; iy < ny; ++iy) {
        for (int ix = 0; ix < nx; ++ix) {
            Viewpoint vp;
            vp.x = grid_origin_x_ +
                   (static_cast<double>(ix) - nx * 0.5 + 0.5) * res;
            vp.y = grid_origin_y_ +
                   (static_cast<double>(iy) - ny * 0.5 + 0.5) * res;
            vp.z = robot_z_;
            for (int k = 0; k < options_.viewpoint().yaw_samples(); ++k) {
                Viewpoint yaw_vp = vp;
                yaw_vp.yaw = -M_PI + k * yaw_step;
                yaw_vp.id = next_id_++;
                viewpoints_.push_back(yaw_vp);
            }
        }
    }
}

int ViewpointManager::CellIndexForPosition(double x, double y, int grid_cols,
                                           double cell_size, double origin_x,
                                           double origin_y) const
{
    const int cx =
        static_cast<int>(std::floor((x - origin_x) / cell_size));
    const int cy =
        static_cast<int>(std::floor((y - origin_y) / cell_size));
    if (cx < 0 || cy < 0) {
        return -1;
    }
    return cy * grid_cols + cx;
}

bool ViewpointManager::IsCollision(double x, double y) const
{
    if (!env_) {
        return false;
    }
    return env_->IsOccupied(x, y);
}

void ViewpointManager::UpdateFromEnv(
    const PlanningEnv& env, const std::vector<CellStatus>& cell_status,
    int grid_cols, double cell_size, double origin_x, double origin_y)
{
    env_ = &env;
    UpdateLineOfSightFlags(env);
    ScoreViewpoints(env);
    for (auto& vp : viewpoints_) {
        if (!vp.in_los || !vp.connected) {
            vp.gain = 0.0;
            continue;
        }
        if (!env.IsInExplorationArea(vp.x, vp.y)) {
            vp.gain = 0.0;
            continue;
        }
        if (IsCollision(vp.x, vp.y)) {
            vp.gain = 0.0;
            continue;
        }
        const double dist =
            ::autonomy::common::math::Vec2d(vp.x, vp.y)
                .DistanceTo(
                    ::autonomy::common::math::Vec2d(robot_x_, robot_y_));
        if (dist > options_.viewpoint().resolution() *
                      std::max(options_.viewpoint().number_x(),
                               options_.viewpoint().number_y()) *
                      0.6) {
            vp.gain = 0.0;
            continue;
        }
        const int cell_idx =
            CellIndexForPosition(vp.x, vp.y, grid_cols, cell_size, origin_x,
                                 origin_y);
        vp.cell_index = cell_idx;
        if (cell_idx >= 0 &&
            static_cast<size_t>(cell_idx) < cell_status.size()) {
            const auto st = cell_status[static_cast<size_t>(cell_idx)];
            if (st == CellStatus::kCovered) {
                vp.gain *= 0.25;
            }
        }
    }
}

void ViewpointManager::UpdateLineOfSightFlags(const PlanningEnv& env)
{
    const bool stop_at_unknown = options_.los_stop_at_unknown();
    const double neighbor_dist = options_.viewpoint().resolution() * 1.5;
    for (auto& vp : viewpoints_) {
        vp.in_los = HasLineOfSight(env, robot_x_, robot_y_, vp.x, vp.y,
                                   stop_at_unknown);
        vp.connected = false;
        if (!vp.in_los) {
            continue;
        }
        // Connected if any nearby lattice neighbor also has robot LOS.
        for (const auto& other : viewpoints_) {
            if (other.id == vp.id) {
                continue;
            }
            const double d =
                ::autonomy::common::math::Vec2d(vp.x, vp.y)
                    .DistanceTo(
                        ::autonomy::common::math::Vec2d(other.x, other.y));
            if (d > neighbor_dist || d < 1e-6) {
                continue;
            }
            if (HasLineOfSight(env, vp.x, vp.y, other.x, other.y,
                               stop_at_unknown)) {
                vp.connected = true;
                break;
            }
        }
        // Isolated but robot-visible points still count as connected.
        if (!vp.connected) {
            vp.connected = true;
        }
    }
}

void ViewpointManager::ScoreViewpoints(const PlanningEnv& env)
{
    const auto& targets = env.targets();
    const auto& uncovered = env.uncovered();
    for (auto& vp : viewpoints_) {
        if (!vp.in_los) {
            vp.gain = 0.0;
            continue;
        }
        const Eigen::Quaterniond q =
            ::autonomy::common::transform::RollPitchYaw(0.0, 0.0, vp.yaw);
        commsgs::geometry_msgs::Transform map_t_camera;
        map_t_camera.translation.x = vp.x;
        map_t_camera.translation.y = vp.y;
        map_t_camera.translation.z =
            vp.z + options_.viewpoint().robot_height();
        map_t_camera.rotation.w = q.w();
        map_t_camera.rotation.x = q.x();
        map_t_camera.rotation.y = q.y();
        map_t_camera.rotation.z = q.z();
        vp.gain = camera_.ComputeGain(map_t_camera, targets, uncovered, &env);
    }
}

std::vector<Viewpoint> ViewpointManager::GetCandidates(double min_gain) const
{
    std::vector<Viewpoint> out;
    for (const auto& vp : viewpoints_) {
        if (vp.visited || !vp.in_los || !vp.connected || vp.gain < min_gain) {
            continue;
        }
        out.push_back(vp);
    }
    std::sort(out.begin(), out.end(),
              [](const Viewpoint& a, const Viewpoint& b) {
                  return a.gain > b.gain;
              });
    return out;
}

Viewpoint ViewpointManager::GetRobotViewpoint() const
{
    Viewpoint vp;
    vp.x = robot_x_;
    vp.y = robot_y_;
    vp.z = robot_z_;
    vp.yaw = robot_yaw_;
    vp.id = -1;
    return vp;
}

void ViewpointManager::MarkVisited(int id)
{
    for (auto& vp : viewpoints_) {
        if (vp.id == id) {
            vp.visited = true;
        }
    }
}

void ViewpointManager::MarkVisitedNear(double x, double y, double radius)
{
    for (auto& vp : viewpoints_) {
        const double dist =
            ::autonomy::common::math::Vec2d(vp.x, vp.y)
                .DistanceTo(::autonomy::common::math::Vec2d(x, y));
        if (dist < radius) {
            vp.visited = true;
        }
    }
}

}  // namespace exploration
}  // namespace autonomy
