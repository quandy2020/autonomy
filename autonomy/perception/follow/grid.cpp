/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/perception/follow/grid.hpp"

#include "autonomy/map/grid_map/grid_map_core/iterators/grid_map_iterator.hpp"
#include "autonomy/map/grid_map/grid_map_msgs/grid_map_converter.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace autonomy {
namespace perception {
namespace follow {
namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Follow: " + message;
    }
}

}  // namespace

LocalGrid::LocalGrid(proto::FollowOptions options)
    : options_(std::move(options)),
      map_({"elevation", "obstacle", "traversability"}) {
    const double length = options_.grid_length_m();
    map_.setGeometry(::grid_map::Length(length, length),
                     options_.grid_resolution_m(),
                     ::grid_map::Position(0.0, 0.0));
    map_.setFrameId(options_.map_frame());
}

bool LocalGrid::Update(
    const automsgs::msgs::sensor_msgs::Image& depth,
    const automsgs::msgs::sensor_msgs::CameraInfo& camera,
    const automsgs::msgs::geometry_msgs::TransformStamped& camera_to_map,
    const automsgs::msgs::geometry_msgs::Pose& robot_pose,
    std::string* error) {
    if (depth.encoding() != "32FC1" || camera.k_size() < 9) {
        SetError(error, "depth must be 32FC1 with valid CameraInfo.");
        return false;
    }

    const ::grid_map::Position center(robot_pose.position().x(),
                                      robot_pose.position().y());
    map_.move(center);
    map_.clearAll();

    const double fx = camera.k(0);
    const double fy = camera.k(4);
    const double ppx = camera.k(2);
    const double ppy = camera.k(5);
    const auto& t = camera_to_map.transform().translation();
    const auto& q = camera_to_map.transform().rotation();
    const double qw = q.w();
    const double qx = q.x();
    const double qy = q.y();
    const double qz = q.z();
    const float* depth_data =
        reinterpret_cast<const float*>(depth.data().data());
    const int width = static_cast<int>(depth.width());
    const int height = static_cast<int>(depth.height());
    const size_t need =
        static_cast<size_t>(width) * static_cast<size_t>(height) * sizeof(float);
    if (depth.data().size() < need) {
        SetError(error, "depth buffer is too small.");
        return false;
    }

    const double robot_z = robot_pose.position().z();
    const int stride = std::max(1, width / 160);
    for (int v = 0; v < height; v += stride) {
        for (int u = 0; u < width; u += stride) {
            const float z = depth_data[static_cast<size_t>(v) * width + u];
            if (!std::isfinite(z) || z < options_.min_depth_m() ||
                z > options_.max_depth_m()) {
                continue;
            }
            const double X = (static_cast<double>(u) - ppx) * z / fx;
            const double Y = (static_cast<double>(v) - ppy) * z / fy;
            const double Z = z;
            const double rx =
                (1 - 2 * (qy * qy + qz * qz)) * X +
                (2 * (qx * qy - qz * qw)) * Y +
                (2 * (qx * qz + qy * qw)) * Z + t.x();
            const double ry =
                (2 * (qx * qy + qz * qw)) * X +
                (1 - 2 * (qx * qx + qz * qz)) * Y +
                (2 * (qy * qz - qx * qw)) * Z + t.y();
            const double rz =
                (2 * (qx * qz - qy * qw)) * X +
                (2 * (qy * qz + qx * qw)) * Y +
                (1 - 2 * (qx * qx + qy * qy)) * Z + t.z();

            const ::grid_map::Position pos(rx, ry);
            if (!map_.isInside(pos)) {
                continue;
            }
            const float height_rel = static_cast<float>(rz - robot_z);
            float& elev = map_.atPosition("elevation", pos);
            if (!std::isfinite(elev) || height_rel < elev) {
                elev = height_rel;
            }
            const bool occupied =
                height_rel > options_.obstacle_height_m();
            float& obstacle = map_.atPosition("obstacle", pos);
            obstacle = occupied ? 1.0F : 0.0F;
            float& trav = map_.atPosition("traversability", pos);
            trav = occupied ? 1.0F : 0.0F;
        }
    }

    // Inflate obstacles by robot radius.
    const int inflate = static_cast<int>(std::ceil(
        options_.robot_radius_m() / options_.grid_resolution_m()));
    if (inflate > 0) {
        const ::grid_map::Size size = map_.getSize();
        ::grid_map::Matrix inflated = map_["obstacle"];
        for (int r = 0; r < size(0); ++r) {
            for (int c = 0; c < size(1); ++c) {
                if (!(map_.at("obstacle", ::grid_map::Index(r, c)) > 0.5F)) {
                    continue;
                }
                for (int dy = -inflate; dy <= inflate; ++dy) {
                    for (int dx = -inflate; dx <= inflate; ++dx) {
                        const int nr = r + dy;
                        const int nc = c + dx;
                        if (nr < 0 || nc < 0 || nr >= size(0) ||
                            nc >= size(1)) {
                            continue;
                        }
                        inflated(nr, nc) = 1.0F;
                    }
                }
            }
        }
        map_["obstacle"] = inflated;
        map_["traversability"] = inflated;
    }
    return true;
}

bool LocalGrid::ToMessage(automsgs::msgs::map_msgs::GridMap* message) const {
    if (message == nullptr) {
        return false;
    }
    ::grid_map::GridMapConverter::toMessage(map_, *message);
    return true;
}

void LocalGrid::ClearDisk(double x, double y, double radius_m) {
    if (!(radius_m > 0.0)) {
        return;
    }
    const double radius_sq = radius_m * radius_m;
    for (::grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
        ::grid_map::Position pos;
        map_.getPosition(*it, pos);
        const double dx = pos.x() - x;
        const double dy = pos.y() - y;
        if (dx * dx + dy * dy > radius_sq) {
            continue;
        }
        map_.at("obstacle", *it) = 0.0F;
        map_.at("traversability", *it) = 0.0F;
    }
}

bool LocalGrid::IsTraversable(double x, double y) const {
    const ::grid_map::Position pos(x, y);
    if (!map_.isInside(pos)) {
        return false;
    }
    const float trav = map_.atPosition("traversability", pos);
    if (!std::isfinite(trav)) {
        return false;
    }
    return trav < 0.5F;
}

}  // namespace follow
}  // namespace perception
}  // namespace autonomy
