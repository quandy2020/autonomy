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

#include "autonomy/perception/behaviors/follow/grid.hpp"

#include <array>
#include <cstring>
#include <limits>

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
      map_({"elevation", "obstacle", "traversability", "semantic_obstacle"}) {
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
    std::string* error,
    const automsgs::msgs::sensor_msgs::Image* semantic) {
    if (depth.encoding() != "32FC1" || depth.is_bigendian() ||
        camera.k_size() < 9) {
        SetError(error, "depth must be 32FC1 with valid CameraInfo.");
        return false;
    }

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
    if (depth.width() == 0 || depth.height() == 0 ||
        depth.width() > std::numeric_limits<int>::max() ||
        depth.height() > std::numeric_limits<int>::max() ||
        depth.step() < sizeof(float) * static_cast<uint64_t>(depth.width()) ||
        depth.data().size() < static_cast<uint64_t>(depth.step()) * depth.height() ||
        !std::isfinite(fx) || !std::isfinite(fy) || fx <= 0 || fy <= 0 ||
        !std::isfinite(ppx) || !std::isfinite(ppy)) {
        SetError(error, "invalid depth layout or camera intrinsics.");
        return false;
    }
    const auto& stamp = depth.header().stamp();
    const bool aligned = semantic && semantic->encoding() == "mono8" &&
        camera.width() == depth.width() && camera.height() == depth.height() &&
        camera.header().frame_id() == options_.camera_frame() &&
        depth.header().frame_id() == options_.camera_frame() &&
        depth.header().frame_id() == semantic->header().frame_id() &&
        (stamp.sec() != 0 || stamp.nanosec() != 0) &&
        stamp.sec() == semantic->header().stamp().sec() &&
        stamp.nanosec() == semantic->header().stamp().nanosec() &&
        semantic->width() == depth.width() && semantic->height() == depth.height() &&
        semantic->step() >= semantic->width() &&
        semantic->data().size() >= static_cast<uint64_t>(semantic->step()) *
            semantic->height();
    std::array<bool, 256> blocked_labels{};
    for (const auto label : options_.semantic_obstacle_labels())
        if (label < 255)
            blocked_labels[label] = true;
    const int width = static_cast<int>(depth.width());
    const int height = static_cast<int>(depth.height());

    const ::grid_map::Position center(robot_pose.position().x(),
                                      robot_pose.position().y());
    map_.move(center);
    map_.clearAll();

    const double robot_z = robot_pose.position().z();
    const int stride = std::max(1, width / 160);
    for (int v = 0; v < height; v += stride) {
        for (int u = 0; u < width; u += stride) {
            float z;
            std::memcpy(&z, depth.data().data() +
                            static_cast<size_t>(v) * depth.step() +
                            static_cast<size_t>(u) * sizeof(z), sizeof(z));
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
            const bool semantic_blocked = aligned &&
                blocked_labels[static_cast<uint8_t>(semantic->data()[
                    static_cast<size_t>(v) * semantic->step() + u])];
            if (semantic_blocked)
                map_.atPosition("semantic_obstacle", pos) = 1.0F;
            const bool occupied = semantic_blocked ||
                height_rel > options_.obstacle_height_m();
            float& obstacle = map_.atPosition("obstacle", pos);
            obstacle = occupied || obstacle > 0.5F ? 1.0F : 0.0F;
            map_.atPosition("traversability", pos) = obstacle;
        }
    }

    // Inflate obstacles by robot radius.
    const int inflate = static_cast<int>(std::ceil(
        options_.robot_radius_m() / options_.grid_resolution_m()));
    if (inflate > 0) {
        const ::grid_map::Size size = map_.getSize();
        ::grid_map::Matrix inflated = map_["obstacle"];
        ::grid_map::Matrix semantic_inflated = map_["semantic_obstacle"];
        for (int r = 0; r < size(0); ++r) {
            for (int c = 0; c < size(1); ++c) {
                if (!(map_.at("obstacle", ::grid_map::Index(r, c)) > 0.5F)) {
                    continue;
                }
                const bool semantic_blocked =
                    map_.at("semantic_obstacle", ::grid_map::Index(r, c)) > 0.5F;
                for (int dy = -inflate; dy <= inflate; ++dy) {
                    for (int dx = -inflate; dx <= inflate; ++dx) {
                        const int nr = r + dy;
                        const int nc = c + dx;
                        if (nr < 0 || nc < 0 || nr >= size(0) ||
                            nc >= size(1)) {
                            continue;
                        }
                        inflated(nr, nc) = 1.0F;
                        if (semantic_blocked)
                            semantic_inflated(nr, nc) = 1.0F;
                    }
                }
            }
        }
        map_["semantic_obstacle"] = semantic_inflated;
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
        if (dx * dx + dy * dy > radius_sq ||
            map_.at("semantic_obstacle", *it) > 0.5F ||
            !std::isfinite(map_.at("obstacle", *it))) {
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
