/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file local_esdf.cpp
 * @brief Build a robot-frame ESDF from a depth image using map/esdf.
 */

#include "autonomy/control/controller/policy_controller/sand_planner/local_esdf.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/core.hpp>

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace {

bool DecodeDepthMetres(const automsgs::msgs::sensor_msgs::Image& image,
                       cv::Mat* metres, std::string* error) {
    if (metres == nullptr || image.width() == 0 || image.height() == 0 ||
        image.data().empty()) {
        if (error != nullptr) {
            *error = "Empty depth image";
        }
        return false;
    }
    const int width = static_cast<int>(image.width());
    const int height = static_cast<int>(image.height());
    const std::string& encoding = image.encoding();
    auto* raw = reinterpret_cast<unsigned char*>(
        const_cast<char*>(image.data().data()));
    if (encoding == "32FC1" || encoding == "32FC") {
        cv::Mat depth(height, width, CV_32FC1, raw,
                      image.step() > 0 ? image.step()
                                       : static_cast<size_t>(width) * 4);
        depth.convertTo(*metres, CV_32FC1);
        return true;
    }
    if (encoding == "16UC1" || encoding == "mono16" || encoding == "16UC") {
        cv::Mat depth(height, width, CV_16UC1, raw,
                      image.step() > 0 ? image.step()
                                       : static_cast<size_t>(width) * 2);
        depth.convertTo(*metres, CV_32FC1, 0.001);
        return true;
    }
    if (error != nullptr) {
        *error = "Unsupported depth encoding: " + encoding;
    }
    return false;
}

}  // namespace

map::esdf::Options MakeLocalEsdfOptions(const proto::SandPlanner& options) {
    map::esdf::Options map_options;
    map_options.origin = Eigen::Vector3d(
        options.esdf_origin_x(), options.esdf_origin_y(),
        options.esdf_origin_z());
    map_options.map_size = Eigen::Vector3d(
        options.esdf_size_x(), options.esdf_size_y(), options.esdf_size_z());
    map_options.resolution = options.esdf_resolution();
    map_options.min_ray_length = 0.0;
    map_options.max_ray_length =
        options.max_depth() > 0.0f ? options.max_depth() : 8.0;
    return map_options;
}

std::vector<Eigen::Vector3d> UnprojectDepthToRobot(
    const ObservationSnapshot& snapshot, const proto::SandPlanner& options,
    std::string* error) {
    std::vector<Eigen::Vector3d> points;
    if (snapshot.depth_history.empty() && snapshot.image == nullptr) {
        return points;
    }
    const auto& image = snapshot.depth_history.empty()
                            ? *snapshot.image
                            : *snapshot.depth_history.back();
    cv::Mat metres;
    if (!DecodeDepthMetres(image, &metres, error) || metres.empty()) {
        return points;
    }
    const int stride = std::max(1, options.pixel_stride());
    const float max_depth =
        options.max_depth() > 0.0f ? options.max_depth() : 8.0f;
    const float scale_x =
        static_cast<float>(metres.cols) / std::max(1.0f, options.source_width());
    const float scale_y =
        static_cast<float>(metres.rows) / std::max(1.0f, options.source_height());
    const float fx = options.focal_x() * scale_x;
    const float fy = options.focal_y() * scale_y;
    const float cx = options.principal_x() * scale_x;
    const float cy = options.principal_y() * scale_y;
    const bool optical = options.depth_optical_frame();
    for (int row = 0; row < metres.rows; row += stride) {
        for (int col = 0; col < metres.cols; col += stride) {
            const float z = metres.at<float>(row, col);
            if (!std::isfinite(z) || z <= 0.05f || z > max_depth) {
                continue;
            }
            const float x_cam = (static_cast<float>(col) - cx) / fx * z;
            const float y_cam = (static_cast<float>(row) - cy) / fy * z;
            if (optical) {
                points.emplace_back(z, -x_cam, -y_cam);
            } else {
                points.emplace_back(x_cam, y_cam, z);
            }
        }
    }
    return points;
}

std::unique_ptr<map::esdf::EsdfMap> BuildLocalEsdfFromDepth(
    const ObservationSnapshot& snapshot, const proto::SandPlanner& options,
    std::string* error) {
    auto map = std::make_unique<map::esdf::EsdfMap>(
        MakeLocalEsdfOptions(options));
    const std::vector<Eigen::Vector3d> cloud =
        UnprojectDepthToRobot(snapshot, options, error);
    if (cloud.empty()) {
        return map;
    }
    const Eigen::Vector3d sensor = Eigen::Vector3d::Zero();
    for (int round = 0; round < 2; ++round) {
        map->IntegrateCloud(sensor, cloud);
        map->UpdateOccupancy(true);
    }
    map->UpdateESDF();
    return map;
}

double QueryLocalClearance(const map::esdf::EsdfMap& map,
                           const Eigen::Vector3d& position) {
    const double clearance = map.GetDistance(position);
    // Outside the AABB: EsdfMap returns kUndefined. Empty / never-updated
    // cells return kInfinity metres. Neither is a measured obstacle distance.
    if (clearance < 0.0 ||
        clearance >= 0.5 * static_cast<double>(map::esdf::kInfinity)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return clearance;
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
