/*
 * Copyright 2026 The Openbot Authors (duyongquan)
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

#include "autonomy/task/apps/teleop/rgbd_obstacle_feeder.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "autonomy/common/logging.hpp"

namespace autonomy::task::teleop {
namespace {

using commsgs::sensor_msgs::Image;
using commsgs::sensor_msgs::PointCloud2;
using commsgs::sensor_msgs::PointField;

bool IsUint16Depth(const std::string& encoding) {
    return encoding == "16UC1" || encoding == "mono16";
}

bool IsFloatDepth(const std::string& encoding) {
    return encoding == "32FC1";
}

float ReadDepthMeters(const Image& image, size_t index, bool is_uint16) {
    if (is_uint16) {
        if (image.data.size() < index + 2) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        uint16_t raw = 0;
        std::memcpy(&raw, &image.data[index], sizeof(uint16_t));
        if (raw == 0) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        return static_cast<float>(raw) * 0.001f;
    }
    if (image.data.size() < index + 4) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    float depth = 0.0f;
    std::memcpy(&depth, &image.data[index], sizeof(float));
    return depth;
}

void InitXYZCloudHeader(PointCloud2& cloud, const Image& image,
                        const std::string& frame_id) {
    cloud.header = image.header;
    cloud.header.frame_id = frame_id;
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = false;
    cloud.fields.resize(3);
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].datatype = PointField::FLOAT32;
    cloud.fields[0].count = 1;
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].datatype = PointField::FLOAT32;
    cloud.fields[1].count = 1;
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].datatype = PointField::FLOAT32;
    cloud.fields[2].count = 1;
    cloud.point_step = 12;
}

}  // namespace

void RgbdObstacleFeeder::Configure(
    std::shared_ptr<autolink::Node> node,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
    const Options& options) {
    node_ = std::move(node);
    costmap_ = std::move(costmap);
    options_ = options;
    started_ = false;
    reader_.reset();
}

void RgbdObstacleFeeder::Start() {
    if (started_) {
        return;
    }
    if (!node_ || !costmap_) {
        AERROR << "RgbdObstacleFeeder: Configure() before Start()";
        return;
    }
    RgbdObstacleFeeder* self = this;
    reader_ = node_->CreateReader<Image>(
        options_.depth_topic,
        [self](const std::shared_ptr<Image>& msg) { self->OnDepthImage(msg); });
    if (!reader_) {
        AERROR << "RgbdObstacleFeeder: failed to subscribe "
               << options_.depth_topic;
        return;
    }
    started_ = true;
    AINFO << "RgbdObstacleFeeder: listening on " << options_.depth_topic;
}

bool RgbdObstacleFeeder::IsCloudFresh() const {
    if (last_cloud_time_.time_since_epoch().count() == 0) {
        return false;
    }
    const auto elapsed =
        std::chrono::steady_clock::now() - last_cloud_time_;
    return std::chrono::duration<double>(elapsed).count() <=
           options_.stale_timeout_sec;
}

void RgbdObstacleFeeder::OnDepthImage(const std::shared_ptr<Image>& msg) {
    if (!msg || !costmap_) {
        return;
    }
    PointCloud2 cloud = ProjectDepth(*msg);
    if (cloud.width == 0) {
        return;
    }
    costmap_->feedPointCloud2(cloud);
    last_cloud_time_ = std::chrono::steady_clock::now();
}

PointCloud2 RgbdObstacleFeeder::ProjectDepth(const Image& image) const {
    PointCloud2 cloud;
    if (image.width == 0 || image.height == 0 || image.step == 0) {
        return cloud;
    }

    const bool uint16_depth = IsUint16Depth(image.encoding);
    const bool float_depth = IsFloatDepth(image.encoding);
    if (!uint16_depth && !float_depth) {
        AWARN << "RgbdObstacleFeeder: unsupported depth encoding "
              << image.encoding;
        return cloud;
    }

    const int stride = std::max(1, options_.stride);
    const size_t bytes_per_pixel =
        uint16_depth ? sizeof(uint16_t) : sizeof(float);

    std::vector<float> points;
    points.reserve(static_cast<size_t>(image.width * image.height / stride));

    for (uint32_t v = 0; v < image.height; v += static_cast<uint32_t>(stride)) {
        for (uint32_t u = 0; u < image.width; u += static_cast<uint32_t>(stride)) {
            const size_t index =
                static_cast<size_t>(v) * image.step + static_cast<size_t>(u) *
                                                          bytes_per_pixel;
            const float depth =
                ReadDepthMeters(image, index, uint16_depth);
            if (!std::isfinite(depth) || depth < options_.min_depth ||
                depth > options_.max_depth) {
                continue;
            }

            // Camera optical frame: +x right, +y down, +z forward.
            const float x =
                static_cast<float>((static_cast<double>(u) - options_.cx) *
                                   depth / options_.fx);
            const float y =
                static_cast<float>((static_cast<double>(v) - options_.cy) *
                                   depth / options_.fy);
            const float z = depth;

            // Height band in camera frame (up ≈ -y) until base_link TF is wired.
            const float height_up = -y;
            if (height_up < options_.min_height ||
                height_up > options_.max_height) {
                continue;
            }

            points.push_back(x);
            points.push_back(y);
            points.push_back(z);
        }
    }

    InitXYZCloudHeader(cloud, image, options_.camera_frame);
    cloud.width = static_cast<uint32_t>(points.size() / 3);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step);
    if (!points.empty()) {
        std::memcpy(cloud.data.data(), points.data(),
                    points.size() * sizeof(float));
    }
    return cloud;
}

}  // namespace autonomy::task::teleop
