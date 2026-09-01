/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 */

#include "autonomy/task/teleop/cloud_feeder.hpp"

#include <cmath>
#include <cstring>
#include <vector>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/sensor_msgs/point_field.pb.h>

#include "autonomy/common/logging.hpp"

namespace autonomy::task::teleop {
namespace {

using automsgs::msgs::sensor_msgs::CameraInfo;
using automsgs::msgs::sensor_msgs::Image;
using automsgs::msgs::sensor_msgs::PointCloud2;
using automsgs::msgs::sensor_msgs::PointField;

float ReadDepthMeters(const Image& depth, int u, int v) {
    const int width = static_cast<int>(depth.width());
    const int height = static_cast<int>(depth.height());
    if (u < 0 || v < 0 || u >= width || v >= height) {
        return 0.f;
    }
    const size_t index =
        static_cast<size_t>(v) * static_cast<size_t>(width) + static_cast<size_t>(u);
    const std::string& encoding = depth.encoding();
    const std::string& data = depth.data();

    if (encoding == "32FC1") {
        if (data.size() < (index + 1) * sizeof(float)) {
            return 0.f;
        }
        float value = 0.f;
        std::memcpy(&value, data.data() + index * sizeof(float), sizeof(float));
        return value;
    }
    if (encoding == "16UC1") {
        if (data.size() < (index + 1) * sizeof(uint16_t)) {
            return 0.f;
        }
        uint16_t raw = 0;
        std::memcpy(&raw, data.data() + index * sizeof(uint16_t),
                    sizeof(uint16_t));
        return static_cast<float>(raw) * 0.001f;
    }
    return 0.f;
}

PointCloud2 BuildOpticalFrameCloud(const Image& depth, const CameraInfo& info,
                                   int decimation, double min_depth,
                                   double max_depth) {
    PointCloud2 cloud;
    if (info.k_size() < 9 || depth.width() == 0 || depth.height() == 0) {
        return cloud;
    }

    const float fx = static_cast<float>(info.k(0));
    const float fy = static_cast<float>(info.k(4));
    const float cx = static_cast<float>(info.k(2));
    const float cy = static_cast<float>(info.k(5));
    if (fx <= 0.f || fy <= 0.f) {
        return cloud;
    }

    const int step = std::max(1, decimation);
    const int width = static_cast<int>(depth.width());
    const int height = static_cast<int>(depth.height());

    std::vector<float> xyz;
    xyz.reserve(static_cast<std::size_t>((width / step) * (height / step) * 3));
    for (int v = 0; v < height; v += step) {
        for (int u = 0; u < width; u += step) {
            const float d = ReadDepthMeters(depth, u, v);
            if (!std::isfinite(d) || d < static_cast<float>(min_depth) ||
                d > static_cast<float>(max_depth)) {
                continue;
            }
            const float x = (static_cast<float>(u) - cx) * d / fx;
            const float y = (static_cast<float>(v) - cy) * d / fy;
            // Optical (x right, y down, z forward) → REP-103 camera_link.
            xyz.push_back(d);
            xyz.push_back(-x);
            xyz.push_back(-y);
        }
    }

    const uint32_t count = static_cast<uint32_t>(xyz.size() / 3);
    *cloud.mutable_header() = depth.header();
    cloud.set_height(1);
    cloud.set_width(count);
    cloud.set_is_bigendian(false);
    cloud.set_point_step(12);
    cloud.set_row_step(12 * count);
    cloud.set_is_dense(false);

    auto* field_x = cloud.add_fields();
    field_x->set_name("x");
    field_x->set_offset(0);
    field_x->set_datatype(PointField::FLOAT32);
    field_x->set_count(1);
    auto* field_y = cloud.add_fields();
    field_y->set_name("y");
    field_y->set_offset(4);
    field_y->set_datatype(PointField::FLOAT32);
    field_y->set_count(1);
    auto* field_z = cloud.add_fields();
    field_z->set_name("z");
    field_z->set_offset(8);
    field_z->set_datatype(PointField::FLOAT32);
    field_z->set_count(1);

    cloud.mutable_data()->resize(static_cast<std::size_t>(count) * 12);
    std::memcpy(cloud.mutable_data()->data(), xyz.data(), cloud.data().size());
    return cloud;
}

}  // namespace

/**
 * @brief Subscribe to RGB-D topics and bind costmap
 */
void PointCloudObstacleFeeder::Configure(
    std::shared_ptr<autolink::Node> node,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
    const Options& options) {
    node_ = std::move(node);
    costmap_ = std::move(costmap);
    options_ = options;
    started_ = false;
    cloud_reader_.reset();
    depth_reader_.reset();
    camera_info_reader_.reset();
    camera_info_.reset();
}

/**
 * @brief Create readers and begin feeding costmap
 */
void PointCloudObstacleFeeder::Start() {
    if (started_) {
        return;
    }
    if (!node_ || !costmap_) {
        AERROR << "PointCloudObstacleFeeder: Configure() before Start()";
        return;
    }

    PointCloudObstacleFeeder* self = this;
    if (!options_.cloud_topic.empty()) {
        cloud_reader_ = node_->CreateReader<PointCloud2>(
            options_.cloud_topic,
            [self](const std::shared_ptr<PointCloud2>& msg) {
                self->OnPointCloud(msg);
            });
        if (!cloud_reader_) {
            AERROR << "PointCloudObstacleFeeder: failed to subscribe "
                   << options_.cloud_topic;
        } else {
            AINFO << "PointCloudObstacleFeeder: listening on "
                  << options_.cloud_topic;
        }
    }

    if (!options_.camera_info_topic.empty()) {
        camera_info_reader_ = node_->CreateReader<CameraInfo>(
            options_.camera_info_topic,
            [self](const std::shared_ptr<CameraInfo>& msg) {
                self->OnCameraInfo(msg);
            });
        if (!camera_info_reader_) {
            AWARN << "PointCloudObstacleFeeder: failed to subscribe "
                  << options_.camera_info_topic;
        }
    }

    if (!options_.depth_topic.empty()) {
        depth_reader_ = node_->CreateReader<Image>(
            options_.depth_topic, [self](const std::shared_ptr<Image>& msg) {
                self->OnDepthImage(msg);
            });
        if (!depth_reader_) {
            AERROR << "PointCloudObstacleFeeder: failed to subscribe "
                   << options_.depth_topic;
        } else {
            AINFO << "PointCloudObstacleFeeder: depth projection from "
                  << options_.depth_topic;
        }
    }

    if (!cloud_reader_ && !depth_reader_) {
        AWARN << "PointCloudObstacleFeeder: no cloud or depth source configured";
        return;
    }
    started_ = true;
}

/**
 * @brief Shutdown readers
 */
void PointCloudObstacleFeeder::Stop() {
    if (cloud_reader_) {
        cloud_reader_->Shutdown();
        cloud_reader_.reset();
    }
    if (depth_reader_) {
        depth_reader_->Shutdown();
        depth_reader_.reset();
    }
    if (camera_info_reader_) {
        camera_info_reader_->Shutdown();
        camera_info_reader_.reset();
    }
    camera_info_.reset();
    started_ = false;
}

/**
 * @brief Check cloud freshness against stale timeout
 */
bool PointCloudObstacleFeeder::IsCloudFresh() const {
    if (last_cloud_time_.time_since_epoch().count() == 0) {
        return false;
    }
    const auto elapsed =
        std::chrono::steady_clock::now() - last_cloud_time_;
    return std::chrono::duration<double>(elapsed).count() <=
           options_.stale_timeout_sec;
}

/**
 * @brief True after first cloud or projected depth feed
 */
bool PointCloudObstacleFeeder::HasReceivedCloud() const {
    return last_cloud_time_.time_since_epoch().count() != 0;
}

/**
 * @brief Feed PointCloud2 marks into rolling costmap
 */
void PointCloudObstacleFeeder::OnPointCloud(
    const std::shared_ptr<PointCloud2>& msg) {
    if (!msg || !costmap_) {
        return;
    }
    if (msg->width() == 0 && msg->height() == 0) {
        return;
    }
    costmap_->feedPointCloud2(*msg);
    last_cloud_time_ = std::chrono::steady_clock::now();
}

/**
 * @brief Cache camera intrinsics for depth projection
 */
void PointCloudObstacleFeeder::OnCameraInfo(
    const std::shared_ptr<CameraInfo>& msg) {
    if (!msg) {
        return;
    }
    camera_info_ = *msg;
}

/**
 * @brief Project depth image and mark obstacles
 */
void PointCloudObstacleFeeder::OnDepthImage(
    const std::shared_ptr<Image>& msg) {
    if (!msg || !costmap_ || !camera_info_.has_value()) {
        return;
    }
    if (IsCloudFresh() && cloud_reader_) {
        return;
    }
    FeedDepth(*msg, *camera_info_);
}

/**
 * @brief Back-project depth pixels to 3D and update costmap
 */
void PointCloudObstacleFeeder::FeedDepth(
    const Image& depth, const CameraInfo& info) {
    const PointCloud2 cloud = BuildOpticalFrameCloud(
        depth, info, options_.depth_decimation, options_.min_depth_m,
        options_.max_depth_m);
    if (cloud.width() == 0) {
        return;
    }
    costmap_->feedPointCloud2(cloud);
    last_cloud_time_ = std::chrono::steady_clock::now();
}

}  // namespace autonomy::task::teleop
