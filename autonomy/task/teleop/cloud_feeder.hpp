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

#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "autolink/node/node.hpp"
#include "autolink/node/reader.hpp"
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy::task::teleop {

/**
 * @class teleop::PointCloudObstacleFeeder
 * @brief Feeds RGB-D point clouds (or projected depth) into the teleop costmap
 */
class PointCloudObstacleFeeder
{
public:
    // RGB-D / depth subscription and projection options.
    struct Options {
        // Explicit PointCloud2 topic (optional if depth projection is set).
        std::string cloud_topic{"/camera/depth/points"};
        // Depth image topic for on-the-fly projection (autosim default).
        std::string depth_topic;
        // CameraInfo topic required for depth projection.
        std::string camera_info_topic;
        // Pixel decimation when projecting depth to 3D points.
        int depth_decimation{4};
        // Minimum valid depth (m).
        double min_depth_m{0.1};
        // Maximum valid depth (m).
        double max_depth_m{4.0};
        // Max age before cloud data is considered stale (seconds).
        double stale_timeout_sec{0.5};
    };

    /**
     * @brief Subscribe to cloud and/or depth topics
     * @param node Autolink node for readers
     * @param costmap Target rolling costmap wrapper
     * @param options Feeder configuration
     */
    void Configure(std::shared_ptr<autolink::Node> node,
                   std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
                   const Options& options);

    /**
     * @brief Activate sensor readers (no-op if already started)
     */
    void Start();

    /**
     * @brief Deactivate sensor readers
     */
    void Stop();

    /**
     * @brief True if last cloud arrived within stale_timeout_sec
     */
    bool IsCloudFresh() const;

    /**
     * @brief True if at least one cloud has been received
     */
    bool HasReceivedCloud() const;

    /**
     * @brief Current feeder configuration
     */
    const Options& options() const { return options_; }

private:
    /**
     * @brief Handle incoming PointCloud2 and mark obstacles on costmap
     */
    void OnPointCloud(
        const std::shared_ptr<automsgs::msgs::sensor_msgs::PointCloud2>& msg);

    /**
     * @brief Cache CameraInfo for depth projection
     */
    void OnCameraInfo(
        const std::shared_ptr<automsgs::msgs::sensor_msgs::CameraInfo>& msg);

    /**
     * @brief Project depth image to points and feed costmap
     */
    void OnDepthImage(
        const std::shared_ptr<automsgs::msgs::sensor_msgs::Image>& msg);

    /**
     * @brief Project depth + intrinsics to 3D and mark costmap cells
     */
    void FeedDepth(
        const automsgs::msgs::sensor_msgs::Image& depth,
        const automsgs::msgs::sensor_msgs::CameraInfo& info);

    // Loaded feeder configuration.
    Options options_;
    // Autolink node for readers.
    std::shared_ptr<autolink::Node> node_;
    // Target costmap for obstacle marking.
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_;
    // Optional PointCloud2 subscription.
    std::shared_ptr<autolink::Reader<automsgs::msgs::sensor_msgs::PointCloud2>>
        cloud_reader_;
    // Optional depth image subscription.
    std::shared_ptr<autolink::Reader<automsgs::msgs::sensor_msgs::Image>>
        depth_reader_;
    // Optional CameraInfo subscription.
    std::shared_ptr<autolink::Reader<automsgs::msgs::sensor_msgs::CameraInfo>>
        camera_info_reader_;
    // Cached intrinsics for depth projection.
    std::optional<automsgs::msgs::sensor_msgs::CameraInfo> camera_info_;
    // Timestamp of last cloud or projected depth feed.
    std::chrono::steady_clock::time_point last_cloud_time_{};
    // True after Start() has been called.
    bool started_{false};
};

}  // namespace autonomy::task::teleop
