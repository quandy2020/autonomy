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
#include <string>

#include "autolink/node/node.hpp"
#include "autolink/node/reader.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy::task::teleop {

/** PointCloud2 → local costmap obstacle layer (RGB-D, stereo, lidar, …). */
class PointCloudObstacleFeeder
{
public:
    struct Options {
        std::string cloud_topic{"/camera/depth/points"};
        double stale_timeout_sec{0.5};
    };

    void Configure(std::shared_ptr<autolink::Node> node,
                   std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
                   const Options& options);

    void Start();
    void Stop();

    bool IsCloudFresh() const;

    const Options& options() const { return options_; }

private:
    void OnPointCloud(
        const std::shared_ptr<commsgs::sensor_msgs::PointCloud2>& msg);

    Options options_;
    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_;
    std::shared_ptr<autolink::Reader<commsgs::sensor_msgs::PointCloud2>>
        reader_;
    std::chrono::steady_clock::time_point last_cloud_time_{};
    bool started_{false};
};

}  // namespace autonomy::task::teleop
