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

/** Depth image → PointCloud2 → local costmap obstacle layer. */
class RgbdObstacleFeeder
{
public:
    struct Options {
        std::string depth_topic{"/camera/depth/image_raw"};
        std::string camera_frame{"camera_depth_optical_frame"};
        double fx{525.0};
        double fy{525.0};
        double cx{320.0};
        double cy{240.0};
        double min_depth{0.2};
        double max_depth{4.0};
        double min_height{-0.3};
        double max_height{0.5};
        int stride{4};
        double stale_timeout_sec{0.5};
    };

    void Configure(std::shared_ptr<autolink::Node> node,
                   std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
                   const Options& options);

    void Start();

    bool IsCloudFresh() const;

    const Options& options() const { return options_; }

private:
    void OnDepthImage(const std::shared_ptr<commsgs::sensor_msgs::Image>& msg);

    commsgs::sensor_msgs::PointCloud2 ProjectDepth(
        const commsgs::sensor_msgs::Image& image) const;

    Options options_;
    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_;
    std::shared_ptr<autolink::Reader<commsgs::sensor_msgs::Image>> reader_;
    std::chrono::steady_clock::time_point last_cloud_time_{};
    bool started_{false};
};

}  // namespace autonomy::task::teleop
