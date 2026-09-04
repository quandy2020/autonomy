/*
 * Copyright 2025 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_PERCEPTION_FATHOM_FATHOM_NODE_RUNNER_HPP_
#define AUTONOMY_PERCEPTION_FATHOM_FATHOM_NODE_RUNNER_HPP_

#include "autonomy/perception/fathom/config.hpp"
#include "autonomy/perception/fathom/depth/refiner.hpp"

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include <memory>
#include <string>

namespace autonomy {
namespace perception {
namespace fathom {

/**
 * Owns a configured Fathom refiner for synchronous process-facing use.
 *
 * Model initialization happens in Create. The runner deliberately does not
 * define transport topics or perception-server options; callers provide and
 * receive existing automsgs sensor messages for each frame.
 */
class FathomNodeRunner {
public:
    static std::unique_ptr<FathomNodeRunner> Create(
        const FathomConfig& config, std::string* error = nullptr);

    FathomNodeRunner(const FathomNodeRunner&) = delete;
    FathomNodeRunner& operator=(const FathomNodeRunner&) = delete;
    FathomNodeRunner(FathomNodeRunner&&) = delete;
    FathomNodeRunner& operator=(FathomNodeRunner&&) = delete;

    /** Refine one aligned RGB-D frame and emit metric depth plus XYZ cloud. */
    bool Process(const automsgs::msgs::sensor_msgs::Image& rgb,
                 const automsgs::msgs::sensor_msgs::Image& raw_depth,
                 const automsgs::msgs::sensor_msgs::CameraInfo& camera_info,
                 automsgs::msgs::sensor_msgs::Image* refined_depth,
                 automsgs::msgs::sensor_msgs::PointCloud2* point_cloud,
                 std::string* error = nullptr);

private:
    explicit FathomNodeRunner(std::unique_ptr<DepthRefiner> refiner);

    std::unique_ptr<DepthRefiner> refiner_;
};

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FATHOM_FATHOM_NODE_RUNNER_HPP_
