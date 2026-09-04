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

#ifndef AUTONOMY_PERCEPTION_FATHOM_DEPTH_REFINER_HPP_
#define AUTONOMY_PERCEPTION_FATHOM_DEPTH_REFINER_HPP_

#include "autonomy/perception/fathom/config.hpp"
#include "autonomy/perception/fathom/engine/model.hpp"

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include <memory>
#include <string>

namespace autonomy {
namespace perception {
namespace fathom {

/**
 * @file refiner.hpp
 * @brief Automsgs-facing depth-refinement facade.
 */

/**
 * Converts aligned RGB-D messages to Fathom tensors and reconstructs outputs.
 *
 * The overload accepting a FathomModelRunner keeps facade orchestration
 * independent from the concrete common-network backend and is intended for
 * embedding and focused tests. The no-runner overload creates FathomEngine.
 */
class DepthRefiner {
public:
    static std::unique_ptr<DepthRefiner> Create(const FathomConfig& config,
                                                std::string* error = nullptr);
    static std::unique_ptr<DepthRefiner> Create(
        const FathomConfig& config, std::unique_ptr<FathomModelRunner> runner,
        std::string* error = nullptr);

    DepthRefiner(const DepthRefiner&) = delete;
    DepthRefiner& operator=(const DepthRefiner&) = delete;
    DepthRefiner(DepthRefiner&&) = delete;
    DepthRefiner& operator=(DepthRefiner&&) = delete;

    /**
     * Refine one aligned RGB-D frame and project its valid metric depth.
     *
     * Output messages are cleared before work begins and assigned only after
     * all processing, inference, and projection steps succeed.
     */
    bool Refine(const automsgs::msgs::sensor_msgs::Image& rgb,
                const automsgs::msgs::sensor_msgs::Image& raw_depth,
                const automsgs::msgs::sensor_msgs::CameraInfo& camera_info,
                automsgs::msgs::sensor_msgs::Image* refined_depth,
                automsgs::msgs::sensor_msgs::PointCloud2* point_cloud,
                std::string* error = nullptr);

private:
    DepthRefiner(FathomConfig config,
                 std::unique_ptr<FathomModelRunner> runner);

    FathomConfig config_;
    std::unique_ptr<FathomModelRunner> runner_;
};

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FATHOM_DEPTH_REFINER_HPP_
