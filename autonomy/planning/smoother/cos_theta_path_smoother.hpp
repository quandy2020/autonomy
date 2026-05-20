/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/planning/common/smoother_interface.hpp"
#include "autonomy/planning/math/discretized_points_smoothing/cos_theta_smoother.hpp"
#include "autonomy/planning/proto/math/cos_theta_smoother_config.pb.h"

namespace autonomy {
namespace planning {
namespace smoother {

/** Smoother plugin wrapping cos-theta IPOPT optimization. */
class CosThetaPathSmoother : public common::Smoother
{
public:
    CosThetaPathSmoother() = default;
    ~CosThetaPathSmoother() override = default;

    void Configure(
        std::string name, std::shared_ptr<void> /*costmap_sub*/,
        std::shared_ptr<map::costmap_2d::Costmap2DWrapper> /*costmap_wrapper*/)
        override;

    void ApplyOptions(const proto::math::CosThetaSmootherConfig& config,
                      double path_bound);

    void Cleanup() override {}
    void Activate() override {}
    void Deactivate() override {}

    bool Smooth(commsgs::planning_msgs::Path& path,
                const std::chrono::milliseconds& max_time) override;

private:
    std::string name_;
    proto::math::CosThetaSmootherConfig config_;
    double path_bound_{0.5};
    std::unique_ptr<math::CosThetaSmoother> solver_;
};

}  // namespace smoother
}  // namespace planning
}  // namespace autonomy
