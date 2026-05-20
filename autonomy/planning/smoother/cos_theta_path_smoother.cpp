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

#include "autonomy/planning/smoother/cos_theta_path_smoother.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"

#include <utility>
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/planning/utils/smoother_utils.hpp"

namespace autonomy {
namespace planning {
namespace smoother {

void CosThetaPathSmoother::Configure(
    std::string name, std::shared_ptr<void> /*costmap_sub*/,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> /*costmap_wrapper*/) {
    name_ = std::move(name);
    if (!solver_) {
        solver_ = std::make_unique<math::CosThetaSmoother>(config_);
    }
    AINFO << "Configured CosThetaPathSmoother plugin: " << name_;
}

void CosThetaPathSmoother::ApplyOptions(
    const proto::math::CosThetaSmootherConfig& config, double path_bound) {
    config_ = config;
    path_bound_ = path_bound > 0.0 ? path_bound : 0.5;
    solver_ = std::make_unique<math::CosThetaSmoother>(config_);
}

bool CosThetaPathSmoother::Smooth(commsgs::planning_msgs::Path& path,
                                  const std::chrono::milliseconds& /*max_time*/) {
    if (path.poses.size() < 2 || !solver_) {
        return true;
    }

    std::vector<std::pair<double, double>> raw_points;
    raw_points.reserve(path.poses.size());
    for (const auto& pose : path.poses) {
        raw_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }

    std::vector<double> bounds(path.poses.size(), path_bound_);
    std::vector<double> opt_x;
    std::vector<double> opt_y;
    if (!solver_->Solve(raw_points, bounds, &opt_x, &opt_y)) {
        AWARN << "CosThetaPathSmoother failed to optimize path";
        return false;
    }
    if (opt_x.size() != path.poses.size() || opt_y.size() != path.poses.size()) {
        AWARN << "CosThetaPathSmoother returned unexpected point count";
        return false;
    }

    for (size_t i = 0; i < path.poses.size(); ++i) {
        path.poses[i].pose.position.x = opt_x[i];
        path.poses[i].pose.position.y = opt_y[i];
    }

    if (path.poses.size() >= 3) {
        bool reversing_segment = false;
        utils::updateApproximatePathOrientations(path, reversing_segment, true);
    }
    return true;
}

}  // namespace smoother
}  // namespace planning
}  // namespace autonomy

using autonomy::planning::common::Smoother;
using autonomy::planning::smoother::CosThetaPathSmoother;

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(CosThetaPathSmoother, Smoother);
