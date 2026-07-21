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
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/control/controller/teb_controller/core/feasibility_checker.hpp"
#include "autonomy/control/controller/teb_controller/core/homotopy_class_planner.hpp"
#include "autonomy/control/controller/teb_controller/core/misc.hpp"
#include "autonomy/control/controller/teb_controller/core/optimal_planner.hpp"
#include "autonomy/control/controller/teb_controller/core/planner_interface.hpp"
#include "autonomy/control/controller/teb_controller/core/recovery_behaviors.hpp"
#include "autonomy/control/controller/teb_controller/core/teb_config.hpp"
#include "autonomy/control/controller/teb_controller/tools/costmap_obstacle_converter.hpp"
#include "autonomy/control/proto/teb_controller.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {

class Optimizer
{
public:
    void initialize(const std::string& name,
                    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
                    const proto::TEBControllerOptions* options,
                    double controller_frequency);

    commsgs::geometry_msgs::TwistStamped evalControl(
        const commsgs::geometry_msgs::PoseStamped& pose,
        const commsgs::geometry_msgs::Twist& velocity,
        const std::vector<commsgs::geometry_msgs::PoseStamped>& plan,
        const commsgs::geometry_msgs::PoseStamped& goal);

    void setSpeedLimit(double speed_limit, bool percentage);
    void reset();

private:
    void applyOptionsToConfig();
    void updateFootprintFromCostmap();
    void configureRobotModel();
    void validateFootprints() const;
    void applyShrinkHorizon(
        std::vector<commsgs::geometry_msgs::PoseStamped>& plan) const;
    void saturateVelocity(double& vx, double& vy, double& omega) const;
    void markInfeasible();
    void clearInfeasible();

    static PoseSE2 ToPoseSE2(const commsgs::geometry_msgs::Pose& pose);
    static Twist ToTebTwist(const commsgs::geometry_msgs::Twist& twist);

    std::string name_;
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
    const proto::TEBControllerOptions* options_{nullptr};
    TebConfig teb_config_;
    double base_max_vel_x_{0.5};
    double base_max_vel_x_backwards_{0.2};
    double base_max_vel_y_{0.0};
    double base_max_vel_theta_{1.0};
    ObstContainer obstacles_;
    ViaPointContainer via_points_;
    std::unique_ptr<PlannerInterface> planner_;
    std::unique_ptr<tools::CostmapObstacleConverter> obstacle_converter_;
    CostmapFeasibilityModel costmap_model_;
    std::vector<Point> footprint_spec_;
    double robot_inscribed_radius_{0.0};
    double robot_circumscribed_radius_{0.0};
    FailureDetector failure_detector_;
    RotType last_preferred_rotdir_{RotType::none};
    double controller_frequency_{5.0};
    std::chrono::steady_clock::time_point time_last_oscillation_{};
    std::chrono::steady_clock::time_point time_last_infeasible_plan_{};
    int no_infeasible_plans_{0};
    bool has_plan_{false};
};

}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
