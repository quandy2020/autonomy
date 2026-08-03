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

#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <automsgs/msgs/planning_msgs/planning_msgs.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/common/planner_interface.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"

namespace autonomy {
namespace planning {
namespace planner {
namespace theta_star {

/** Grid-based Theta* any-angle global planner. */
class ThetaStarPlanner : public common::GlobalPlanner
{
public:
    ThetaStarPlanner() = default;

    ThetaStarPlanner(const proto::PlannerOptions& options,
                     const std::string& name,
                     std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap);

    ~ThetaStarPlanner() override;

    uint32 CreatePlan(const automsgs::msgs::geometry_msgs::PoseStamped& start,
                      const automsgs::msgs::geometry_msgs::PoseStamped& goal,
                      automsgs::msgs::planning_msgs::Path& plan,
                      std::function<bool()> cancel_checker) override;

private:
    bool makePlan(const automsgs::msgs::geometry_msgs::Pose& start,
                  const automsgs::msgs::geometry_msgs::Pose& goal,
                  std::function<bool()> cancel_checker,
                  automsgs::msgs::planning_msgs::Path& plan);

    bool worldToMap(double wx, double wy, unsigned int& mx,
                    unsigned int& my) const;
    void mapToWorld(unsigned int mx, unsigned int my, double& wx,
                    double& wy) const;

    bool isCellBlocked(unsigned char cost) const;
    bool lineOfSight(unsigned int x0, unsigned int y0, unsigned int x1,
                     unsigned int y1) const;

    double euclideanDistance(unsigned int x0, unsigned int y0, unsigned int x1,
                             unsigned int y1) const;
    double traversalCost(unsigned int x, unsigned int y) const;
    double heuristic(unsigned int x, unsigned int y, unsigned int goal_x,
                     unsigned int goal_y) const;

    void InitFromOptions();

    std::vector<unsigned char> planning_costmap_copy_;
    std::string global_frame_;

    unsigned int size_x_{0};
    unsigned int size_y_{0};

    int how_many_corners_{8};
    bool allow_unknown_{true};
    double w_euc_cost_{2.0};
    double w_traversal_cost_{1.0};
    double w_heuristic_cost_{1.0};
    int terminal_checking_interval_{5000};
};

}  // namespace theta_star
}  // namespace planner
}  // namespace planning
}  // namespace autonomy
