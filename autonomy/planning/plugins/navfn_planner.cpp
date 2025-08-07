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

#include "autonomy/planning/plugins/navfn_planner.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/common/logging.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;  // NOLINT
using std::placeholders::_1;

namespace autonomy {
namespace planning {
namespace plugins {

NavfnPlanner::NavfnPlanner()
: tf_(nullptr), costmap_(nullptr)
{
}

NavfnPlanner::NavfnPlanner(
    std::string name, std::shared_ptr<Buffer> tf,
    std::shared_ptr<map::costmap_2d::Costmap2D> costmap)
: tf_(tf.get()), costmap_(costmap.get())
{

}

NavfnPlanner::~NavfnPlanner()
{
    LOG(INFO) << "Destroying plugin " 
              << name_.c_str() 
              << " of type NavfnPlanner.";
}

uint32 NavfnPlanner::MakePlan(
        const commsgs::geometry_msgs::PoseStamped& start,
        const commsgs::geometry_msgs::PoseStamped& goal, double tolerance,
        std::vector<commsgs::geometry_msgs::PoseStamped>& plan, double& cost,
        std::string& message) 
{
    return 0;
}

bool NavfnPlanner::Cancel()
{
    return true;
}

}  // namespace plugins
}  // namespace planning
}  // namespace autonomy