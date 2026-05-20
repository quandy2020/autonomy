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

#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include "autonomy/control/controller_server.hpp"
#include "autonomy/control/utils/odometry_utils.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/planning/smoother_server.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace tasks {
namespace common {

/** Shared in-process services for behavior-tree plugins (blackboard key:
 * "task_context"). */
struct TaskContext {
    std::shared_ptr<planning::PlannerServer> planner;
    std::shared_ptr<planning::SmootherServer> smoother;
    std::shared_ptr<control::ControllerServer> controller;
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> global_costmap;
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> local_costmap;
    std::shared_ptr<transform::Buffer> tf;
    std::shared_ptr<control::utils::OdomSmoother> odom_smoother;

    std::string global_frame{"map"};
    std::string robot_base_frame{"base_link"};
    double transform_tolerance{0.1};

    std::string selected_planner_id{"navfn_planner"};
    std::string selected_smoother_id{"simple_smoother"};
    std::string selected_controller_id{"FollowPath"};
    std::string selected_goal_checker_id;
    std::string selected_progress_checker_id;

    std::atomic<bool>* cancel_flag{nullptr};

    bool IsCancelRequested() const {
        return cancel_flag != nullptr && cancel_flag->load();
    }

    std::function<bool()> CancelChecker() const {
        return [this]() { return IsCancelRequested(); };
    }
};

}  // namespace common
}  // namespace tasks
}  // namespace autonomy
