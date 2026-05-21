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

#include "autonomy/control/controller/mppi_controller/critic_manager.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/control/controller/mppi_controller/critics/constraint_critic.hpp"
#include "autonomy/control/controller/mppi_controller/critics/cost_critic.hpp"
#include "autonomy/control/controller/mppi_controller/critics/goal_angle_critic.hpp"
#include "autonomy/control/controller/mppi_controller/critics/goal_critic.hpp"
#include "autonomy/control/controller/mppi_controller/critics/path_align_critic.hpp"
#include "autonomy/control/controller/mppi_controller/critics/path_angle_critic.hpp"
#include "autonomy/control/controller/mppi_controller/critics/path_follow_critic.hpp"
#include "autonomy/control/controller/mppi_controller/critics/prefer_forward_critic.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {

namespace {

std::unique_ptr<critics::CriticFunction> CreateCritic(const std::string& name) {
    if (name == "ConstraintCritic") {
        return std::make_unique<critics::ConstraintCritic>();
    }
    if (name == "CostCritic") {
        return std::make_unique<critics::CostCritic>();
    }
    if (name == "GoalCritic") {
        return std::make_unique<critics::GoalCritic>();
    }
    if (name == "GoalAngleCritic") {
        return std::make_unique<critics::GoalAngleCritic>();
    }
    if (name == "PathAlignCritic") {
        return std::make_unique<critics::PathAlignCritic>();
    }
    if (name == "PathFollowCritic") {
        return std::make_unique<critics::PathFollowCritic>();
    }
    if (name == "PathAngleCritic") {
        return std::make_unique<critics::PathAngleCritic>();
    }
    if (name == "PreferForwardCritic") {
        return std::make_unique<critics::PreferForwardCritic>();
    }
    return nullptr;
}

}  // namespace

void CriticManager::configure(
    const proto::MPPIControllerOptions& options,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    loadCritics(options, std::move(costmap_wrapper));
}

void CriticManager::loadCritics(
    const proto::MPPIControllerOptions& options,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    critics_.clear();
    for (const auto& name : options.critics()) {
        auto critic = CreateCritic(name);
        if (!critic) {
            AWARN << "MPPI CriticManager: unknown critic '" << name << "'";
            continue;
        }
        critic->Configure(options, costmap_wrapper);
        AINFO << "MPPI critic loaded: " << name;
        critics_.push_back(std::move(critic));
    }
}

void CriticManager::evalTrajectoriesScores(CriticData& data) const {
    for (const auto& critic : critics_) {
        if (data.fail_flag) {
            break;
        }
        critic->score(data);
    }
}

}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
