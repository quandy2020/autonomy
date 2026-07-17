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

#include <memory>
#include <string>
#include <utility>

#include "autolink/common/log.hpp"
#include "autonomy/control/controller/mppi_controller/critics/cost_critic.hpp"
#include "autonomy/control/controller/mppi_controller/critics/path_follow_critic.hpp"
#include "autonomy/control/controller/mppi_controller/critics/prefer_forward_critic.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {
namespace {

std::string NormalizeCriticName(std::string name) {
    const std::string prefix = "mppi::critics::";
    if (name.rfind(prefix, 0) == 0) {
        name = name.substr(prefix.size());
    }
    return name;
}

std::unique_ptr<CriticFunction> CreateCriticByName(const std::string& raw_name) {
    const std::string name = NormalizeCriticName(raw_name);

    if (name == "CostCritic" || name == "cost_critic") {
        return std::make_unique<critics::CostCritic>();
    }
    if (name == "PathFollowCritic" || name == "path_follow_critic") {
        return std::make_unique<critics::PathFollowCritic>();
    }
    if (name == "PreferForwardCritic" || name == "prefer_forward_critic") {
        return std::make_unique<critics::PreferForwardCritic>();
    }
    return nullptr;
}

}  // namespace

void CriticManager::configure(
    std::shared_ptr<autolink::Node> parent, const std::string& name,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_ros,
    const proto::MPPIControllerOptions* options) {
    parent_ = parent;
    costmap_ros_ = costmap_ros;
    name_ = name;
    options_ = options;

    getParams();
    loadCritics();
}

void CriticManager::getParams() {
    critic_names_.clear();
    if (options_) {
        for (int i = 0; i < options_->critics_size(); ++i) {
            critic_names_.push_back(options_->critics(i));
        }
    }
}

void CriticManager::loadCritics() {
    critics_.clear();

    auto append_critic = [this](const std::string& critic_name) {
        auto critic = CreateCriticByName(critic_name);
        if (!critic) {
            AERROR << "Unknown or unsupported MPPI critic (static registry): "
                   << critic_name;
            return;
        }
        const std::string short_name = NormalizeCriticName(critic_name);
        critic->configure(parent_, name_, short_name, costmap_ros_, options_);
        critics_.push_back(std::move(critic));
        AINFO << "Loaded MPPI critic: " << short_name;
    };

    for (const auto& name : critic_names_) {
        append_critic(name);
    }

    if (critics_.empty()) {
        AWARN << "No MPPI critics loaded from options; "
                 "using CostCritic + PathFollowCritic defaults";
        append_critic("CostCritic");
        append_critic("PathFollowCritic");
    }
}

std::string CriticManager::getFullName(const std::string& name) {
    return "mppi::critics::" + name;
}

void CriticManager::evalTrajectoriesScores(CriticData& data) const {
    for (const auto& critic : critics_) {
        if (data.fail_flag) {
            break;
        }
        critic->score(data);
    }
}

}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
