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

#include "autolink/common/log.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {

void CriticManager::configure(std::shared_ptr<autolink::Node> parent, const std::string& name,
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
  for (auto name : critic_names_) {
    std::string fullname = getFullName(name);
    // TODO: Implement plugin loading using
    // autolink::class_loader::ClassLoaderManager For now, this is a
    // placeholder - actual plugin loading needs to be implemented based on
    // the autolink plugin system
    AINFO << "Critic to be loaded: " << fullname;
  }
}

std::string CriticManager::getFullName(const std::string& name) { return "mppi::critics::" + name; }

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