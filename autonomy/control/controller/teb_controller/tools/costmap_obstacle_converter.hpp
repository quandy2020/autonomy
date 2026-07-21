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

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include "autonomy/control/controller/teb_controller/core/obstacles.hpp"

#include "autonomy/control/proto/teb_controller.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {
namespace tools {

/**
 * @brief Convert lethal costmap cells to TEB point obstacles (non-ROS).
 *
 * Inspired by costmap_converter point sampling; clusters are not required for
 * the first integration — grid downsampling keeps the optimization tractable.
 */
class CostmapObstacleConverter {
 public:
  explicit CostmapObstacleConverter(const proto::TEBControllerOptions& options);

  void update(const map::costmap_2d::Costmap2D& costmap,
              const commsgs::geometry_msgs::Pose& robot_pose);

  teb_local_planner::ObstContainer& obstacles() { return obstacles_; }
  const teb_local_planner::ObstContainer& obstacles() const {
    return obstacles_;
  }

 private:
  proto::TEBControllerOptions options_;
  teb_local_planner::ObstContainer obstacles_;
  std::vector<boost::shared_ptr<teb_local_planner::Obstacle>> storage_;
};

}  // namespace tools
}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
