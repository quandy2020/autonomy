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

#include <memory>
#include <string>
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/control/proto/teb_controller.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {
namespace tools {

class PathHandler {
 public:
  void initialize(std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
                  std::shared_ptr<transform::Buffer> tf,
                  const proto::TEBControllerOptions* options);

  void setPath(const commsgs::planning_msgs::Path& path);

  std::vector<commsgs::geometry_msgs::PoseStamped> transformGlobalPlan(
      const commsgs::geometry_msgs::PoseStamped& robot_pose) const;

  commsgs::geometry_msgs::PoseStamped getLocalGoal(
      const commsgs::geometry_msgs::PoseStamped& robot_pose) const;

 private:
  std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
  std::shared_ptr<transform::Buffer> tf_buffer_;
  const proto::TEBControllerOptions* options_{nullptr};
  commsgs::planning_msgs::Path global_plan_;
};

}  // namespace tools
}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
