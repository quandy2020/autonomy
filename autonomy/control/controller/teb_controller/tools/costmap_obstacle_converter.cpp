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

#include "autonomy/control/controller/teb_controller/tools/costmap_obstacle_converter.hpp"

#include <cmath>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {
namespace tools {

namespace {
using map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using map::costmap_2d::LETHAL_OBSTACLE;

double getYaw(const commsgs::geometry_msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

CostmapObstacleConverter::CostmapObstacleConverter(
    const proto::TEBControllerOptions& options)
    : options_(options) {}

void CostmapObstacleConverter::update(
    const map::costmap_2d::Costmap2D& costmap,
    const commsgs::geometry_msgs::Pose& robot_pose) {
  obstacles_.clear();
  storage_.clear();

  if (!options_.include_costmap_obstacles()) {
    return;
  }

  const double resolution =
      options_.costmap_obstacle_sample_resolution() > 0.0
          ? options_.costmap_obstacle_sample_resolution()
          : costmap.getResolution();
  const int stride = std::max(
      1, static_cast<int>(std::lround(resolution / costmap.getResolution())));

  const double yaw = getYaw(robot_pose.orientation);
  const double cos_th = std::cos(yaw);
  const double sin_th = std::sin(yaw);
  const double behind_dist =
      options_.costmap_obstacles_behind_robot_dist() > 0.0
          ? options_.costmap_obstacles_behind_robot_dist()
          : 0.0;

  unsigned int size_x = costmap.getSizeInCellsX();
  unsigned int size_y = costmap.getSizeInCellsY();

  for (unsigned int my = 0; my < size_y; my += static_cast<unsigned int>(stride)) {
    for (unsigned int mx = 0; mx < size_x; mx += static_cast<unsigned int>(stride)) {
      const unsigned char cost = costmap.getCost(mx, my);
      if (cost < INSCRIBED_INFLATED_OBSTACLE) {
        continue;
      }

      double wx = 0.0;
      double wy = 0.0;
      costmap.mapToWorld(mx, my, wx, wy);

      const double dx = wx - robot_pose.position.x;
      const double dy = wy - robot_pose.position.y;
      const double forward = dx * cos_th + dy * sin_th;
      if (forward < -behind_dist) {
        continue;
      }

      auto obstacle =
          boost::make_shared<teb_local_planner::PointObstacle>(wx, wy);
      storage_.push_back(obstacle);
      obstacles_.push_back(obstacle);
    }
  }
}

}  // namespace tools
}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
