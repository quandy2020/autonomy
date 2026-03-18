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

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/costmap_2d/footprint_collision_checker.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace pure_pursuit_controller {

/**
 * @class nav2_regulated_pure_pursuit_controller::PathHandler
 * @brief Handles input paths to transform them to local frames required
 */
class PathHandler {
 public:
  /**
   * @brief Constructor for nav2_regulated_pure_pursuit_controller::PathHandler
   */
  PathHandler(double transform_tolerance, std::shared_ptr<transform::Buffer> tf,
              std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper);

  /**
   * @brief Destrructor for nav2_regulated_pure_pursuit_controller::PathHandler
   */
  ~PathHandler() = default;

  /**
   * @brief Transforms global plan into same frame as pose and clips poses ineligible for lookaheadPoint
   * Points ineligible to be selected as a lookahead point if they are any of the following:
   * - Outside the local_costmap (collision avoidance cannot be assured)
   * @param pose pose to transform
   * @param max_robot_pose_search_dist Distance to search for matching nearest path point
   * @param reject_unit_path If true, fail if path has only one pose
   * @return Path in new frame
   */
  commsgs::planning_msgs::Path transformGlobalPlan(const commsgs::geometry_msgs::PoseStamped& pose,
                                                   double max_robot_pose_search_dist, bool reject_unit_path = false);

  /**
   * @brief Transform a pose to another frame.
   * @param frame Frame ID to transform to
   * @param in_pose Pose input to transform
   * @param out_pose transformed output
   * @return bool if successful
   */
  bool transformPose(const std::string frame, const commsgs::geometry_msgs::PoseStamped& in_pose,
                     commsgs::geometry_msgs::PoseStamped& out_pose) const;

  void setPlan(const commsgs::planning_msgs::Path& path) { global_plan_ = path; }

  commsgs::planning_msgs::Path getPlan() { return global_plan_; }

 protected:
  /**
   * Get the greatest extent of the costmap in meters from the center.
   * @return max of distance from center in meters to edge of costmap
   */
  double getCostmapMaxExtent() const;

  double transform_tolerance_;
  std::shared_ptr<transform::Buffer> tf_buffer_;
  std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
  commsgs::planning_msgs::Path global_plan_;
};

}  // namespace pure_pursuit_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
