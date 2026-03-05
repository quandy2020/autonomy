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
#include <utility>
#include <vector>

#include "autolink/autolink.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/control/proto/mppi_controller.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {
namespace tools {

using PathIterator = std::vector<commsgs::geometry_msgs::PoseStamped>::iterator;
using PathRange = std::pair<PathIterator, PathIterator>;

/**
 * @class mppi::PathHandler
 * @brief Manager of incoming reference paths for transformation and processing
 */

class PathHandler {
 public:
  /**
   * @brief Constructor for mppi::PathHandler
   */
  PathHandler() = default;

  /**
   * @brief Destructor for mppi::PathHandler
   */
  ~PathHandler() = default;

  /**
   * @brief Initialize path handler on bringup
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param costmap_ros Costmap2DROS object of environment
   * @param tf TF buffer for transformations
   * @param dynamic_parameter_handler Parameter handler object
   */
  void initialize(std::shared_ptr<autolink::Node> parent, const std::string& name,
                  std::shared_ptr<map::costmap_2d::Costmap2DWrapper>, std::shared_ptr<autonomy::transform::Buffer>,
                  const proto::MPPIControllerOptions*);

  /**
   * @brief Set new reference path
   * @param Plan Path to use
   */
  void setPath(const commsgs::planning_msgs::Path& plan);

  /**
   * @brief Get reference path
   * @return Path
   */
  commsgs::planning_msgs::Path& getPath();

  /**
   * @brief transform global plan to local applying constraints,
   * then prune global plan
   * @param robot_pose Pose of robot
   * @return global plan in local frame
   */
  commsgs::planning_msgs::Path transformPath(const commsgs::geometry_msgs::PoseStamped& robot_pose);

  /**
   * @brief Get the global goal pose transformed to the local frame
   * @param stamp Time to get the goal pose at
   * @return Transformed goal pose
   */
  commsgs::geometry_msgs::PoseStamped getTransformedGoal(const commsgs::builtin_interfaces::Time& stamp);

 protected:
  /**
   * @brief Transform a pose to another frame
   * @param frame Frame to transform to
   * @param in_pose Input pose
   * @param out_pose Output pose
   * @return Bool if successful
   */
  bool transformPose(const std::string& frame, const commsgs::geometry_msgs::PoseStamped& in_pose,
                     commsgs::geometry_msgs::PoseStamped& out_pose) const;

  /**
   * @brief Get largest dimension of costmap (radially)
   * @return Max distance from center of costmap to edge
   */
  double getMaxCostmapDist();

  /**
   * @brief Transform a pose to the global reference frame
   * @param pose Current pose
   * @return output poose in global reference frame
   */
  commsgs::geometry_msgs::PoseStamped transformToGlobalPlanFrame(const commsgs::geometry_msgs::PoseStamped& pose);

  /**
   * @brief Get global plan within window of the local costmap size
   * @param global_pose Robot pose
   * @return plan transformed in the costmap frame and iterator to the first
   * pose of the global plan (for pruning)
   */
  std::pair<commsgs::planning_msgs::Path, PathIterator> getGlobalPlanConsideringBoundsInCostmapFrame(
      const commsgs::geometry_msgs::PoseStamped& global_pose);

  /**
   * @brief Prune a path to only interesting portions
   * @param plan Plan to prune
   * @param end Final path iterator
   */
  void prunePlan(commsgs::planning_msgs::Path& plan, const PathIterator end);

  /**
   * @brief Check if the robot pose is within the set inversion tolerances
   * @param robot_pose Robot's current pose to check
   * @return bool If the robot pose is within the set inversion tolerances
   */
  bool isWithinInversionTolerances(const commsgs::geometry_msgs::PoseStamped& robot_pose);

  std::string name_;
  std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_;
  std::shared_ptr<autonomy::transform::Buffer> tf_buffer_;
  const proto::MPPIControllerOptions* options_;

  commsgs::planning_msgs::Path global_plan_;
  commsgs::planning_msgs::Path global_plan_up_to_inversion_;

  double max_robot_pose_search_dist_{0};
  double prune_distance_{0};
  double transform_tolerance_{0};
  float inversion_xy_tolerance_{0.2};
  float inversion_yaw_tolerance{0.4};
  bool enforce_path_inversion_{false};
  unsigned int inversion_locale_{0u};
};

}  // namespace tools
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy