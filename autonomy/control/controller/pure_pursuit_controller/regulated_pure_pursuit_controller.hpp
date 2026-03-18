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
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autolink/autolink.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/common/goal_checker_interface.hpp"
#include "autonomy/control/controller/pure_pursuit_controller/collision_checker.hpp"
#include "autonomy/control/controller/pure_pursuit_controller/parameter_handler.hpp"
#include "autonomy/control/controller/pure_pursuit_controller/path_handler.hpp"
#include "autonomy/control/controller/pure_pursuit_controller/regulation_functions.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace pure_pursuit_controller {

/**
 * @class nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
 * @brief Regulated pure pursuit controller plugin
 */
class RegulatedPurePursuitController : public common::ControllerInterface {
 public:
  /**
   * @brief Constructor for nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
   */
  RegulatedPurePursuitController() = default;

  /**
   * @brief Destrructor for nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
   */
  ~RegulatedPurePursuitController() override = default;

  /**
   * @brief Configure controller state machine
   * @param options Controller options
   * @param name Name of plugin
   * @param tf_buffer TF buffer
   * @param costmap_wrapper Costmap2DWrapper object of environment
   */
  void Configure(const proto::ControllerOptions& options, std::string name,
                 std::shared_ptr<transform::Buffer> tf_buffer,
                 std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) override;

  /**
   * @brief Cleanup controller state machine
   */
  void Cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void Activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void Deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param cmd_vel   Output velocity command
   * @param goal_checker   Ptr to the goal checker for this task
   * @param message   Optional detailed outcome message
   * @return          Result code (0 = success)
   */
  uint32 ComputeVelocityCommands(const commsgs::geometry_msgs::PoseStamped& pose,
                                 const commsgs::geometry_msgs::TwistStamped& velocity,
                                 commsgs::geometry_msgs::TwistStamped& cmd_vel, common::GoalChecker* goal_checker,
                                 std::string& message) override;

  /**
   * @brief Check if the goal pose has been achieved
   * @param dist_tolerance Distance tolerance
   * @param angle_tolerance Angle tolerance
   * @return True if goal reached
   */
  bool IsGoalReached(double dist_tolerance, double angle_tolerance) override;

  /**
   * @brief Set the global plan
   * @param path The global plan
   */
  void SetPlan(const commsgs::planning_msgs::Path& path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void SetSpeedLimit(const double& speed_limit, const bool& percentage) override;

  /**
   * @brief Reset the state of the controller
   */
  void Reset() override;

  // Legacy methods for compatibility (not part of ControllerInterface)
  bool cancel();

 protected:
  /**
   * @brief Get lookahead distance
   * @param cmd the current speed to use to compute lookahead point
   * @return lookahead distance
   */
  double getLookAheadDistance(const commsgs::geometry_msgs::TwistStamped&);

  /**
   * @brief Creates a PointStamped message for visualization
   * @param carrot_pose Input carrot point as a PoseStamped
   * @return CarrotMsg a carrot point marker, PointStamped
   */
  std::unique_ptr<commsgs::geometry_msgs::PointStamped> createCarrotMsg(
      const commsgs::geometry_msgs::PoseStamped& carrot_pose);

  /**
   * @brief Whether robot should rotate to rough path heading
   * @param carrot_pose current lookahead point
   * @param angle_to_path Angle of robot output relative to carrot marker
   * @param x_vel_sign Velocoty sign (forward or backward)
   * @return Whether should rotate to path heading
   */
  bool shouldRotateToPath(const commsgs::geometry_msgs::PoseStamped& carrot_pose, double& angle_to_path,
                          double& x_vel_sign);

  /**
   * @brief Whether robot should rotate to final goal orientation
   * @param carrot_pose current lookahead point
   * @return Whether should rotate to goal heading
   */
  bool shouldRotateToGoalHeading(const commsgs::geometry_msgs::PoseStamped& carrot_pose);

  /**
   * @brief Create a smooth and kinematically smoothed rotation command
   * @param linear_vel linear velocity
   * @param angular_vel angular velocity
   * @param angle_to_path Angle of robot output relative to carrot marker
   * @param curr_speed the current robot speed
   */
  void rotateToHeading(double& linear_vel, double& angular_vel, const double& angle_to_path,
                       const commsgs::geometry_msgs::TwistStamped& curr_speed);

  /**
   * @brief apply regulation constraints to the system
   * @param linear_vel robot command linear velocity input
   * @param lookahead_dist optimal lookahead distance
   * @param curvature curvature of path
   * @param speed Speed of robot
   * @param pose_cost cost at this pose
   */
  void applyConstraints(const double& curvature, const commsgs::geometry_msgs::TwistStamped& speed,
                        const double& pose_cost, const commsgs::planning_msgs::Path& path, double& linear_vel,
                        double& sign);

  /**
   * @brief Find the intersection a circle and a line segment.
   * This assumes the circle is centered at the origin.
   * If no intersection is found, a floating point error will occur.
   * @param p1 first endpoint of line segment
   * @param p2 second endpoint of line segment
   * @param r radius of circle
   * @return point of intersection
   */
  static commsgs::geometry_msgs::Point circleSegmentIntersection(const commsgs::geometry_msgs::Point& p1,
                                                                 const commsgs::geometry_msgs::Point& p2, double r);

  /**
   * @brief Get lookahead point
   * @param lookahead_dist Optimal lookahead distance
   * @param path Current global path
   * @param interpolate_after_goal If true, interpolate the lookahead point after the goal based
   * on the orientation given by the position of the last two pose of the path
   * @return Lookahead point
   */
  commsgs::geometry_msgs::PoseStamped getLookAheadPoint(const double&, const commsgs::planning_msgs::Path&,
                                                        bool interpolate_after_goal = false);

  /**
   * @brief checks for the cusp position
   * @param pose Pose input to determine the cusp position
   * @return robot distance from the cusp
   */
  double findVelocitySignChange(const commsgs::planning_msgs::Path& transformed_plan);

  std::shared_ptr<autolink::Node> node_;
  std::shared_ptr<transform::Buffer> tf_buffer_;
  std::string plugin_name_;
  std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
  map::costmap_2d::Costmap2D* costmap_;

  const proto::PurePursuitControllerOptions* pp_options_ = nullptr;
  double goal_dist_tol_;
  double control_duration_;
  bool cancelling_ = false;
  bool finished_cancelling_ = false;
  bool is_rotating_to_heading_ = false;
  bool has_reached_xy_tolerance_ = false;
  double last_dist_to_goal_ = std::numeric_limits<double>::infinity();
  double last_angle_to_goal_ = std::numeric_limits<double>::infinity();

  std::shared_ptr<autolink::Writer<commsgs::planning_msgs::Path>> global_path_pub_;
  std::shared_ptr<autolink::Writer<commsgs::geometry_msgs::PointStamped>> carrot_pub_;
  std::shared_ptr<autolink::Writer<commsgs::geometry_msgs::PointStamped>> curvature_carrot_pub_;
  // TODO: Add Bool message type to commsgs or use a different approach
  // std::shared_ptr<autolink::Writer<commsgs::std_msgs::Bool>> is_rotating_to_heading_pub_;
  std::shared_ptr<autolink::Writer<commsgs::planning_msgs::Path>> carrot_arc_pub_;
  std::unique_ptr<PathHandler> path_handler_;
  std::unique_ptr<ParameterHandler> param_handler_;
  std::unique_ptr<CollisionChecker> collision_checker_;
};

}  // namespace pure_pursuit_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
