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

#include "autolink/autolink.hpp"
#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/controller/teb_controller/optimizer.hpp"
#include "autonomy/control/controller/teb_controller/tools/path_handler.hpp"
#include "autonomy/control/proto/teb_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {

class TEBController : public common::ControllerInterface {
 public:
  TEBController() = default;

  void Configure(const proto::ControllerOptions& options, std::string name,
                 std::shared_ptr<transform::Buffer> tf,
                 std::shared_ptr<map::costmap_2d::Costmap2DWrapper>
                     costmap_wrapper);

  void Cleanup();
  void Activate();
  void Deactivate();
  void Reset() override;

  uint32 ComputeVelocityCommands(
      const commsgs::geometry_msgs::PoseStamped& pose,
      const commsgs::geometry_msgs::TwistStamped& velocity,
      commsgs::geometry_msgs::TwistStamped& cmd_vel,
      common::GoalChecker* goal_checker, std::string& message) override;

  void SetPlan(const commsgs::planning_msgs::Path& path) override;
  void SetSpeedLimit(const double& speed_limit,
                     const bool& percentage) override;
  bool IsGoalReached(double dist_tolerance, double angle_tolerance) override;

 private:
  std::string name_;
  proto::TEBControllerOptions options_;
  std::shared_ptr<transform::Buffer> tf_buffer_;
  std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
  Optimizer optimizer_;
  tools::PathHandler path_handler_;

  commsgs::geometry_msgs::PoseStamped last_robot_pose_;
  commsgs::geometry_msgs::Twist last_robot_velocity_;
  commsgs::geometry_msgs::Pose last_goal_pose_;
  common::GoalChecker* last_goal_checker_{nullptr};
  bool has_control_state_{false};
};

}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
