// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Ported into autonomy::control::tools::controllers
// ROS-free POD stand-ins for trajectory_msgs JointTrajectory{,Point}.

#ifndef AUTONOMY_CONTROL_TOOLS_CONTROLLERS_JOINT_TRAJECTORY__TRAJECTORY_TYPES_HPP_
#define AUTONOMY_CONTROL_TOOLS_CONTROLLERS_JOINT_TRAJECTORY__TRAJECTORY_TYPES_HPP_

#include <string>
#include <vector>

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace joint_trajectory
{

struct TrajectoryPoint
{
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
  std::vector<double> effort;
  /// Absolute offset from trajectory start [s].
  double time_from_start{0.0};
};

struct JointTrajectory
{
  /// Trajectory reference stamp [s]. Zero means "use first sample time".
  double header_stamp{0.0};
  std::vector<std::string> joint_names;
  std::vector<TrajectoryPoint> points;
};

}  // namespace joint_trajectory
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS_CONTROLLERS_JOINT_TRAJECTORY__TRAJECTORY_TYPES_HPP_
