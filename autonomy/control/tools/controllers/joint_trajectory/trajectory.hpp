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

#ifndef AUTONOMY_CONTROL_TOOLS_CONTROLLERS_JOINT_TRAJECTORY__TRAJECTORY_HPP_
#define AUTONOMY_CONTROL_TOOLS_CONTROLLERS_JOINT_TRAJECTORY__TRAJECTORY_HPP_

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

#include "autonomy/control/tools/controllers/joint_trajectory/interpolation_methods.hpp"
#include "autonomy/control/tools/controllers/joint_trajectory/trajectory_types.hpp"

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace joint_trajectory
{

using TrajectoryPointIter = std::vector<TrajectoryPoint>::iterator;
using TrajectoryPointConstIter = std::vector<TrajectoryPoint>::const_iterator;

class Trajectory
{
public:
  Trajectory();

  explicit Trajectory(std::shared_ptr<JointTrajectory> joint_trajectory);

  Trajectory(
    double current_time_sec, const TrajectoryPoint & current_point,
    std::shared_ptr<JointTrajectory> joint_trajectory);

  void set_point_before_trajectory_msg(
    double current_time_sec, const TrajectoryPoint & current_point,
    const std::vector<bool> & joints_angle_wraparound = std::vector<bool>());

  void update(std::shared_ptr<JointTrajectory> joint_trajectory);

  bool sample(
    double sample_time_sec, interpolation_methods::InterpolationMethod interpolation_method,
    TrajectoryPoint & output_state, TrajectoryPointConstIter & start_segment_itr,
    TrajectoryPointConstIter & end_segment_itr, bool search_monotonically_increasing = true);

  void interpolate_between_points(
    double time_a_sec, const TrajectoryPoint & state_a, double time_b_sec,
    const TrajectoryPoint & state_b, double sample_time_sec, TrajectoryPoint & output);

  TrajectoryPointConstIter begin() const;
  TrajectoryPointConstIter end() const;

  double time_from_start() const;

  bool has_trajectory_msg() const;
  bool has_nontrivial_msg() const;

  std::shared_ptr<JointTrajectory> get_trajectory_msg() const { return trajectory_msg_; }

  bool is_sampled_already() const { return sampled_already_; }

  size_t last_sample_index() const { return last_sample_idx_; }

private:
  void deduce_from_derivatives(
    TrajectoryPoint & first_state, TrajectoryPoint & second_state, size_t dim, double delta_t);

  std::shared_ptr<JointTrajectory> trajectory_msg_;
  double trajectory_start_time_;

  double time_before_traj_msg_;
  TrajectoryPoint state_before_traj_msg_;

  bool sampled_already_ = false;
  size_t last_sample_idx_ = 0;
};

template <class T>
inline std::vector<size_t> mapping(const T & t1, const T & t2)
{
  if (t1.size() > t2.size())
  {
    return std::vector<size_t>();
  }

  std::vector<size_t> mapping_vector(t1.size());
  for (auto t1_it = t1.begin(); t1_it != t1.end(); ++t1_it)
  {
    auto t2_it = std::find(t2.begin(), t2.end(), *t1_it);
    if (t2.end() == t2_it)
    {
      return std::vector<size_t>();
    }
    const size_t t1_dist = static_cast<size_t>(std::distance(t1.begin(), t1_it));
    const size_t t2_dist = static_cast<size_t>(std::distance(t2.begin(), t2_it));
    mapping_vector[t1_dist] = t2_dist;
  }
  return mapping_vector;
}

void wraparound_joint(
  std::vector<double> & current_position, const std::vector<double> & next_position,
  const std::vector<bool> & joints_angle_wraparound);

}  // namespace joint_trajectory
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS_CONTROLLERS_JOINT_TRAJECTORY__TRAJECTORY_HPP_
