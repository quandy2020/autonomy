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

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "autonomy/control/tools/controllers/joint_trajectory/trajectory.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace joint_trajectory
{

namespace
{
inline double shortest_angular_distance(double from, double to)
{
  double result = std::fmod(to - from + M_PI, 2.0 * M_PI);
  if (result < 0.0)
  {
    result += 2.0 * M_PI;
  }
  return result - M_PI;
}

inline void require_trajectory(const std::shared_ptr<JointTrajectory> & msg)
{
  if (!msg)
  {
    throw std::runtime_error("Trajectory message pointer is null");
  }
}
}  // namespace

Trajectory::Trajectory() : trajectory_start_time_(0.0), time_before_traj_msg_(0.0) {}

Trajectory::Trajectory(std::shared_ptr<JointTrajectory> joint_trajectory)
: trajectory_msg_(joint_trajectory), trajectory_start_time_(joint_trajectory->header_stamp)
{
}

Trajectory::Trajectory(
  double current_time_sec, const TrajectoryPoint & current_point,
  std::shared_ptr<JointTrajectory> joint_trajectory)
: trajectory_msg_(joint_trajectory), trajectory_start_time_(joint_trajectory->header_stamp)
{
  set_point_before_trajectory_msg(current_time_sec, current_point);
  update(joint_trajectory);
}

void Trajectory::set_point_before_trajectory_msg(
  double current_time_sec, const TrajectoryPoint & current_point,
  const std::vector<bool> & joints_angle_wraparound)
{
  time_before_traj_msg_ = current_time_sec;
  state_before_traj_msg_ = current_point;

  if (current_point.velocities.empty() && !trajectory_msg_->points[0].velocities.empty())
  {
    state_before_traj_msg_.velocities.resize(trajectory_msg_->points[0].velocities.size(), 0.0);
  }
  if (current_point.accelerations.empty() && !trajectory_msg_->points[0].accelerations.empty())
  {
    state_before_traj_msg_.accelerations.resize(
      trajectory_msg_->points[0].accelerations.size(), 0.0);
  }

  wraparound_joint(
    state_before_traj_msg_.positions, trajectory_msg_->points[0].positions,
    joints_angle_wraparound);
}

void wraparound_joint(
  std::vector<double> & current_position, const std::vector<double> & next_position,
  const std::vector<bool> & joints_angle_wraparound)
{
  for (size_t i = 0; i < joints_angle_wraparound.size(); i++)
  {
    if (joints_angle_wraparound[i])
    {
      double dist = shortest_angular_distance(current_position[i], next_position[i]);

      if (std::abs(std::abs(dist) - M_PI) < 1e-9)
      {
        dist = next_position[i] > current_position[i] ? std::abs(dist) : -std::abs(dist);
      }

      current_position[i] = next_position[i] - dist;
    }
  }
}

void Trajectory::update(std::shared_ptr<JointTrajectory> joint_trajectory)
{
  trajectory_msg_ = joint_trajectory;
  trajectory_start_time_ = joint_trajectory->header_stamp;
  sampled_already_ = false;
  last_sample_idx_ = 0;
}

bool Trajectory::sample(
  double sample_time_sec, interpolation_methods::InterpolationMethod interpolation_method,
  TrajectoryPoint & output_state, TrajectoryPointConstIter & start_segment_itr,
  TrajectoryPointConstIter & end_segment_itr, bool search_monotonically_increasing)
{
  require_trajectory(trajectory_msg_);

  if (trajectory_msg_->points.empty())
  {
    start_segment_itr = end();
    end_segment_itr = end();
    return false;
  }

  if (!sampled_already_)
  {
    if (trajectory_start_time_ == 0.0)
    {
      trajectory_start_time_ = sample_time_sec;
    }
    sampled_already_ = true;
  }

  if (sample_time_sec < time_before_traj_msg_)
  {
    return false;
  }

  output_state = TrajectoryPoint();
  auto & first_point_in_msg = trajectory_msg_->points[0];
  const double first_point_timestamp =
    trajectory_start_time_ + first_point_in_msg.time_from_start;

  if (sample_time_sec < first_point_timestamp)
  {
    if (interpolation_method == interpolation_methods::InterpolationMethod::NONE)
    {
      output_state = state_before_traj_msg_;
    }
    else
    {
      deduce_from_derivatives(
        state_before_traj_msg_, first_point_in_msg, state_before_traj_msg_.positions.size(),
        first_point_timestamp - time_before_traj_msg_);

      interpolate_between_points(
        time_before_traj_msg_, state_before_traj_msg_, first_point_timestamp, first_point_in_msg,
        sample_time_sec, output_state);
    }
    start_segment_itr = begin();
    end_segment_itr = begin();
    return true;
  }

  const auto last_idx = trajectory_msg_->points.size() - 1;
  for (size_t i = last_sample_idx_; i < last_idx; ++i)
  {
    auto & point = trajectory_msg_->points[i];
    auto & next_point = trajectory_msg_->points[i + 1];

    const double t0 = trajectory_start_time_ + point.time_from_start;
    const double t1 = trajectory_start_time_ + next_point.time_from_start;

    if (sample_time_sec >= t0 && sample_time_sec < t1)
    {
      if (interpolation_method == interpolation_methods::InterpolationMethod::NONE)
      {
        output_state = next_point;
      }
      else
      {
        deduce_from_derivatives(
          point, next_point, state_before_traj_msg_.positions.size(), t1 - t0);

        interpolate_between_points(t0, point, t1, next_point, sample_time_sec, output_state);
      }
      start_segment_itr = begin() + static_cast<TrajectoryPointConstIter::difference_type>(i);
      end_segment_itr = begin() + static_cast<TrajectoryPointConstIter::difference_type>(i + 1);
      output_state.time_from_start = next_point.time_from_start;
      if (search_monotonically_increasing)
      {
        last_sample_idx_ = i;
      }
      return true;
    }
  }

  start_segment_itr = --end();
  end_segment_itr = end();
  last_sample_idx_ = last_idx;

  if (trajectory_msg_->points[last_idx].positions.empty() && last_idx > 0)
  {
    auto & prev_point = trajectory_msg_->points[last_idx - 1];
    auto & last_point = trajectory_msg_->points[last_idx];
    if (!prev_point.positions.empty())
    {
      const double t_prev = trajectory_start_time_ + prev_point.time_from_start;
      const double t_last = trajectory_start_time_ + last_point.time_from_start;
      deduce_from_derivatives(
        prev_point, last_point, state_before_traj_msg_.positions.size(), t_last - t_prev);
    }
  }

  output_state = (*start_segment_itr);
  if (output_state.positions.empty())
  {
    start_segment_itr = end();
    end_segment_itr = end();
    return false;
  }
  if (output_state.velocities.empty())
  {
    output_state.velocities.resize(output_state.positions.size(), 0.0);
  }
  if (output_state.accelerations.empty())
  {
    output_state.accelerations.resize(output_state.positions.size(), 0.0);
  }
  if (output_state.effort.empty())
  {
    output_state.effort.resize(output_state.positions.size(), 0.0);
  }
  return true;
}

void Trajectory::interpolate_between_points(
  double time_a_sec, const TrajectoryPoint & state_a, double time_b_sec,
  const TrajectoryPoint & state_b, double sample_time_sec, TrajectoryPoint & output)
{
  double duration_so_far = sample_time_sec - time_a_sec;
  const double duration_btwn_points = time_b_sec - time_a_sec;

  const size_t dim = state_a.positions.size();
  output.positions.resize(dim, 0.0);
  output.velocities.resize(dim, 0.0);
  output.accelerations.resize(dim, 0.0);
  output.effort.resize(dim, 0.0);

  auto generate_powers = [](int n, double x, double * powers) {
    powers[0] = 1.0;
    for (int i = 1; i <= n; ++i)
    {
      powers[i] = powers[i - 1] * x;
    }
  };

  bool has_velocity = !state_a.velocities.empty() && !state_b.velocities.empty();
  bool has_accel = !state_a.accelerations.empty() && !state_b.accelerations.empty();
  bool has_effort = !state_a.effort.empty() && !state_b.effort.empty();
  if (duration_so_far < 0.0)
  {
    duration_so_far = 0.0;
    has_velocity = has_accel = false;
  }
  if (duration_so_far > duration_btwn_points)
  {
    duration_so_far = duration_btwn_points;
    has_velocity = has_accel = false;
  }

  double t[6];
  generate_powers(5, duration_so_far, t);

  if (has_effort)
  {
    for (size_t i = 0; i < dim; ++i)
    {
      double coefficients[2] = {0.0, 0.0};
      coefficients[0] = state_a.effort[i];
      if (duration_btwn_points != 0.0)
      {
        coefficients[1] = (state_b.effort[i] - state_a.effort[i]) / duration_btwn_points;
      }
      output.effort[i] = t[0] * coefficients[0] + t[1] * coefficients[1];
    }
  }

  if (!has_velocity && !has_accel)
  {
    for (size_t i = 0; i < dim; ++i)
    {
      double coefficients[2] = {0.0, 0.0};
      coefficients[0] = state_a.positions[i];
      if (duration_btwn_points != 0.0)
      {
        coefficients[1] = (state_b.positions[i] - state_a.positions[i]) / duration_btwn_points;
      }
      output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1];
      output.velocities[i] = t[0] * coefficients[1];
    }
  }
  else if (has_velocity && !has_accel)
  {
    double T[4];
    generate_powers(3, duration_btwn_points, T);

    for (size_t i = 0; i < dim; ++i)
    {
      double coefficients[4] = {0.0, 0.0, 0.0, 0.0};
      coefficients[0] = state_a.positions[i];
      coefficients[1] = state_a.velocities[i];
      if (duration_btwn_points != 0.0)
      {
        coefficients[2] =
          (-3.0 * state_a.positions[i] + 3.0 * state_b.positions[i] -
           2.0 * state_a.velocities[i] * T[1] - state_b.velocities[i] * T[1]) /
          T[2];
        coefficients[3] =
          (2.0 * state_a.positions[i] - 2.0 * state_b.positions[i] +
           state_a.velocities[i] * T[1] + state_b.velocities[i] * T[1]) /
          T[3];
      }

      output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1] +
                            t[2] * coefficients[2] + t[3] * coefficients[3];
      output.velocities[i] =
        t[0] * coefficients[1] + t[1] * 2.0 * coefficients[2] + t[2] * 3.0 * coefficients[3];
      output.accelerations[i] = t[0] * 2.0 * coefficients[2] + t[1] * 6.0 * coefficients[3];
    }
  }
  else if (has_velocity && has_accel)
  {
    double T[6];
    generate_powers(5, duration_btwn_points, T);

    for (size_t i = 0; i < dim; ++i)
    {
      double coefficients[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      coefficients[0] = state_a.positions[i];
      coefficients[1] = state_a.velocities[i];
      coefficients[2] = 0.5 * state_a.accelerations[i];
      if (duration_btwn_points != 0.0)
      {
        coefficients[3] =
          (-20.0 * state_a.positions[i] + 20.0 * state_b.positions[i] -
           3.0 * state_a.accelerations[i] * T[2] + state_b.accelerations[i] * T[2] -
           12.0 * state_a.velocities[i] * T[1] - 8.0 * state_b.velocities[i] * T[1]) /
          (2.0 * T[3]);
        coefficients[4] =
          (30.0 * state_a.positions[i] - 30.0 * state_b.positions[i] +
           3.0 * state_a.accelerations[i] * T[2] - 2.0 * state_b.accelerations[i] * T[2] +
           16.0 * state_a.velocities[i] * T[1] + 14.0 * state_b.velocities[i] * T[1]) /
          (2.0 * T[4]);
        coefficients[5] =
          (-12.0 * state_a.positions[i] + 12.0 * state_b.positions[i] -
           state_a.accelerations[i] * T[2] + state_b.accelerations[i] * T[2] -
           6.0 * state_a.velocities[i] * T[1] - 6.0 * state_b.velocities[i] * T[1]) /
          (2.0 * T[5]);
      }

      output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1] +
                            t[2] * coefficients[2] + t[3] * coefficients[3] +
                            t[4] * coefficients[4] + t[5] * coefficients[5];
      output.velocities[i] = t[0] * coefficients[1] + t[1] * 2.0 * coefficients[2] +
                             t[2] * 3.0 * coefficients[3] + t[3] * 4.0 * coefficients[4] +
                             t[4] * 5.0 * coefficients[5];
      output.accelerations[i] = t[0] * 2.0 * coefficients[2] + t[1] * 6.0 * coefficients[3] +
                                t[2] * 12.0 * coefficients[4] + t[3] * 20.0 * coefficients[5];
    }
  }
}

void Trajectory::deduce_from_derivatives(
  TrajectoryPoint & first_state, TrajectoryPoint & second_state, size_t dim, double delta_t)
{
  if (first_state.effort.empty())
  {
    first_state.effort.assign(dim, 0.0);
  }
  if (second_state.effort.empty())
  {
    second_state.effort.assign(dim, 0.0);
  }
  if (second_state.positions.empty())
  {
    second_state.positions.resize(dim);
    if (first_state.velocities.empty())
    {
      first_state.velocities.resize(dim, 0.0);
    }
    if (second_state.velocities.empty())
    {
      second_state.velocities.resize(dim);
      if (first_state.accelerations.empty())
      {
        first_state.accelerations.resize(dim, 0.0);
      }
      for (size_t i = 0; i < dim; ++i)
      {
        second_state.velocities[i] =
          first_state.velocities[i] +
          (first_state.accelerations[i] + second_state.accelerations[i]) * 0.5 * delta_t;
      }
    }
    for (size_t i = 0; i < dim; ++i)
    {
      second_state.positions[i] =
        first_state.positions[i] +
        (first_state.velocities[i] + second_state.velocities[i]) * 0.5 * delta_t;
    }
  }
}

TrajectoryPointConstIter Trajectory::begin() const
{
  require_trajectory(trajectory_msg_);
  return trajectory_msg_->points.begin();
}

TrajectoryPointConstIter Trajectory::end() const
{
  require_trajectory(trajectory_msg_);
  return trajectory_msg_->points.end();
}

double Trajectory::time_from_start() const { return trajectory_start_time_; }

bool Trajectory::has_trajectory_msg() const { return trajectory_msg_.get() != nullptr; }

bool Trajectory::has_nontrivial_msg() const
{
  return has_trajectory_msg() && trajectory_msg_->points.size() > 1;
}

}  // namespace joint_trajectory
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy
