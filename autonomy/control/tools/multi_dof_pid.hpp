// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

// Stand-in for ros2_controllers pid_controller algorithm core (ROS-free).

#ifndef AUTONOMY_CONTROL_TOOLS__MULTI_DOF_PID_HPP_
#define AUTONOMY_CONTROL_TOOLS__MULTI_DOF_PID_HPP_

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include "autonomy/control/tools/pid.hpp"

namespace autonomy {
namespace control {
namespace tools {

/**
 * \brief Independent per-DOF PID bank.
 *
 * Mirrors the core loop of `pid_controller::PidController` without ROS:
 * a vector of `Pid` instances, one command per error channel.
 */
class MultiDofPid
{
public:
  MultiDofPid() = default;

  explicit MultiDofPid(std::size_t dof) { resize(dof); }

  explicit MultiDofPid(std::vector<Pid> pids) : pids_(std::move(pids)) {}

  void resize(std::size_t dof)
  {
    pids_.resize(dof);
  }

  std::size_t size() const { return pids_.size(); }

  Pid & at(std::size_t i) { return pids_.at(i); }
  const Pid & at(std::size_t i) const { return pids_.at(i); }

  std::vector<Pid> & pids() { return pids_; }
  const std::vector<Pid> & pids() const { return pids_; }

  /**
   * \brief Compute one command per DOF.
   * \param errors  Error vector (size must equal `size()`)
   * \param dt      Timestep [s]
   * \return Command vector of the same length
   */
  std::vector<double> compute_command(const std::vector<double> & errors, double dt)
  {
    if (errors.size() != pids_.size())
    {
      throw std::invalid_argument(
        "MultiDofPid::compute_command: errors size does not match number of PIDs");
    }
    std::vector<double> commands(pids_.size());
    for (std::size_t i = 0; i < pids_.size(); ++i)
    {
      commands[i] = pids_[i].compute_command(errors[i], dt);
    }
    return commands;
  }

  /**
   * \brief Compute with explicit error derivatives (when available).
   */
  std::vector<double> compute_command(
    const std::vector<double> & errors, const std::vector<double> & error_dots, double dt)
  {
    if (errors.size() != pids_.size() || error_dots.size() != pids_.size())
    {
      throw std::invalid_argument(
        "MultiDofPid::compute_command: errors / error_dots size mismatch");
    }
    std::vector<double> commands(pids_.size());
    for (std::size_t i = 0; i < pids_.size(); ++i)
    {
      commands[i] = pids_[i].compute_command(errors[i], error_dots[i], dt);
    }
    return commands;
  }

  void reset()
  {
    for (auto & pid : pids_)
    {
      pid.reset();
    }
  }

private:
  std::vector<Pid> pids_;
};

}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS__MULTI_DOF_PID_HPP_
