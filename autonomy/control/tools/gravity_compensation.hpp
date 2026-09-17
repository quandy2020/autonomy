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

// Ported into autonomy::control::tools (pure Eigen math; no TF / ROS / FilterBase)

#ifndef AUTONOMY_CONTROL_TOOLS__GRAVITY_COMPENSATION_HPP_
#define AUTONOMY_CONTROL_TOOLS__GRAVITY_COMPENSATION_HPP_

#include <Eigen/Dense>

namespace autonomy {
namespace control {
namespace tools {

using Wrench6d = Eigen::Matrix<double, 6, 1>;
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

/**
 * \brief Subtract gravity force / CoG torque from a sensor-frame wrench.
 *
 * Port of the core math from `control_filters::GravityCompensation` without TF:
 * - Gravity force in world: `g * mass`
 * - Rotate into sensor: `R_sensor_from_world * (g * mass)`
 * - `force  -= R * (g * mass)`
 * - `torque -= cog.cross(R * (g * mass))`
 *
 * \param wrench               Measured wrench in sensor frame [fx,fy,fz,tx,ty,tz]
 * \param R_sensor_from_world  Rotation that maps world vectors into the sensor frame
 * \param cog_sensor           Center of gravity expressed in the sensor frame [m]
 * \param mass                 Tool / end-effector mass [kg]
 * \param gravity_world        Gravity acceleration vector in the world frame [m/s^2]
 * \return Compensated wrench in the sensor frame
 */
inline Wrench6d compensate_gravity(
  const Wrench6d & wrench,
  const Matrix3d & R_sensor_from_world,
  const Vector3d & cog_sensor,
  double mass,
  const Vector3d & gravity_world)
{
  const Vector3d gravity_force_sensor = R_sensor_from_world * (gravity_world * mass);
  const Vector3d gravity_torque_sensor = cog_sensor.cross(gravity_force_sensor);

  Wrench6d out = wrench;
  out.head<3>() -= gravity_force_sensor;
  out.tail<3>() -= gravity_torque_sensor;
  return out;
}

/**
 * \brief Stateful helper holding CoG / mass / gravity; caller supplies R each step.
 */
class GravityCompensation
{
public:
  GravityCompensation() = default;

  GravityCompensation(
    const Vector3d & cog_sensor, double mass, const Vector3d & gravity_world)
  : cog_sensor_(cog_sensor), mass_(mass), gravity_world_(gravity_world)
  {
  }

  void set_cog(const Vector3d & cog_sensor) { cog_sensor_ = cog_sensor; }
  void set_mass(double mass) { mass_ = mass; }
  void set_gravity(const Vector3d & gravity_world) { gravity_world_ = gravity_world; }

  const Vector3d & cog() const { return cog_sensor_; }
  double mass() const { return mass_; }
  const Vector3d & gravity() const { return gravity_world_; }

  Wrench6d update(const Wrench6d & wrench, const Matrix3d & R_sensor_from_world) const
  {
    return compensate_gravity(
      wrench, R_sensor_from_world, cog_sensor_, mass_, gravity_world_);
  }

private:
  Vector3d cog_sensor_{Vector3d::Zero()};
  double mass_{0.0};
  Vector3d gravity_world_{0.0, 0.0, -9.81};
};

}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS__GRAVITY_COMPENSATION_HPP_
