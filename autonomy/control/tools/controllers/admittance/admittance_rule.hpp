// Copyright (c) 2022, PickNik, Inc.
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
// Pure 6DOF Cartesian admittance math (no kinematics_interface / ROS / ParamListener).

#ifndef AUTONOMY_CONTROL_TOOLS_CONTROLLERS_ADMITTANCE__ADMITTANCE_RULE_HPP_
#define AUTONOMY_CONTROL_TOOLS_CONTROLLERS_ADMITTANCE__ADMITTANCE_RULE_HPP_

#include <cmath>
#include <functional>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "autonomy/control/tools/filters.hpp"

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace admittance
{

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix3d = Eigen::Matrix3d;
using Vector3d = Eigen::Vector3d;

/**
 * \brief Optional joint-space helpers (caller-supplied kinematics).
 *
 * CartToJoint: map a 6D Cartesian delta at configuration `q` into joint deltas.
 * JointToCart: map joint deltas into a 6D Cartesian delta.
 */
using CartToJoint = std::function<bool(
  const Eigen::VectorXd & q, const Vector6d & cart, Eigen::VectorXd * joint)>;
using JointToCart = std::function<bool(
  const Eigen::VectorXd & q, const Eigen::VectorXd & joint, Vector6d * cart)>;

/**
 * \brief Transform sensor-frame wrench into base, optional gravity offset, EMA smooth.
 *
 * \param wrench_sensor   Measured wrench [fx,fy,fz,tx,ty,tz] in the sensor frame
 * \param R_base_sensor   Rotation mapping sensor vectors into the base frame
 * \param gravity_force_base  Optional gravity force to subtract in base (e.g. [0,0,-mg]);
 *                            pass zero to skip force offset
 * \param cog_base        CoG position in base used for torque offset (torque -= cog × F_g)
 * \param alpha           Exponential smoother coefficient in (0, 1]
 * \param wrench_smooth_io  In/out last smoothed wrench in base (seeded on first call)
 * \param initialized     In/out flag for first-sample seeding
 * \return Smoothed wrench expressed in the base frame
 */
inline Vector6d preprocess_wrench(
  const Vector6d & wrench_sensor,
  const Matrix3d & R_base_sensor,
  const Vector3d & gravity_force_base,
  const Vector3d & cog_base,
  double alpha,
  Vector6d & wrench_smooth_io,
  bool & initialized)
{
  Vector6d wrench_base;
  wrench_base.head<3>() = R_base_sensor * wrench_sensor.head<3>();
  wrench_base.tail<3>() = R_base_sensor * wrench_sensor.tail<3>();

  wrench_base.head<3>() -= gravity_force_base;
  wrench_base.tail<3>() -= cog_base.cross(gravity_force_base);

  if (!initialized)
  {
    wrench_smooth_io = wrench_base;
    initialized = true;
    return wrench_smooth_io;
  }
  for (Eigen::Index i = 0; i < 6; ++i)
  {
    wrench_smooth_io[i] =
      exponentialSmoothing(wrench_base[i], wrench_smooth_io[i], alpha);
  }
  return wrench_smooth_io;
}

/**
 * \brief 6DOF Cartesian admittance: X_ddot = M^{-1} (F - D X_dot - K X).
 *
 * Stiffness / damping diagonals are expressed in the control frame and rotated
 * into the base frame (same transform as upstream admittance_rule_impl).
 */
class AdmittanceRule
{
public:
  AdmittanceRule()
  {
    mass_.setOnes();
    mass_inv_.setOnes();
    damping_.setZero();
    stiffness_.setZero();
    selected_axes_.setOnes();
    X_.setZero();
    X_dot_.setZero();
    X_ddot_.setZero();
    wrench_base_.setZero();
    joint_pos_.resize(0);
    joint_vel_.resize(0);
    joint_acc_.resize(0);
  }

  /**
   * \brief Set M, K, selected axes, and either damping ratios or absolute damping.
   *
   * If `damping_is_ratio` is true (default), each channel uses
   * `D_i = 2 * ζ_i * sqrt(M_i * K_i)` like upstream.
   */
  void set_gains(
    const Vector6d & mass,
    const Vector6d & damping_ratio_or_damping,
    const Vector6d & stiffness,
    const Vector6d & selected_axes,
    bool damping_is_ratio = true)
  {
    mass_ = mass;
    stiffness_ = stiffness;
    selected_axes_ = selected_axes;
    for (Eigen::Index i = 0; i < 6; ++i)
    {
      mass_inv_[i] = (mass_[i] != 0.0) ? (1.0 / mass_[i]) : 0.0;
      if (damping_is_ratio)
      {
        damping_[i] =
          damping_ratio_or_damping[i] * 2.0 * std::sqrt(mass_[i] * stiffness_[i]);
      }
      else
      {
        damping_[i] = damping_ratio_or_damping[i];
      }
    }
  }

  void set_cart_to_joint(CartToJoint fn) { cart_to_joint_ = std::move(fn); }
  void set_joint_to_cart(JointToCart fn) { joint_to_cart_ = std::move(fn); }

  void set_joint_damping(double joint_damping) { joint_damping_ = joint_damping; }

  void reset_state()
  {
    X_.setZero();
    X_dot_.setZero();
    X_ddot_.setZero();
    wrench_base_.setZero();
    joint_pos_.setZero();
    joint_vel_.setZero();
    joint_acc_.setZero();
  }

  /**
   * \brief Pure Cartesian update (no kinematics).
   *
   * Uses the provided pose / velocity as the admittance offset state when
   * integrating; stores results in X / X_dot / X_ddot / wrench_base.
   *
   * \param wrench_base       External wrench already expressed in the base frame
   * \param X                 Current Cartesian offset [m, rad] (base)
   * \param X_dot             Current Cartesian velocity
   * \param rot_base_control  Rotation from control frame into base
   * \param dt                Timestep [s]
   */
  void update_cartesian(
    const Vector6d & wrench_base,
    const Vector6d & X,
    const Vector6d & X_dot,
    const Matrix3d & rot_base_control,
    double dt)
  {
    wrench_base_ = wrench_base;
    X_ = X;
    X_dot_ = X_dot;

    const Eigen::Matrix<double, 6, 6> K = build_cartesian_matrix(stiffness_, rot_base_control);
    const Eigen::Matrix<double, 6, 6> D = build_cartesian_matrix(damping_, rot_base_control);

    Vector6d F_base = mask_wrench(wrench_base_, rot_base_control, selected_axes_);
    X_ddot_ = mass_inv_.cwiseProduct(F_base - D * X_dot_ - K * X_);

    X_dot_ += X_ddot_ * dt;
    X_ += X_dot_ * dt;
  }

  /**
   * \brief Compute X_ddot in Cartesian, then integrate in joint space via callbacks.
   *
   * Mirrors upstream `calculate_admittance_rule` after X_ddot is formed:
   * CartToJoint(X_ddot) → joint_acc, optional joint damping, integrate vel/pos,
   * then JointToCart for admittance velocity / acceleration feedback.
   *
   * \return false if a required callback is missing or returns false
   */
  bool update_joint(
    const Vector6d & wrench_base,
    const Vector6d & X,
    const Vector6d & X_dot,
    const Matrix3d & rot_base_control,
    const Eigen::VectorXd & current_joint_pos,
    double dt)
  {
    if (!cart_to_joint_ || !joint_to_cart_)
    {
      return false;
    }

    wrench_base_ = wrench_base;
    X_ = X;
    X_dot_ = X_dot;

    const Eigen::Matrix<double, 6, 6> K = build_cartesian_matrix(stiffness_, rot_base_control);
    const Eigen::Matrix<double, 6, 6> D = build_cartesian_matrix(damping_, rot_base_control);

    Vector6d F_base = mask_wrench(wrench_base_, rot_base_control, selected_axes_);
    X_ddot_ = mass_inv_.cwiseProduct(F_base - D * X_dot_ - K * X_);

    ensure_joint_size(current_joint_pos.size());

    if (!cart_to_joint_(current_joint_pos, X_ddot_, &joint_acc_))
    {
      return false;
    }

    for (Eigen::Index i = 0; i < joint_acc_.size(); ++i)
    {
      joint_acc_[i] -= joint_damping_ * joint_vel_[i];
    }

    joint_vel_ += joint_acc_ * dt;
    joint_pos_ += joint_vel_ * dt;

    if (!joint_to_cart_(current_joint_pos, joint_vel_, &X_dot_))
    {
      return false;
    }
    if (!joint_to_cart_(current_joint_pos, joint_acc_, &X_ddot_))
    {
      return false;
    }
    return true;
  }

  const Vector6d & mass() const { return mass_; }
  const Vector6d & mass_inv() const { return mass_inv_; }
  const Vector6d & damping() const { return damping_; }
  const Vector6d & stiffness() const { return stiffness_; }
  const Vector6d & selected_axes() const { return selected_axes_; }
  const Vector6d & X() const { return X_; }
  const Vector6d & X_dot() const { return X_dot_; }
  const Vector6d & X_ddot() const { return X_ddot_; }
  const Vector6d & wrench_base() const { return wrench_base_; }
  const Eigen::VectorXd & joint_pos() const { return joint_pos_; }
  const Eigen::VectorXd & joint_vel() const { return joint_vel_; }
  const Eigen::VectorXd & joint_acc() const { return joint_acc_; }

  Vector6d & X() { return X_; }
  Vector6d & X_dot() { return X_dot_; }
  Vector6d & X_ddot() { return X_ddot_; }
  Eigen::VectorXd & joint_pos() { return joint_pos_; }
  Eigen::VectorXd & joint_vel() { return joint_vel_; }
  Eigen::VectorXd & joint_acc() { return joint_acc_; }

private:
  static Eigen::Matrix<double, 6, 6> build_cartesian_matrix(
    const Vector6d & diagonal, const Matrix3d & rot_base_control)
  {
    Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Zero();
    Matrix3d M_pos = Matrix3d::Zero();
    Matrix3d M_rot = Matrix3d::Zero();
    M_pos.diagonal() = diagonal.head<3>();
    M_rot.diagonal() = diagonal.tail<3>();
    M_pos = rot_base_control * M_pos * rot_base_control.transpose();
    M_rot = rot_base_control * M_rot * rot_base_control.transpose();
    M.block<3, 3>(0, 0) = M_pos;
    M.block<3, 3>(3, 3) = M_rot;
    return M;
  }

  static Vector6d mask_wrench(
    const Vector6d & F_base_in, const Matrix3d & rot_base_control, const Vector6d & selected_axes)
  {
    Vector6d F_base = F_base_in;
    Vector6d F_control;
    F_control.head<3>() = rot_base_control.transpose() * F_base.head<3>();
    F_control.tail<3>() = rot_base_control.transpose() * F_base.tail<3>();
    F_control = F_control.cwiseProduct(selected_axes);
    F_base.head<3>() = rot_base_control * F_control.head<3>();
    F_base.tail<3>() = rot_base_control * F_control.tail<3>();
    return F_base;
  }

  void ensure_joint_size(Eigen::Index n)
  {
    if (joint_pos_.size() != n)
    {
      joint_pos_ = Eigen::VectorXd::Zero(n);
      joint_vel_ = Eigen::VectorXd::Zero(n);
      joint_acc_ = Eigen::VectorXd::Zero(n);
    }
  }

  Vector6d mass_;
  Vector6d mass_inv_;
  Vector6d damping_;
  Vector6d stiffness_;
  Vector6d selected_axes_;
  Vector6d X_;
  Vector6d X_dot_;
  Vector6d X_ddot_;
  Vector6d wrench_base_;

  Eigen::VectorXd joint_pos_;
  Eigen::VectorXd joint_vel_;
  Eigen::VectorXd joint_acc_;
  double joint_damping_{0.0};

  CartToJoint cart_to_joint_;
  JointToCart joint_to_cart_;
};

}  // namespace admittance
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS_CONTROLLERS_ADMITTANCE__ADMITTANCE_RULE_HPP_
