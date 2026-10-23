/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_VXSLAM_IMU_STATE_HPP_
#define ATLAS_VXSLAM_IMU_STATE_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>

#include "common/math/so3.hpp"

namespace atlas {
namespace vxslam {

constexpr double kGravity = 9.8;
/** @brief Alias used by Voxel-SLAM IMU EKF (m/s²). */
constexpr double kGravityMs2 = kGravity;
constexpr int kStateDim = 15;

template <int N>
using MatrixList =
    std::vector<Eigen::Matrix<double, N, N>,
                Eigen::aligned_allocator<Eigen::Matrix<double, N, N>>>;
template <int N>
using VectorList =
    std::vector<Eigen::Matrix<double, N, 1>,
                Eigen::aligned_allocator<Eigen::Matrix<double, N, 1>>>;

/** @brief 15-DoF IMU+LiDAR state: R, p, v, bg, ba, g (+ cov). */
struct ImuState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double t = 0.0;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  Eigen::Vector3d g = Eigen::Vector3d(0, 0, -kGravity);
  Eigen::Matrix<double, kStateDim, kStateDim> cov =
      Eigen::Matrix<double, kStateDim, kStateDim>::Identity();

  ImuState() { Reset(); }

  ImuState(double t_in, const Eigen::Matrix3d& R_in,
           const Eigen::Vector3d& p_in, const Eigen::Vector3d& v_in,
           const Eigen::Vector3d& bg_in, const Eigen::Vector3d& ba_in,
           const Eigen::Vector3d& g_in = Eigen::Vector3d(0, 0, -kGravity))
      : t(t_in), R(R_in), p(p_in), v(v_in), bg(bg_in), ba(ba_in), g(g_in) {}

  ImuState& operator+=(const Eigen::Matrix<double, kStateDim, 1>& dx) {
    R = R * math::Exp(Eigen::Vector3d(dx.template block<3, 1>(0, 0)));
    p += dx.template block<3, 1>(3, 0);
    v += dx.template block<3, 1>(6, 0);
    bg += dx.template block<3, 1>(9, 0);
    ba += dx.template block<3, 1>(12, 0);
    return *this;
  }

  Eigen::Matrix<double, kStateDim, 1> operator-(const ImuState& b) const {
    Eigen::Matrix<double, kStateDim, 1> a;
    a.template block<3, 1>(0, 0) =
        math::Log(Eigen::Matrix3d(b.R.transpose() * R));
    a.template block<3, 1>(3, 0) = p - b.p;
    a.template block<3, 1>(6, 0) = v - b.v;
    a.template block<3, 1>(9, 0) = bg - b.bg;
    a.template block<3, 1>(12, 0) = ba - b.ba;
    return a;
  }

  void Reset() {
    t = 0;
    R.setIdentity();
    p.setZero();
    v.setZero();
    bg.setZero();
    ba.setZero();
    cov.setIdentity();
    cov *= 0.0001;
    cov.block<6, 6>(9, 9) = Eigen::Matrix<double, 6, 6>::Identity() * 0.00001;
  }

  void SetZero() { Reset(); }
};

inline Eigen::Matrix3d Jr(Eigen::Vector3d vec) {
  const double ang = vec.norm();
  if (ang < 1e-9) {
    return Eigen::Matrix3d::Identity();
  }
  vec /= ang;
  const double ra = std::sin(ang) / ang;
  return ra * Eigen::Matrix3d::Identity() + (1.0 - ra) * vec * vec.transpose() -
         (1.0 - std::cos(ang)) / ang * math::SkewSymmetric(vec);
}

inline Eigen::Matrix3d JrInv(const Eigen::Matrix3d& rot) {
  const Eigen::AngleAxisd aa(rot);
  const Eigen::Vector3d axi = aa.axis();
  const double ang = aa.angle();
  if (ang < 1e-9) {
    return Eigen::Matrix3d::Identity();
  }
  const double ctt = ang / 2.0 / std::tan(ang / 2.0);
  return ctt * Eigen::Matrix3d::Identity() +
         (1.0 - ctt) * axi * axi.transpose() +
         ang / 2.0 * math::SkewSymmetric(axi);
}

}  // namespace vxslam
}  // namespace atlas

#endif
