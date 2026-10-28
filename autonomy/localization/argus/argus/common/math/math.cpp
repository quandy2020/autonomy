/*
 * Copyright 2026 The Openbot Authors
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

#include "common/math/math.hpp"

#include <algorithm>
#include <cmath>

namespace argus {
namespace common {
namespace {

constexpr double kSmallAngle = 1e-8;

Eigen::Matrix3d TranslationJacobian(const Eigen::Vector3d& omega) {
  const double theta = omega.norm();
  const Eigen::Matrix3d skew = SkewSymmetric(omega);
  if (theta < kSmallAngle) {
    return Eigen::Matrix3d::Identity() + 0.5 * skew;
  }
  const double theta2 = theta * theta;
  const double theta3 = theta2 * theta;
  return Eigen::Matrix3d::Identity() + (1.0 - std::cos(theta)) / theta2 * skew +
         (theta - std::sin(theta)) / theta3 * skew * skew;
}

Eigen::Matrix3d TranslationJacobianInverse(const Eigen::Vector3d& omega) {
  const double theta = omega.norm();
  const Eigen::Matrix3d skew = SkewSymmetric(omega);
  if (theta < kSmallAngle) {
    return Eigen::Matrix3d::Identity() - 0.5 * skew;
  }
  const double theta2 = theta * theta;
  const double coefficient =
      (1.0 / theta2) -
      (1.0 + std::cos(theta)) / (2.0 * theta * std::sin(theta));
  return Eigen::Matrix3d::Identity() - 0.5 * skew + coefficient * skew * skew;
}

}  // namespace

Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d& vector) {
  Eigen::Matrix3d skew;
  skew << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(),
      -vector.y(), vector.x(), 0.0;
  return skew;
}

Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& angle_axis) {
  const double theta = angle_axis.norm();
  if (theta < kSmallAngle) {
    return Eigen::Matrix3d::Identity();
  }
  const Eigen::Matrix3d skew = SkewSymmetric(angle_axis / theta);
  return Eigen::Matrix3d::Identity() + std::sin(theta) * skew +
         (1.0 - std::cos(theta)) * skew * skew;
}

Eigen::Vector3d LogSO3(const Eigen::Matrix3d& rotation) {
  const double cosine = std::clamp(0.5 * (rotation.trace() - 1.0), -1.0, 1.0);
  const double theta = std::acos(cosine);
  const Eigen::Vector3d vee(rotation(2, 1) - rotation(1, 2),
                            rotation(0, 2) - rotation(2, 0),
                            rotation(1, 0) - rotation(0, 1));
  if (theta < kSmallAngle) {
    return 0.5 * vee;
  }
  return (0.5 * theta / std::sin(theta)) * vee;
}

Eigen::Isometry3d ExpSE3(const Eigen::Matrix<double, 6, 1>& twist) {
  const Eigen::Vector3d omega = twist.head<3>();
  const Eigen::Vector3d rho = twist.tail<3>();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = ExpSO3(omega);
  pose.translation() = TranslationJacobian(omega) * rho;
  return pose;
}

Eigen::Matrix<double, 6, 1> LogSE3(const Eigen::Isometry3d& pose) {
  const Eigen::Vector3d omega = LogSO3(pose.rotation());
  Eigen::Matrix<double, 6, 1> twist;
  twist.head<3>() = omega;
  twist.tail<3>() = TranslationJacobianInverse(omega) * pose.translation();
  return twist;
}

}  // namespace common
}  // namespace argus
