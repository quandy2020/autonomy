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

/**
 * @file se3.hpp
 * @brief SE(3) facade over `transform::Rigid3d`.
 *
 * Method names match FAST-LIVO2 `Sophus::SE3` call sites
 * (`rotation_matrix`, `translation`, `inverse`). Storage is
 * `transform::Rigid3d`: \(p_w = R p_b + t\).
 */

#ifndef ATLAS_COMMON_MATH_SE3_HPP_
#define ATLAS_COMMON_MATH_SE3_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common/transform/rigid_transform.hpp"
#include "common/transform/transform.hpp"

namespace atlas {
namespace math {

class SE3 {
 public:
  SE3() = default;

  /** @param R Rotation, world ← body. @param t Translation. */
  SE3(const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
      : T_(t, Eigen::Quaterniond(R)) {}

  explicit SE3(const transform::Rigid3d& T) : T_(T) {}

  explicit SE3(const Eigen::Isometry3d& T) : T_(transform::ToRigid3(T)) {}

  Eigen::Matrix3d rotation_matrix() const {
    return T_.rotation().toRotationMatrix();
  }
  Eigen::Matrix3d rotationMatrix() const { return rotation_matrix(); }
  Eigen::Vector3d translation() const { return T_.translation(); }

  const transform::Rigid3d& rigid() const { return T_; }

  SE3 inverse() const { return SE3(T_.inverse()); }

  Eigen::Vector3d operator*(const Eigen::Vector3d& p) const { return T_ * p; }

  SE3 operator*(const SE3& other) const { return SE3(T_ * other.T_); }

  Eigen::Isometry3d toIsometry() const { return transform::ToIsometry(T_); }

 private:
  transform::Rigid3d T_ = transform::Rigid3d::Identity();
};

inline transform::Rigid3d ToRigid3(const SE3& T) { return T.rigid(); }

inline SE3 ToSE3(const transform::Rigid3d& rigid) { return SE3(rigid); }

}  // namespace math

namespace transform {

inline Rigid3d ToRigid3(const math::SE3& T) { return math::ToRigid3(T); }

inline math::SE3 ToSE3(const Rigid3d& rigid) { return math::ToSE3(rigid); }

}  // namespace transform
}  // namespace atlas

#endif  // ATLAS_COMMON_MATH_SE3_HPP_
