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
 * @file point_cluster.hpp
 * @brief First/second-moment accumulator for voxel plane fitting.
 *
 * Ported from Voxel-SLAM `PointCluster`; the accumulator lets a voxel plane be
 * merged, split, and transformed without revisiting its points.
 */

#ifndef ATLAS_MAP_POINT_CLUSTER_HPP_
#define ATLAS_MAP_POINT_CLUSTER_HPP_

#include <Eigen/Core>

namespace atlas {
namespace map {

/** @brief First/second-moment accumulator for plane fitting. */
class PointCluster {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  int N = 0;

  void Clear() {
    P.setZero();
    v.setZero();
    N = 0;
  }
  void clear() { Clear(); }

  void Push(const Eigen::Vector3d& vec) {
    ++N;
    P += vec * vec.transpose();
    v += vec;
  }
  void push(const Eigen::Vector3d& vec) { Push(vec); }

  Eigen::Matrix3d Cov() const {
    const Eigen::Vector3d c = v / static_cast<double>(N);
    return P / static_cast<double>(N) - c * c.transpose();
  }
  Eigen::Matrix3d cov() const { return Cov(); }

  PointCluster& operator+=(const PointCluster& other) {
    P += other.P;
    v += other.v;
    N += other.N;
    return *this;
  }

  PointCluster& operator-=(const PointCluster& other) {
    P -= other.P;
    v -= other.v;
    N -= other.N;
    return *this;
  }

  void Transform(const PointCluster& source, const Eigen::Matrix3d& rotation,
                 const Eigen::Vector3d& translation) {
    N = source.N;
    v = rotation * source.v + static_cast<double>(N) * translation;
    const Eigen::Matrix3d rotation_translation_cross =
        rotation * source.v * translation.transpose();
    P = rotation * source.P * rotation.transpose() +
        rotation_translation_cross + rotation_translation_cross.transpose() +
        static_cast<double>(N) * translation * translation.transpose();
  }
  void transform(const PointCluster& source, const Eigen::Matrix3d& rotation,
                 const Eigen::Vector3d& translation) {
    Transform(source, rotation, translation);
  }
};

}  // namespace map
}  // namespace atlas

#endif  // ATLAS_MAP_POINT_CLUSTER_HPP_
