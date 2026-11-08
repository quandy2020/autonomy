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

#ifndef ATLAS_BACKEND_LOOP_CLOSURE_TRIANGLE_TRIANGLE_PLANE_HPP_
#define ATLAS_BACKEND_LOOP_CLOSURE_TRIANGLE_TRIANGLE_PLANE_HPP_

#include <Eigen/Core>

namespace atlas {
namespace backend {
namespace loop_closure {
namespace triangle {

struct TrianglePlane {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  float radius = 0.f;
  float min_eigen_value = 1.f;
  float d = 0.f;
  int id = 0;
  int points_size = 0;
  bool is_plane = false;
};

}  // namespace triangle
}  // namespace loop_closure
}  // namespace backend
}  // namespace atlas

#endif  // ATLAS_BACKEND_LOOP_CLOSURE_TRIANGLE_TRIANGLE_PLANE_HPP_
