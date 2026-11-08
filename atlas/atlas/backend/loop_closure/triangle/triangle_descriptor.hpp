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
 * @file triangle_descriptor.hpp
 * @brief BTC / STD descriptor types (Voxel-SLAM, ROS/PCL free).
 */

#ifndef ATLAS_BACKEND_LOOP_CLOSURE_TRIANGLE_TRIANGLE_DESCRIPTOR_HPP_
#define ATLAS_BACKEND_LOOP_CLOSURE_TRIANGLE_TRIANGLE_DESCRIPTOR_HPP_

#include "backend/loop_closure/loop_closure_options.hpp"
#include "backend/loop_closure/triangle/binary_descriptor.hpp"
#include "backend/loop_closure/triangle/triangle_plane.hpp"

namespace atlas {
namespace backend {
namespace loop_closure {
namespace triangle {

struct TriangleDescriptor {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d triangle = Eigen::Vector3d::Zero();
  Eigen::Vector3d angle = Eigen::Vector3d::Zero();
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  int frame_number = 0;
  BinaryDescriptor binary_a;
  BinaryDescriptor binary_b;
  BinaryDescriptor binary_c;
};

}  // namespace triangle
}  // namespace loop_closure
}  // namespace backend
}  // namespace atlas

#endif  // ATLAS_BACKEND_LOOP_CLOSURE_TRIANGLE_TRIANGLE_DESCRIPTOR_HPP_
