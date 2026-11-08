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

#ifndef ATLAS_BACKEND_LOOP_CLOSURE_TRIANGLE_BINARY_DESCRIPTOR_HPP_
#define ATLAS_BACKEND_LOOP_CLOSURE_TRIANGLE_BINARY_DESCRIPTOR_HPP_

#include <Eigen/Core>
#include <vector>

namespace atlas {
namespace backend {
namespace loop_closure {
namespace triangle {

struct BinaryDescriptor {
  std::vector<bool> occupy_array;
  unsigned char summary = 0;
  Eigen::Vector3d location = Eigen::Vector3d::Zero();
};

}  // namespace triangle
}  // namespace loop_closure
}  // namespace backend
}  // namespace atlas

#endif  // ATLAS_BACKEND_LOOP_CLOSURE_TRIANGLE_BINARY_DESCRIPTOR_HPP_
