// Copyright 2014 Open Source Robotics Foundation, Inc.
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

/**
 * @file utils.h
 * @brief TF2 convenience helpers: Euler YPR, yaw, and identity transforms.
 *
 * Templated over any type convertible to @c tf2::Quaternion / Transform via
 * @ref impl::toQuaternion and @c convert.
 *
 * @see impl/utils.h
 * @see convert.h
 */

#ifndef TF2_UTILS_H
#define TF2_UTILS_H

#include <autoviz/transform/tf2/LinearMath/Quaternion.h>
#include <autoviz/transform/tf2/LinearMath/Transform.h>
#include <autoviz/transform/tf2/impl/utils.h>

namespace autoviz {
namespace transform {
namespace tf2 {

/**
 * @brief Extracts yaw, pitch, roll from anything convertible to a quaternion.
 *
 * Conventions match ROS / @c Matrix3x3::getEulerYPR.
 *
 * @tparam A Rotation / quaternion-like type.
 * @param a Source rotation.
 * @param[out] yaw Yaw about Z (radians).
 * @param[out] pitch Pitch about Y (radians).
 * @param[out] roll Roll about X (radians).
 */
template <class A>
void getEulerYPR(const A& a, double& yaw, double& pitch, double& roll) {
    tf2::Quaternion q = impl::toQuaternion(a);
    impl::getEulerYPR(q, yaw, pitch, roll);
}

/**
 * @brief Returns only the yaw of anything convertible to a quaternion.
 *
 * Specialization of @ref getEulerYPR useful for planar navigation.
 *
 * @tparam A Rotation / quaternion-like type.
 * @param a Source rotation.
 * @return Yaw about Z in radians.
 */
template <class A>
double getYaw(const A& a) {
    tf2::Quaternion q = impl::toQuaternion(a);
    return impl::getYaw(q);
}

/**
 * @brief Returns an identity transform converted to type @c A.
 *
 * @tparam A Type supporting @c convert from @c tf2::Transform.
 * @return Identity transform as @c A.
 */
template <class A>
A getTransformIdentity() {
    tf2::Transform t;
    t.setIdentity();
    A a;
    convert(t, a);
    return a;
}

}  // namespace tf2
}  // namespace transform
}  // namespace autoviz

#endif  // TF2_UTILS_H
