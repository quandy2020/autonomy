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
 * @brief Internal quaternion / Euler helpers used by @ref tf2::getEulerYPR.
 *
 * Provides @c toQuaternion overloads and yaw/pitch/roll extraction from
 * @c tf2::Quaternion (and Automsgs quaternion messages).
 *
 * @see utils.h
 * @see Quaternion
 */

#ifndef TF2_IMPL_UTILS_H
#define TF2_IMPL_UTILS_H

// #include <autoviz/transform/tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <autoviz/transform/tf2/LinearMath/Quaternion.h>
#include <autoviz/transform/tf2/transform_datatypes.h>

#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/transform/geometry_msgs/pose_stamped.h"
#include "autoviz/transform/tf2/convert.h"

namespace autoviz {
namespace transform {
namespace tf2 {
namespace impl {

/**
 * @brief Identity overload: returns @p q unchanged.
 * @param q TF2 quaternion.
 * @return Copy of @p q.
 */
inline tf2::Quaternion toQuaternion(const tf2::Quaternion& q) {
    return q;
}

/**
 * @brief Converts an Automsgs quaternion message to @c tf2::Quaternion.
 * @param q Automsgs @c geometry_msgs::Quaternion.
 * @return Equivalent TF2 quaternion.
 */
inline tf2::Quaternion toQuaternion(
    const automsgs::msgs::geometry_msgs::Quaternion& q) {
    tf2::Quaternion res;
    fromMsg(q, res);
    return res;
}

/**
 * @brief Converts an Automsgs QuaternionStamped to @c tf2::Quaternion.
 * @param q Automsgs stamped quaternion (uses @c q.quaternion).
 * @return Equivalent TF2 quaternion.
 */
inline tf2::Quaternion toQuaternion(
    const automsgs::msgs::geometry_msgs::QuaternionStamped& q) {
    tf2::Quaternion res;
    fromMsg(q.quaternion, res);
    return res;
}

/**
 * @brief Converts a @ref Stamped object to a TF2 quaternion via @c toMsg.
 * @tparam T Underlying stamped payload.
 * @param t Stamped value convertible to QuaternionStamped.
 * @return Equivalent TF2 quaternion.
 */
template <typename T>
tf2::Quaternion toQuaternion(const tf2::Stamped<T>& t) {
    automsgs::msgs::geometry_msgs::QuaternionStamped q = toMsg(t);
    return toQuaternion(q);
}

/**
 * @brief Generic path: @c toMsg(@p t) as Quaternion, then to TF2.
 * @tparam T Arbitrary type with a Quaternion @c toMsg.
 * @param t Source object.
 * @return Equivalent TF2 quaternion.
 */
template <typename T>
tf2::Quaternion toQuaternion(const T& t) {
    automsgs::msgs::geometry_msgs::Quaternion q = toMsg(t);
    return toQuaternion(q);
}

/**
 * @brief Computes Euler yaw/pitch/roll from a TF2 quaternion.
 *
 * Equivalent to @c Matrix3x3(q).getEulerYPR; includes urdfdom-style
 * normalization for near-gimbal cases.
 *
 * @param q Source quaternion.
 * @param[out] yaw Yaw about Z (radians).
 * @param[out] pitch Pitch about Y (radians).
 * @param[out] roll Roll about X (radians).
 */
inline void getEulerYPR(const tf2::Quaternion& q, double& yaw, double& pitch,
                        double& roll) {
    double sqw;
    double sqx;
    double sqy;
    double sqz;

    sqx = q.x() * q.x();
    sqy = q.y() * q.y();
    sqz = q.z() * q.z();
    sqw = q.w() * q.w();

    // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
    double sarg =
        -2 * (q.x() * q.z() - q.w() * q.y()) /
        (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */
    if (sarg <= -0.99999) {
        pitch = -0.5 * M_PI;
        roll = 0;
        yaw = -2 * atan2(q.y(), q.x());
    } else if (sarg >= 0.99999) {
        pitch = 0.5 * M_PI;
        roll = 0;
        yaw = 2 * atan2(q.y(), q.x());
    } else {
        pitch = asin(sarg);
        roll =
            atan2(2 * (q.y() * q.z() + q.w() * q.x()), sqw - sqx - sqy + sqz);
        yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), sqw + sqx - sqy - sqz);
    }
}

/**
 * @brief Returns only yaw from a TF2 quaternion (navigation helper).
 * @param q Source quaternion.
 * @return Yaw about Z in radians.
 */
inline double getYaw(const tf2::Quaternion& q) {
    double yaw;

    double sqw;
    double sqx;
    double sqy;
    double sqz;

    sqx = q.x() * q.x();
    sqy = q.y() * q.y();
    sqz = q.z() * q.z();
    sqw = q.w() * q.w();

    // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
    double sarg =
        -2 * (q.x() * q.z() - q.w() * q.y()) /
        (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */

    if (sarg <= -0.99999) {
        yaw = -2 * atan2(q.y(), q.x());
    } else if (sarg >= 0.99999) {
        yaw = 2 * atan2(q.y(), q.x());
    } else {
        yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), sqw + sqx - sqy - sqz);
    }
    return yaw;
}

/**
 * @brief Yaw from an Automsgs quaternion message.
 * @param q Automsgs @c geometry_msgs::Quaternion.
 * @return Yaw about Z in radians.
 */
inline double getYaw(const automsgs::msgs::geometry_msgs::Quaternion& q) {
    tf2::Quaternion quat(q.x(), q.y(), q.z(), q.w());
    return getYaw(quat);
}

}  // namespace impl
}  // namespace tf2
}  // namespace transform
}  // namespace autoviz

#endif  // TF2_IMPL_UTILS_H
