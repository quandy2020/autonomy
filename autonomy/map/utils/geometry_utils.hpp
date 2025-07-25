/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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

#pragma once 

#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"

#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace map {
namespace utils {

 /**
 * @brief Get a geometry_msgs Quaternion from a yaw angle
 * @param angle Yaw angle to generate a quaternion from
 * @return geometry_msgs Quaternion
 */
inline commsgs::geometry_msgs::Quaternion OrientationAroundZAxis(double angle)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, angle);  // void returning function
    return {q.x(), q.y(), q.z(), q.w()};
}


}  // namespace utils
}  // namespace map
}  // namespace autonomy
