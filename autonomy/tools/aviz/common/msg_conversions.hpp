/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef AVIZ_COMMON__MSG_CONVERSIONS_HPP_
#define AVIZ_COMMON__MSG_CONVERSIONS_HPP_

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include "autonomy/commsgs/geometry_msgs.hpp"

namespace aviz {
namespace common {

/// Convert autonomy::commsgs::geometry_msgs::Point to Ogre::Vector3
inline Ogre::Vector3 pointMsgToOgre(const autonomy::commsgs::geometry_msgs::Point& point) {
  return Ogre::Vector3(static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z));
}

/// Convert autonomy::commsgs::geometry_msgs::Quaternion to Ogre::Quaternion
inline Ogre::Quaternion quaternionMsgToOgre(const autonomy::commsgs::geometry_msgs::Quaternion& quat) {
  return Ogre::Quaternion(static_cast<float>(quat.w), static_cast<float>(quat.x), static_cast<float>(quat.y),
                          static_cast<float>(quat.z));
}

}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__MSG_CONVERSIONS_HPP_
