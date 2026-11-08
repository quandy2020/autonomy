/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file twist2d_convert.hpp
 * @brief Boundary conversion between automsgs twists and Twist2D.
 */

#pragma once

#include "automsgs/msgs/geometry_msgs/twist.pb.h"
#include "navigation/common/twist2d.hpp"

namespace navigation
{
namespace common
{

/** @brief Extract planar velocity from a 3D twist. */
Twist2D FromProto(const automsgs::msgs::geometry_msgs::Twist& twist);

/** @brief Fill a 3D twist from a planar velocity; z components are zero. */
void ToProto(const Twist2D& twist, automsgs::msgs::geometry_msgs::Twist* proto);

}  // namespace common
}  // namespace navigation
