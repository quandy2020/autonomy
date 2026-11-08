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

#include "navigation/common/twist2d_convert.hpp"

namespace navigation
{
namespace common
{

Twist2D FromProto(const automsgs::msgs::geometry_msgs::Twist& twist)
{
    Twist2D out;
    out.set_x(twist.linear().x());
    out.set_y(twist.linear().y());
    out.set_theta(twist.angular().z());
    return out;
}

void ToProto(const Twist2D& twist, automsgs::msgs::geometry_msgs::Twist* proto)
{
    if (proto == nullptr)
    {
        return;
    }
    auto* linear = proto->mutable_linear();
    linear->set_x(twist.x());
    linear->set_y(twist.y());
    linear->set_z(0.0);
    auto* angular = proto->mutable_angular();
    angular->set_x(0.0);
    angular->set_y(0.0);
    angular->set_z(twist.theta());
}

}  // namespace common
}  // namespace navigation
