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

#include "navigation/safety/velocity_feasibility_filter.hpp"
#include "autonomy/common/math/math.hpp"

#include <algorithm>
#include <cmath>

#include "navigation/common/platform_registry.hpp"

namespace navigation
{
namespace safety
{

bool VelocityFeasibilityFilter::Configure(const common::PlatformProfile& profile)
{
    if (!common::IsValid(profile))
    {
        configured_ = false;
        return false;
    }
    profile_ = profile;
    configured_ = true;
    Reset();
    return true;
}

bool VelocityFeasibilityFilter::Filter(const common::Twist2D& desired,
                                       const common::Twist2D& measured, double dt_s,
                                       common::Twist2D* filtered)
{
    if (!configured_ || filtered == nullptr || !std::isfinite(dt_s) || dt_s <= 0.0)
    {
        return false;
    }
    common::Twist2D limited;
    if (!common::ApplyKinematicLimits(profile_, desired, measured, dt_s, &limited))
    {
        return false;
    }

    common::Twist2D acceleration = navigation::common::MakeTwist2D((limited.x() - measured.x()) / dt_s,
                                 (limited.y() - measured.y()) / dt_s,
                                 (limited.theta() - measured.theta()) / dt_s);
    if (has_previous_acceleration_)
    {
        const double max_delta = profile_.max_jerk_mps3 * dt_s;
        acceleration.set_x(previous_acceleration_.x() +
            autonomy::common::Clamp(static_cast<double>(acceleration.x()) - previous_acceleration_.x(),
                          -max_delta, max_delta));
        acceleration.set_y(previous_acceleration_.y() +
            autonomy::common::Clamp(static_cast<double>(acceleration.y()) - previous_acceleration_.y(),
                          -max_delta, max_delta));
        acceleration.set_theta(previous_acceleration_.theta() +
            autonomy::common::Clamp(static_cast<double>(acceleration.theta()) -
                              previous_acceleration_.theta(),
                          -max_delta, max_delta));
    }

    filtered->set_x(measured.x() + acceleration.x() * dt_s);
    filtered->set_y(measured.y() + acceleration.y() * dt_s);
    filtered->set_theta(measured.theta() + acceleration.theta() * dt_s);
    filtered->set_x(autonomy::common::Clamp(static_cast<double>(filtered->x()),
                                           -profile_.max_linear_mps,
                                           profile_.max_linear_mps));
    filtered->set_y(autonomy::common::Clamp(static_cast<double>(filtered->y()),
                                           -profile_.max_lateral_mps,
                                           profile_.max_lateral_mps));
    filtered->set_theta(autonomy::common::Clamp(static_cast<double>(filtered->theta()),
                                              -profile_.max_angular_radps,
                                              profile_.max_angular_radps));

    previous_acceleration_ = acceleration;
    has_previous_acceleration_ = true;
    return true;
}

void VelocityFeasibilityFilter::Reset()
{
    previous_acceleration_ = common::Twist2D{};
    has_previous_acceleration_ = false;
}

}  // namespace safety
}  // namespace navigation
