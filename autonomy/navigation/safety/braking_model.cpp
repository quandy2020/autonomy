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

#include "navigation/safety/braking_model.hpp"

#include <algorithm>
#include <cmath>

namespace navigation
{
namespace safety
{

BrakingModel::BrakingModel(double max_deceleration_mps2, double reaction_time_s)
    : deceleration_mps2_(std::max(0.1, max_deceleration_mps2)),
      reaction_time_s_(std::max(0.0, reaction_time_s))
{
}

double BrakingModel::StoppingDistance(double speed_mps) const
{
    const double speed = std::abs(speed_mps);
    return speed * speed / (2.0 * deceleration_mps2_);
}

double BrakingModel::MaxSafeSpeed(double distance) const
{
    if (distance <= 0.0)
    {
        return 0.0;
    }
    return std::sqrt(2.0 * deceleration_mps2_ * distance);
}

double BrakingModel::ReactionDistance(double speed_mps) const
{
    return std::abs(speed_mps) * reaction_time_s_;
}

}  // namespace safety
}  // namespace navigation
