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
 * @file velocity_feasibility_filter.hpp
 * @brief Acceleration and jerk limiting for the final command.
 */

#pragma once

#include "navigation/common/platform_profile.hpp"
#include "navigation/common/twist2d.hpp"

namespace navigation
{
namespace safety
{

class VelocityFeasibilityFilter
{
public:
    /** @brief Configure from platform limits. */
    bool Configure(const common::PlatformProfile& profile);

    /**
     * @brief Clamp acceleration and jerk.
     *
     * @param[in] desired Safety-approved velocity.
     * @param[in] measured Measured velocity.
     * @param[in] dt_s Control period in seconds.
     * @param[out] filtered Feasible command.
     */
    bool Filter(const common::Twist2D& desired, const common::Twist2D& measured, double dt_s,
                common::Twist2D* filtered);

    /** @brief Reset jerk history. */
    void Reset();

private:
    /** @brief Platform limits. */
    common::PlatformProfile profile_;
    /** @brief Last commanded acceleration. */
    common::Twist2D previous_acceleration_{};
    /** @brief True once an acceleration sample exists. */
    bool has_previous_acceleration_{false};
    /** @brief True after successful Configure. */
    bool configured_{false};
};

}  // namespace safety
}  // namespace navigation
