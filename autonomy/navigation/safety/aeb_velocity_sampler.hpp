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
 * @file aeb_velocity_sampler.hpp
 * @brief Fallback sampling when the CBF projection is infeasible.
 */

#pragma once

#include <optional>

#include "navigation/common/platform_profile.hpp"
#include "navigation/common/twist2d.hpp"
#include "navigation/proto/navigation.pb.h"
#include "navigation/safety/cbf_filter.hpp"

namespace navigation
{
namespace safety
{

class AebVelocitySampler
{
public:
    /**
     * @brief Pick the largest forward speed that remains CBF-safe.
     *
     * @param[in] desired Preferred velocity.
     * @param[in] observation Near-field obstacles.
     * @param[in] profile Platform limits.
     * @param[in] filter Barrier checker.
     * @param[in] sample_count Number of speed samples (>=1).
     * @return Safe velocity, or nullopt when even zero is unsafe.
     */
    [[nodiscard]] std::optional<common::Twist2D> Sample(
        const common::Twist2D& desired, const ReflexObservation& observation,
        const common::PlatformProfile& profile, const CbfFilter& filter,
        std::size_t sample_count) const;
};

}  // namespace safety
}  // namespace navigation
