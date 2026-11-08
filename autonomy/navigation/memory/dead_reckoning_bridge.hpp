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
 * @file dead_reckoning_bridge.hpp
 * @brief Short-gap pose extrapolation from measured velocity.
 */

#pragma once

#include "navigation/common/pose2d.hpp"
#include "navigation/common/twist2d.hpp"

namespace navigation
{
namespace memory
{

class DeadReckoningBridge
{
public:
    /** @brief Reset to a known pose. */
    void Reset(const common::Pose2D& pose);

    /**
     * @brief Integrate one velocity sample.
     *
     * @param[in] velocity Planar velocity.
     * @param[in] dt_s Integration period in seconds.
     * @return false on non-finite input or dt<=0.
     */
    bool Update(const common::Twist2D& velocity, double dt_s);

    /** @brief Latest extrapolated pose. */
    [[nodiscard]] const common::Pose2D& Pose() const;

    /** @brief True after at least one successful Update. */
    [[nodiscard]] bool Extrapolated() const;

private:
    /** @brief Integrated pose. */
    common::Pose2D pose_;
    /** @brief Whether extrapolation has run. */
    bool extrapolated_{false};
};

}  // namespace memory
}  // namespace navigation
