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
 * @file braking_model.hpp
 * @brief Conservative stopping-distance model used by AEB.
 */

#pragma once

namespace navigation
{
namespace safety
{

class BrakingModel
{
public:
    /**
     * @brief Construct with braking assumptions.
     *
     * @param[in] max_deceleration_mps2 Guaranteed deceleration.
     * @param[in] reaction_time_s Actuation delay.
     */
    BrakingModel(double max_deceleration_mps2, double reaction_time_s);

    /** @brief Distance to stop from a speed, in metres. */
    [[nodiscard]] double StoppingDistance(double speed_mps) const;

    /** @brief Maximum speed for a remaining distance, in m/s. */
    [[nodiscard]] double MaxSafeSpeed(double distance) const;

    /** @brief Distance travelled during the reaction time, in metres. */
    [[nodiscard]] double ReactionDistance(double speed_mps) const;

private:
    /** @brief Guaranteed deceleration in m/s^2. */
    double deceleration_mps2_{1.0};
    /** @brief Actuation delay in seconds. */
    double reaction_time_s_{0.15};
};

}  // namespace safety
}  // namespace navigation
