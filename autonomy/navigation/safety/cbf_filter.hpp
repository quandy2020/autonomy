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
 * @file cbf_filter.hpp
 * @brief Control-barrier-function velocity projection.
 */

#pragma once

#include <array>
#include <optional>

#include "navigation/common/platform_profile.hpp"
#include "navigation/common/twist2d.hpp"
#include "navigation/proto/navigation.pb.h"

namespace navigation
{
namespace safety
{

/**
 * @brief Projects a desired velocity onto the intersection of barrier
 * half-planes and the platform velocity box.
 *
 * @par Threading
 * Stateless; safe to reuse across cycles on one thread.
 */
class CbfFilter
{
public:
    /**
     * @brief Configure barrier parameters.
     *
     * @param[in] profile Platform velocity box and footprint.
     * @param[in] alpha Barrier decay rate in 1/s (> 0).
     * @param[in] margin Additional clearance in metres.
     */
    bool Configure(const common::PlatformProfile& profile, double alpha, double margin);

    /**
     * @brief Project a desired velocity onto the safe set.
     *
     * @param[in] desired Candidate velocity.
     * @param[in] observation Near-field obstacles.
     * @return Feasible velocity, or nullopt when infeasible.
     */
    [[nodiscard]] std::optional<common::Twist2D> Project(
        const common::Twist2D& desired, const ReflexObservation& observation) const;

    /**
     * @brief Check every barrier half-plane.
     *
     * @param[in] command Candidate velocity.
     * @param[in] observation Near-field obstacles.
     */
    [[nodiscard]] bool IsSafe(const common::Twist2D& command,
                              const ReflexObservation& observation) const;

private:
    /** @brief Maximum obstacle constraints per cycle. */
    static constexpr std::size_t kMaxObstacles = 32;

    /** @brief Half-plane n·v <= bound. */
    struct Constraint
    {
        /** @brief Unit normal x. */
        double normal_x{0.0};
        /** @brief Unit normal y. */
        double normal_y{0.0};
        /** @brief Right-hand side. */
        double bound{0.0};
    };

    /** @brief Build half-planes from the obstacle set. */
    std::size_t BuildConstraints(const ReflexObservation& observation,
                                 std::array<Constraint, kMaxObstacles>* constraints) const;

    /** @brief Platform limits. */
    common::PlatformProfile profile_;
    /** @brief Barrier decay rate in 1/s. */
    double alpha_{2.0};
    /** @brief Additional clearance in metres. */
    double margin_{0.1};
    /** @brief True after a successful Configure. */
    bool configured_{false};
};

}  // namespace safety
}  // namespace navigation
