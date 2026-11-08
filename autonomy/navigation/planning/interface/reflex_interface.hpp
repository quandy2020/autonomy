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
 * @file reflex_interface.hpp
 * @brief Near-field reflex boundary used by the safety chain.
 */

#pragma once

#include "navigation/common/twist2d.hpp"
#include "navigation/proto/navigation.pb.h"

namespace navigation
{
namespace planning
{

struct ReflexCommand
{
    common::Twist2D velocity;
    bool intervened{false};
    double risk{0.0};
};

/**
 * @brief Near-field reactive policy.
 *
 * @par Threading
 * Called from the safety process only; implementations must be stateless or
 * internally synchronized.
 */
class ReflexInterface
{
public:
    virtual ~ReflexInterface() = default;

    virtual ReflexCommand Compute(const ReflexObservation& observation,
                                  const common::Twist2D& desired, double dt_s) = 0;

    virtual void Reset() = 0;
};

}  // namespace planning
}  // namespace navigation
