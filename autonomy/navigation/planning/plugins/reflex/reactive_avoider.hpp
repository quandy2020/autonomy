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
 * @file reactive_avoider.hpp
 * @brief Near-field reactive avoidance primitive (planning plugin).
 */

#pragma once

#include "navigation/planning/interface/reflex_interface.hpp"
#include "navigation/proto/plugins.pb.h"

namespace navigation
{
namespace planning
{

class ReactiveAvoider final : public ReflexInterface
{
public:
    /** @brief Construct with near-field avoidance options. */
    explicit ReactiveAvoider(plugins::ReactiveAvoiderOptions options);

    ReflexCommand Compute(const ReflexObservation& observation,
                          const common::Twist2D& desired, double dt_s) override;

    /** @brief Reset cached state (currently stateless). */
    void Reset() override;

private:
    plugins::ReactiveAvoiderOptions options_;
};

}  // namespace planning
}  // namespace navigation
