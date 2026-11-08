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
 * @file reflex_policy_arbiter.hpp
 * @brief Continuous blend between policy intent and near-field reflex.
 */

#pragma once

#include "navigation/common/twist2d.hpp"
#include "navigation/planning/interface/reflex_interface.hpp"
#include "navigation/proto/navigation.pb.h"
#include "navigation/proto/plugins.pb.h"
#include "navigation/safety/ttc_evaluator.hpp"

namespace navigation
{
namespace safety
{

using planning::ReflexCommand;

class ReflexPolicyArbiter
{
public:
    /**
     * @brief Blend policy intent with near-field reflex.
     *
     * @param[in] desired Policy command.
     * @param[in] observation Near-field obstacles.
     * @param[in] options Reflex range/fov/priority.
     * @param[in] priority_ratio Minimum reflex weight when risk exists.
     */
    ReflexCommand Arbitrate(const common::Twist2D& desired,
                            const ReflexObservation& observation,
                            const plugins::ReactiveAvoiderOptions& options,
                            double priority_ratio) const;

private:
    TtcEvaluator ttc_;
};

}  // namespace safety
}  // namespace navigation
