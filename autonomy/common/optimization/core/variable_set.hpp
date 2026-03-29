/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "autonomy/common/optimization/core/composite.hpp"

namespace autonomy {
namespace common {
namespace optimization {

/**
 * @brief  A container holding a set of related optimization variables.
 *
 * This is a single set of variables representing a single concept, e.g
 * "spline coefficients" or "step durations".
 *
 * @ingroup ProblemFormulation
 * @sa Component
 */
class VariableSet : public Component
{
public:
    /**
     * @brief Creates a set of variables representing a single concept.
     * @param n_var  Number of variables.
     * @param name   What the variables represent to (e.g. "spline
     * coefficients").
     */
    VariableSet(int n_var, const std::string& name);
    virtual ~VariableSet() = default;

    // doesn't exist for variables, generated run-time error when used.
    Jacobian GetJacobian() const final {
        throw std::runtime_error("not implemented for variables");
    };
};

}  // namespace optimization
}  // namespace common
}  // namespace autonomy