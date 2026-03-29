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

#include "autonomy/common/optimization/core/constraint_set.hpp"

namespace autonomy {
namespace common {
namespace optimization {

/**
 * @brief A container holding a single cost term.
 *
 * This container builds a scalar cost term from the values of the variables.
 * This can be seen as a constraint with only one row and no bounds.
 *
 * @ingroup ProblemFormulation
 * @sa Component
 */
class CostTerm : public ConstraintSet
{
public:
    CostTerm(const std::string& name);
    virtual ~CostTerm() = default;

private:
    /**
     * @brief  Returns the scalar cost term calculated from the @c variables.
     */
    virtual double GetCost() const = 0;

public:
    /**
     * @brief  Wrapper function that converts double to Eigen::VectorXd.
     */
    VectorXd GetValues() const final;

    /**
     * @brief  Returns infinite bounds (e.g. no bounds).
     */
    VecBound GetBounds() const final;

    /**
     * Cost term printout slightly different from variables/constraints.
     */
    void Print(double tol, int& index) const final;
};

}  // namespace optimization
}  // namespace common
}  // namespace autonomy