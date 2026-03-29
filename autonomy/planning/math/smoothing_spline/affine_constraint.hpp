/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file : affine_constraint.hpp
 **/

#pragma once

#include "Eigen/Core"
#include "autonomy/planning/math/polynomial_xd.hpp"

namespace autonomy {
namespace planning {
namespace math {

class AffineConstraint
{
public:
    AffineConstraint() = default;
    explicit AffineConstraint(const bool is_equality);
    AffineConstraint(const Eigen::MatrixXd& constraint_matrix,
                     const Eigen::MatrixXd& constraint_boundary,
                     const bool is_equality);

    void SetIsEquality(const double is_equality);

    const Eigen::MatrixXd& constraint_matrix() const;
    const Eigen::MatrixXd& constraint_boundary() const;
    bool AddConstraint(const Eigen::MatrixXd& constraint_matrix,
                       const Eigen::MatrixXd& constraint_boundary);

private:
    Eigen::MatrixXd constraint_matrix_;
    Eigen::MatrixXd constraint_boundary_;
    bool is_equality_ = true;
};

}  // namespace math
}  // namespace planning
}  // namespace autonomy
