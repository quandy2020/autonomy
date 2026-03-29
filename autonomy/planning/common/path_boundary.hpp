/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file path_boundary.hpp
 **/

#pragma once

#include <cstddef>
#include <vector>

namespace autonomy {
namespace planning {

// Obstacle corner constraint for piecewise jerk path problem
struct ObsCornerConstraint {
    int left_index;
    int right_index;
    double left_weight;
    double right_weight;
    double lower_bound;
    double upper_bound;
};

using ObsCornerConstraints = std::vector<ObsCornerConstraint>;

// ADC vertex constraint for piecewise jerk path problem
struct ADCVertexConstraint {
    int left_index;
    int right_index;
    double left_weight;
    double right_weight;
    double lower_bound;
    double upper_bound;
};

struct ADCVertexConstraints {
    std::vector<ADCVertexConstraint> constraints;
    double front_edge_to_center = 0.0;

    size_t size() const {
        return constraints.size();
    }
    const ADCVertexConstraint& operator[](size_t i) const {
        return constraints[i];
    }
    ADCVertexConstraint& operator[](size_t i) {
        return constraints[i];
    }
    void push_back(const ADCVertexConstraint& constraint) {
        constraints.push_back(constraint);
    }
};

}  // namespace planning
}  // namespace autonomy
