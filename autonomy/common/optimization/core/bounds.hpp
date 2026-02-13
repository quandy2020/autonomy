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

namespace autonomy {
namespace common {
namespace optimization {

/**
 * @brief Upper and lower bound for optimization variables and constraints.
 */
struct Bounds {
    /**
     * @brief Creates a bound between @a lower and @a upper.
     */
    Bounds(double lower = 0.0, double upper = 0.0) {
        lower_ = lower;
        upper_ = upper;
    }

    double lower_;
    double upper_;

    void operator+=(double scalar) {
        lower_ += scalar;
        upper_ += scalar;
    }

    void operator-=(double scalar) {
        lower_ -= scalar;
        upper_ -= scalar;
    }
};

// settings this as signals infinity for IPOPT/SNOPT solvers
static const double inf = 1.0e20;

static const Bounds NoBound = Bounds(-inf, +inf);
static const Bounds BoundZero = Bounds(0.0, 0.0);
static const Bounds BoundGreaterZero = Bounds(0.0, +inf);
static const Bounds BoundSmallerZero = Bounds(-inf, 0.0);

}  // namespace optimization
}  // namespace common
}  // namespace autonomy