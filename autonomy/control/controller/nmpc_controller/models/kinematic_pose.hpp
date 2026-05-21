/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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
namespace control {
namespace controller {
namespace nmpc {
namespace models {

/** Planar pose [x, y, theta] (m, m, rad) — shared by all planar kinematic models. */
struct Pose2D {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

constexpr int kPoseDim = 3;
constexpr int kKinematicControlDim = 2;

}  // namespace models
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
