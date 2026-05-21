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

#include <Eigen/Dense>

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {
namespace models {

struct Control {
    float vx{0.0f};
    float vy{0.0f};
    float wz{0.0f};
};

struct ControlSequence {
    Eigen::ArrayXf vx;
    Eigen::ArrayXf vy;
    Eigen::ArrayXf wz;

    void reset(unsigned int time_steps) {
        vx.setZero(time_steps);
        vy.setZero(time_steps);
        wz.setZero(time_steps);
    }
};

}  // namespace models
}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
