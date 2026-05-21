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
namespace mppi {
namespace models {

struct ControlConstraints {
    float vx_max{0.5f};
    float vx_min{-0.35f};
    float vy{0.5f};
    float wz{1.9f};
    float ax_max{3.0f};
    float ax_min{-3.0f};
    float ay_max{3.0f};
    float ay_min{-3.0f};
    float az_max{3.5f};
};

struct SamplingStd {
    float vx{0.2f};
    float vy{0.2f};
    float wz{0.4f};
};

}  // namespace models
}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
