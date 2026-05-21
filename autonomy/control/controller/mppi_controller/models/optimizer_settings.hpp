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

#include <cstddef>

#include "autonomy/control/controller/mppi_controller/models/constraints.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {
namespace models {

struct OptimizerSettings {
    ControlConstraints base_constraints;
    ControlConstraints constraints;
    SamplingStd sampling_std;
    float model_dt{0.05f};
    float temperature{0.3f};
    float gamma{0.015f};
    unsigned int batch_size{1000u};
    unsigned int time_steps{56u};
    unsigned int iteration_count{1u};
    bool shift_control_sequence{false};
    size_t retry_attempt_limit{1u};
};

}  // namespace models
}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
