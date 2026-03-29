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
namespace mppi_controller {
namespace models {

/**
 * @struct mppi::models::OptimizerSettings
 * @brief Settings for the optimizer to use
 */
struct OptimizerSettings {
    ControlConstraints base_constraints{0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                        0.0f, 0.0f, 0.0f, 0.0f};
    ControlConstraints constraints{0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                   0.0f, 0.0f, 0.0f, 0.0f};
    SamplingStd sampling_std{0.0f, 0.0f, 0.0f};
    float model_dt{0.0f};
    float temperature{0.0f};
    float gamma{0.0f};
    unsigned int batch_size{0u};
    unsigned int time_steps{0u};
    unsigned int iteration_count{0u};
    bool shift_control_sequence{false};
    size_t retry_attempt_limit{0};
};

}  // namespace models
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy