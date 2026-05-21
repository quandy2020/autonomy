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

#include <condition_variable>
#include <mutex>
#include <random>
#include <thread>

#include "autonomy/control/controller/mppi_controller/models/control_sequence.hpp"
#include "autonomy/control/controller/mppi_controller/models/optimizer_settings.hpp"
#include "autonomy/control/controller/mppi_controller/models/state.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {

class NoiseGenerator
{
public:
    void initialize(models::OptimizerSettings& settings, bool is_holonomic);
    void shutdown();
    void generateNextNoises();
    void setNoisedControls(models::State& state,
                           const models::ControlSequence& control_sequence);
    void reset(models::OptimizerSettings& settings, bool is_holonomic);

private:
    void noiseThread();
    void generateNoisedControls();

    models::OptimizerSettings settings_;
    bool is_holonomic_{false};
    bool active_{false};
    bool ready_{false};
    bool regenerate_noises_{false};

    Eigen::ArrayXXf noises_vx_;
    Eigen::ArrayXXf noises_vy_;
    Eigen::ArrayXXf noises_wz_;

    std::default_random_engine generator_;
    std::normal_distribution<float> ndistribution_vx_;
    std::normal_distribution<float> ndistribution_vy_;
    std::normal_distribution<float> ndistribution_wz_;

    std::thread noise_thread_;
    std::mutex noise_lock_;
    std::condition_variable noise_cond_;
};

}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
