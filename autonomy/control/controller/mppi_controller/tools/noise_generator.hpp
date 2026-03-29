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
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>

#include "autonomy/common/macros.hpp"
#include "autonomy/control/controller/mppi_controller/models/control_sequence.hpp"
#include "autonomy/control/controller/mppi_controller/models/optimizer_settings.hpp"
#include "autonomy/control/controller/mppi_controller/models/state.hpp"
#include "autonomy/control/proto/mppi_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {
namespace tools {

/**
 * @class mppi::NoiseGenerator
 * @brief Generates noise trajectories from optimal trajectory
 */
class NoiseGenerator
{
public:
    /**
     * @brief Constructor for mppi::NoiseGenerator
     */
    NoiseGenerator() = default;

    /**
     * @brief Initialize noise generator with settings and model types
     * @param settings Settings of controller
     * @param is_holonomic If base is holonomic
     * @param name Namespace for configs
     * @param param_handler Get parameters util
     */
    void initialize(models::OptimizerSettings& settings, bool is_holonomic,
                    const std::string& name,
                    const proto::MPPIControllerOptions* options);

    /**
     * @brief Shutdown noise generator thread
     */
    void shutdown();

    /**
     * @brief Signal to the noise thread the controller is ready to generate a
     * new noised control for the next iteration
     */
    void generateNextNoises();

    /**
     * @brief set noised control_sequence to state controls
     * @return noises vx, vy, wz
     */
    void setNoisedControls(models::State& state,
                           const models::ControlSequence& control_sequence);

    /**
     * @brief Reset noise generator with settings and model types
     * @param settings Settings of controller
     * @param is_holonomic If base is holonomic
     */
    void reset(models::OptimizerSettings& settings, bool is_holonomic);

protected:
    /**
     * @brief Thread to execute noise generation process
     */
    void noiseThread();

    /**
     * @brief Generate random controls by gaussian noise with mean in
     * control_sequence_
     *
     * @return tensor of shape [ batch_size_, time_steps_, 2]
     * where 2 stands for v, w
     */
    void generateNoisedControls();

    Eigen::ArrayXXf noises_vx_;
    Eigen::ArrayXXf noises_vy_;
    Eigen::ArrayXXf noises_wz_;

    std::default_random_engine generator_;
    std::normal_distribution<float> ndistribution_vx_;
    std::normal_distribution<float> ndistribution_wz_;
    std::normal_distribution<float> ndistribution_vy_;

    models::OptimizerSettings settings_;
    bool is_holonomic_;

    std::thread noise_thread_;
    std::condition_variable noise_cond_;
    std::mutex noise_lock_;
    bool active_{false}, ready_{false}, regenerate_noises_{false};
};

}  // namespace tools
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy