/*
 * Copyright 2026 The Openbot Authors
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

#include "autonomy/localization/atlas/util/extrinsics.hpp"
#include "autonomy/localization/atlas/util/modality.hpp"

#include <string>

namespace autonomy::localization::atlas {

//! Single-system runtime: one YAML drives sensor enablement + residual mask.
struct RuntimeConfig {
    common::Modality modality = common::Modality::kVio;
    common::ModalityFlags flags{};
    common::Extrinsics extrinsics{};

    struct SensorTopics {
        std::string imu = "/imu";
        std::string rgb = "/camera/rgb/image_raw";
        std::string depth = "/camera/depth/image_raw";
        std::string lidar = "/points";
        std::string odom = "/wheel_odom";
    } topics;

    struct ResidualEnable {
        bool vision = false;
        bool imu = false;
        bool lidar = false;
        bool odom = false;
    } residuals;

    int thread_pool_size = 4;
    bool with_loop_closing = true;
    bool maps_dense_rgbd = true;
    bool maps_g2p5 = false;

    std::string vision_config_path;
    std::string vocab_path;
    std::string lidar_config_path;
};

//! Build from modality (+ optional topic overrides). YAML file load comes next.
RuntimeConfig MakeRuntimeConfig(common::Modality modality);
RuntimeConfig MakeRuntimeConfig(common::Modality modality,
                                const RuntimeConfig::SensorTopics& topics);

//! Load conf/atlas/profiles/*.yaml style runtime config.
RuntimeConfig LoadRuntimeConfig(const std::string& yaml_path);

}  // namespace autonomy::localization::atlas
