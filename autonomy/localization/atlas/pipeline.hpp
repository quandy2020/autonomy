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

#include "autonomy/localization/atlas/util/modality.hpp"
#include "autonomy/localization/atlas/frontend/local_estimator.hpp"
#include "autonomy/localization/atlas/mapping/map_incremental.hpp"
#include "autonomy/localization/atlas/runtime_config.hpp"
#include "autonomy/localization/atlas/sensor/sensor_suite.hpp"

#include <memory>
#include <string>

namespace autonomy::localization::atlas {

class system;

/**
 * Assembles SensorSuite (+ optional system attach) for the single AtlasSystem.
 * No PoseFusion / second lidar SLAM. LocalizationServer wires this type.
 */
class Pipeline {
public:
    struct Options {
        common::Modality modality = common::Modality::kVio;
        std::string atlas_config_path;
        std::string atlas_vocab_path;
        std::string rgb_topic = "/camera/rgb/image_raw";
        std::string depth_topic = "/camera/depth/image_raw";
        std::string seg_topic;
        std::string imu_topic;
        std::string lidar_config_path;
        std::string lidar_topic = "/points";
        std::string lidar_imu_topic = "/imu";
        bool enable_lightning_upstream = false;
        std::string wheel_topic = "/wheel_odom";
    };

    explicit Pipeline(Options options);
    ~Pipeline();

    Pipeline(const Pipeline&) = delete;
    Pipeline& operator=(const Pipeline&) = delete;

    //! Attach the single AtlasSystem (alias name preferred over AttachVisionSystem).
    void AttachSystem(system* slam);
    void AttachVisionSystem(system* slam) { AttachSystem(slam); }

    //! Ensure a LocalEstimator exists (LO/LIO without vision YAML).
    frontend::LocalEstimator* EnsureLocalEstimator();

    bool Start();
    void Shutdown();

    [[nodiscard]] common::Modality modality() const { return runtime_.modality; }
    [[nodiscard]] common::ModalityFlags flags() const { return runtime_.flags; }
    [[nodiscard]] const RuntimeConfig& runtime() const { return runtime_; }

    //! Merge profile RuntimeConfig (calibration / flags / topics) before Start.
    void SetRuntimeConfig(RuntimeConfig cfg);

    system* vision_system() { return vision_; }
    sensor::SensorSuite* sensors() { return sensors_.get(); }
    frontend::LocalEstimator* local_estimator() { return local_estimator_.get(); }
    mapping::MapIncremental* map_incremental() {
        return map_incremental_.get();
    }
    //! Active live-map owner (LocalMapping preferred, else Pipeline fallback).
    mapping::MapIncremental* active_map_incremental();

private:
    void WireLidarIVox();

    Options options_;
    RuntimeConfig runtime_;
    system* vision_ = nullptr;
    std::unique_ptr<sensor::SensorSuite> sensors_;
    std::unique_ptr<frontend::LocalEstimator> local_estimator_;
    //! Fallback live IVox owner when no LocalMapping (LO/LIO without vision).
    std::unique_ptr<mapping::MapIncremental> map_incremental_;
    bool running_ = false;
};

}  // namespace autonomy::localization::atlas
