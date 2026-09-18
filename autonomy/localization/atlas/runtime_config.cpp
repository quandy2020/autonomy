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

#include "autonomy/localization/atlas/runtime_config.hpp"

#include "autonomy/common/param_handler.hpp"

#include <stdexcept>

#include "glog/logging.h"
#include "yaml-cpp/yaml.h"

namespace autonomy::localization::atlas {
namespace {

void ApplySensorNode(const YAML::Node& node, bool* enabled, std::string* topic_a,
                     std::string* topic_b = nullptr,
                     const char* key_a = "topic",
                     const char* key_b = nullptr) {
    if (!node) {
        return;
    }
    if (node["enabled"]) {
        *enabled = node["enabled"].as<bool>(*enabled);
    }
    if (topic_a && node[key_a]) {
        *topic_a = node[key_a].as<std::string>(*topic_a);
    }
    if (topic_b && key_b && node[key_b]) {
        *topic_b = node[key_b].as<std::string>(*topic_b);
    }
}

}  // namespace

RuntimeConfig MakeRuntimeConfig(common::Modality modality) {
    return MakeRuntimeConfig(modality, RuntimeConfig::SensorTopics{});
}

RuntimeConfig MakeRuntimeConfig(common::Modality modality,
                                const RuntimeConfig::SensorTopics& topics) {
    RuntimeConfig cfg;
    cfg.modality = modality;
    cfg.flags = common::FlagsFor(modality);
    cfg.topics = topics;
    cfg.residuals.vision = cfg.flags.use_vision;
    cfg.residuals.imu = cfg.flags.use_imu;
    cfg.residuals.lidar = cfg.flags.use_lidar;
    cfg.residuals.odom = cfg.flags.use_odom;
    return cfg;
}

RuntimeConfig LoadRuntimeConfig(const std::string& yaml_path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        LOG(ERROR) << "LoadRuntimeConfig: failed to load " << yaml_path << ": "
                   << e.what();
        throw;
    }

    const std::string modality_name = root["modality"].as<std::string>("vio");
    RuntimeConfig cfg = MakeRuntimeConfig(common::ParseModality(modality_name));

    const auto sensors = autonomy::common::YamlChild(root, "sensors");
    bool cam = cfg.flags.use_vision;
    bool imu = cfg.flags.use_imu;
    bool lidar = cfg.flags.use_lidar;
    bool odom = cfg.flags.use_odom;

    ApplySensorNode(sensors["imu"], &imu, &cfg.topics.imu);
    ApplySensorNode(sensors["camera"], &cam, &cfg.topics.rgb, &cfg.topics.depth,
                    "rgb_topic", "depth_topic");
    ApplySensorNode(sensors["lidar"], &lidar, &cfg.topics.lidar);
    ApplySensorNode(sensors["odom"], &odom, &cfg.topics.odom);

    cfg.flags.use_vision = cam;
    cfg.flags.use_imu = imu;
    cfg.flags.use_lidar = lidar;
    cfg.flags.use_odom = odom;
    cfg.flags.use_joint =
        (static_cast<int>(cam) + static_cast<int>(lidar) + static_cast<int>(odom) > 1)
        || (lidar && imu) || (odom && imu);

    const auto residuals = autonomy::common::YamlChild(root, "residuals");
    if (residuals) {
        cfg.residuals.vision = residuals["vision"].as<bool>(cfg.flags.use_vision);
        cfg.residuals.imu = residuals["imu"].as<bool>(cfg.flags.use_imu);
        cfg.residuals.lidar = residuals["lidar"].as<bool>(cfg.flags.use_lidar);
        cfg.residuals.odom = residuals["odom"].as<bool>(cfg.flags.use_odom);
    } else {
        cfg.residuals.vision = cfg.flags.use_vision;
        cfg.residuals.imu = cfg.flags.use_imu;
        cfg.residuals.lidar = cfg.flags.use_lidar;
        cfg.residuals.odom = cfg.flags.use_odom;
    }

    const auto system = autonomy::common::YamlChild(root, "system");
    if (system) {
        cfg.thread_pool_size = system["thread_pool_size"].as<int>(cfg.thread_pool_size);
        cfg.with_loop_closing =
            system["with_loop_closing"].as<bool>(cfg.with_loop_closing);
    }

    const auto maps = autonomy::common::YamlChild(root, "maps");
    if (maps) {
        cfg.maps_dense_rgbd = maps["dense_rgbd"].as<bool>(cfg.maps_dense_rgbd);
        cfg.maps_g2p5 = maps["g2p5"].as<bool>(cfg.maps_g2p5);
        cfg.maps_tiled = maps["tiled"].as<bool>(cfg.maps_tiled);
        if (maps["tiled_path"]) {
            cfg.tiled_map_path = maps["tiled_path"].as<std::string>("");
        } else if (maps["path"]) {
            cfg.tiled_map_path = maps["path"].as<std::string>("");
        }
    }
    cfg.enable_lidar_loc =
        root["enable_lidar_loc"].as<bool>(cfg.enable_lidar_loc);
    if (cfg.enable_lidar_loc && !cfg.maps_tiled) {
        cfg.maps_tiled = true;
    }

    // Calibration: inline `calibration:` and/or `calibration_path:`.
    if (root["calibration_path"]) {
        cfg.calibration_path = root["calibration_path"].as<std::string>("");
    }
    if (!cfg.calibration_path.empty()) {
        try {
            cfg.calibration =
                calibration::LoadCalibrationBundle(cfg.calibration_path);
        } catch (const std::exception& e) {
            LOG(WARNING) << "LoadRuntimeConfig: calibration_path='"
                         << cfg.calibration_path
                         << "' not loaded yet (" << e.what()
                         << "); caller may ResolveWorkspacePath and reload";
        }
    }
    if (root["calibration"]) {
        // Inline overrides / fills on top of file.
        auto inline_cal = calibration::LoadCalibrationBundle(root);
        if (!root["calibration_path"]) {
            cfg.calibration = std::move(inline_cal);
        } else {
            // Prefer file as base; re-merge by loading node that has only
            // calibration key — LoadCalibrationBundle already unwraps it.
            cfg.calibration = std::move(inline_cal);
        }
    }
    // Legacy extrinsics block (fusion_default style) → calib + Extrinsics.
    if (root["extrinsics"] && !root["calibration"]) {
        YAML::Node wrap;
        wrap["extrinsics"] = root["extrinsics"];
        cfg.calibration = calibration::LoadCalibrationBundle(wrap);
    }
    cfg.extrinsics = cfg.calibration.ToExtrinsics();

    LOG(INFO) << "LoadRuntimeConfig: " << yaml_path
              << " modality=" << common::ModalityName(cfg.modality)
              << " vision=" << cfg.flags.use_vision
              << " lidar=" << cfg.flags.use_lidar
              << " imu=" << cfg.flags.use_imu
              << " odom=" << cfg.flags.use_odom
              << " calib_path=" << cfg.calibration_path;
    return cfg;
}

}  // namespace autonomy::localization::atlas
