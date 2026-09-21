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

#include "autonomy/localization/atla2/common/config.hpp"

#include "yaml-cpp/yaml.h"

namespace autonomy::localization::atla2 {

bool LoadConfig(const std::string& path, Atla2Config* out) {
  if (!out) {
    return false;
  }
  try {
    const YAML::Node root = YAML::LoadFile(path);
    out->config_path = path;

    if (root["frontend"]) {
      const auto& fe = root["frontend"];
      if (fe["mode"]) {
        out->mode = ParseFrontendMode(fe["mode"].as<std::string>());
      }
      if (fe["fusion"]) {
        const auto f = fe["fusion"].as<std::string>();
        out->fusion = (f == "tight") ? FusionStyle::kTight : FusionStyle::kLoose;
      }
      if (fe["vo"] && fe["vo"]["enabled"]) {
        out->vo_enabled = fe["vo"]["enabled"].as<bool>();
      }
      if (fe["vio"] && fe["vio"]["enabled"]) {
        out->vio_enabled = fe["vio"]["enabled"].as<bool>();
      }
      if (fe["lio"] && fe["lio"]["enabled"]) {
        out->lio_enabled = fe["lio"]["enabled"].as<bool>();
      }
    }

    if (root["backend"]) {
      const auto& be = root["backend"];
      if (be["type"]) {
        const auto t = be["type"].as<std::string>();
        if (t == "ceres" || t == "graph") {
          out->backend = BackendType::kCeres;
        } else {
          out->backend = BackendType::kIekf;
        }
      }
      if (be["window_size"]) {
        out->window_size = be["window_size"].as<int>();
      }
      if (be["max_iterations"]) {
        out->ceres_max_iterations = be["max_iterations"].as<int>();
      }
      if (be["pose_weight"]) {
        out->ceres_pose_weight = be["pose_weight"].as<double>();
      }
      if (be["imu_weight"]) {
        out->ceres_imu_weight = be["imu_weight"].as<double>();
      }
      if (be["visual_weight"]) {
        out->ceres_visual_weight = be["visual_weight"].as<double>();
      }
      if (be["camera"]) {
        const auto& c = be["camera"];
        if (c["fx"]) out->cam_fx = c["fx"].as<double>();
        if (c["fy"]) out->cam_fy = c["fy"].as<double>();
        if (c["cx"]) out->cam_cx = c["cx"].as<double>();
        if (c["cy"]) out->cam_cy = c["cy"].as<double>();
      }
    }

    // VIO / VO default to Ceres graph unless explicitly set to iekf.
    if (!root["backend"] || !root["backend"]["type"]) {
      if (out->mode == FrontendMode::kVio || out->mode == FrontendMode::kVo) {
        out->backend = BackendType::kCeres;
      }
    }

    if (root["sensor"] && root["sensor"]["gps"] && root["sensor"]["gps"]["enabled"]) {
      out->gps_enabled = root["sensor"]["gps"]["enabled"].as<bool>();
    }

    if (root["map"]) {
      if (root["map"]["max_local_map_points"]) {
        out->max_local_map_points = root["map"]["max_local_map_points"].as<int>();
      }
      if (root["map"]["voxel_size"]) {
        out->voxel_size = root["map"]["voxel_size"].as<double>();
      }
    }

    if (root["sync_tol_ms"]) {
      out->sync_tol_ms = root["sync_tol_ms"].as<double>();
    }

    // Derive enable flags from mode.
    if (out->mode == FrontendMode::kVo) {
      out->vo_enabled = true;
      out->vio_enabled = false;
      out->lio_enabled = false;
    } else if (out->mode == FrontendMode::kVio) {
      out->vo_enabled = false;
      out->vio_enabled = true;
      out->lio_enabled = false;
    } else if (out->mode == FrontendMode::kLo || out->mode == FrontendMode::kLio) {
      out->vo_enabled = false;
      out->vio_enabled = false;
      out->lio_enabled = true;
    } else {
      out->vo_enabled = false;
      out->vio_enabled = true;
      out->lio_enabled = true;
    }

    return true;
  } catch (const std::exception&) {
    return false;
  }
}

}  // namespace autonomy::localization::atla2
