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

/**
 * @file config.cpp
 * @brief Implementations of `LoadConfig` / `SaveConfig` / `DumpConfig` (YAML ↔ AtlasConfig).
 */

#include "autonomy/localization/atlas/common/config.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "yaml-cpp/yaml.h"

namespace autonomy {
namespace localization {
namespace atlas {
namespace {

template <typename T>
bool ReadOpt(const YAML::Node& n, const char* key, T* out) {
    if (!n || !n[key]) {
        return false;
    }
    *out = n[key].as<T>();
    return true;
}

CameraModelType ParseCameraType(const std::string& s) {
    if (s == "PinHole" || s == "pinhole") {
        return CameraModelType::kPinhole;
    }
    if (s == "Rectified" || s == "rectified") {
        return CameraModelType::kRectified;
    }
    if (s == "KannalaBrandt" || s == "KannalaBrandt8" || s == "kb8" ||
        s == "fisheye") {
        return CameraModelType::kKannalaBrandt;
    }
    if (s == "RadTan" || s == "radtan") {
        return CameraModelType::kRadTan;
    }
    if (s == "FOV" || s == "fov") {
        return CameraModelType::kFov;
    }
    if (s == "UCM" || s == "ucm") {
        return CameraModelType::kUcm;
    }
    if (s == "EUCM" || s == "eucm") {
        return CameraModelType::kEucm;
    }
    if (s == "DoubleSphere" || s == "ds") {
        return CameraModelType::kDoubleSphere;
    }
    if (s == "Equirectangular" || s == "equirect") {
        return CameraModelType::kEquirectangular;
    }
    if (s == "RadialDivision" || s == "radial_division") {
        return CameraModelType::kRadialDivision;
    }
    return CameraModelType::kPinhole;
}

const char* CameraTypeName(CameraModelType t) {
    switch (t) {
        case CameraModelType::kRectified:
            return "Rectified";
        case CameraModelType::kKannalaBrandt:
            return "KannalaBrandt";
        case CameraModelType::kRadTan:
            return "RadTan";
        case CameraModelType::kFov:
            return "FOV";
        case CameraModelType::kUcm:
            return "UCM";
        case CameraModelType::kEucm:
            return "EUCM";
        case CameraModelType::kDoubleSphere:
            return "DoubleSphere";
        case CameraModelType::kEquirectangular:
            return "Equirectangular";
        case CameraModelType::kRadialDivision:
            return "RadialDivision";
        case CameraModelType::kPinhole:
        default:
            return "PinHole";
    }
}

void ReadDistortion(const YAML::Node& cam, std::vector<double>* out) {
    if (!cam || out == nullptr) {
        return;
    }
    if (cam["distortion"]) {
        *out = cam["distortion"].as<std::vector<double>>();
        return;
    }
    // OpenCV named coeffs.
    std::vector<double> d;
    for (const char* k : {"k1", "k2", "p1", "p2", "k3", "k4"}) {
        if (cam[k]) {
            d.push_back(cam[k].as<double>());
        }
    }
    if (!d.empty()) {
        *out = std::move(d);
    }
}

void LoadAtlasNested(const YAML::Node& root, AtlasConfig* out) {
    if (root["frontend"]) {
        const auto& frontend = root["frontend"];
        if (frontend["mode"]) {
            out->mode = ParseFrontendMode(frontend["mode"].as<std::string>());
        }
        if (frontend["fusion"]) {
            const auto fusion = frontend["fusion"].as<std::string>();
            out->fusion =
                (fusion == "tight") ? FusionStyle::kTight : FusionStyle::kLoose;
        }
        if (frontend["vo"] && frontend["vo"]["enabled"]) {
            out->vo_enabled = frontend["vo"]["enabled"].as<bool>();
        }
        if (frontend["vio"] && frontend["vio"]["enabled"]) {
            out->vio_enabled = frontend["vio"]["enabled"].as<bool>();
        }
        if (frontend["lio"] && frontend["lio"]["enabled"]) {
            out->lio_enabled = frontend["lio"]["enabled"].as<bool>();
        }
    }

    if (root["backend"]) {
        const auto& backend = root["backend"];
        if (backend["type"]) {
            const auto type = backend["type"].as<std::string>();
            if (type == "ceres" || type == "graph") {
                out->backend = BackendType::kCeres;
            } else {
                out->backend = BackendType::kIekf;
            }
        }
        ReadOpt(backend, "window_size", &out->window_size);
        if (backend["max_iterations"]) {
            out->ceres_max_iterations = backend["max_iterations"].as<int>();
        }
        ReadOpt(backend, "pose_weight", &out->ceres_pose_weight);
        ReadOpt(backend, "imu_weight", &out->ceres_imu_weight);
        ReadOpt(backend, "visual_weight", &out->ceres_visual_weight);
    }

    if (root["camera"]) {
        const auto& camera = root["camera"];
        if (camera["sensor"]) {
            const auto sensor = camera["sensor"].as<std::string>();
            if (sensor == "mono" || sensor == "monocular") {
                out->camera_sensor = CameraSensor::kMonocular;
            } else if (sensor == "stereo") {
                out->camera_sensor = CameraSensor::kStereo;
            } else {
                out->camera_sensor = CameraSensor::kRgbd;
            }
        }
        if (camera["type"]) {
            out->camera_type = ParseCameraType(camera["type"].as<std::string>());
        }
        ReadOpt(camera, "fx", &out->camera_fx);
        ReadOpt(camera, "fy", &out->camera_fy);
        ReadOpt(camera, "cx", &out->camera_cx);
        ReadOpt(camera, "cy", &out->camera_cy);
        ReadOpt(camera, "baseline", &out->camera_baseline_meters);
        ReadOpt(camera, "bf", &out->camera_bf);
        ReadOpt(camera, "width", &out->image_width);
        ReadOpt(camera, "height", &out->image_height);
        ReadOpt(camera, "new_width", &out->image_new_width);
        ReadOpt(camera, "new_height", &out->image_new_height);
        ReadOpt(camera, "fps", &out->fps);
        if (camera["rgb"]) {
            try {
                out->rgb = camera["rgb"].as<bool>();
            } catch (const YAML::Exception&) {
                out->rgb = camera["rgb"].as<int>() != 0;
            }
        }
        if (camera["depth_map_factor"]) {
            const double v = camera["depth_map_factor"].as<double>();
            out->depth_map_factor = (v > 1.0) ? (1.0 / v) : v;
        }
        if (camera["depth_threshold"]) {
            out->depth_threshold =
                static_cast<float>(camera["depth_threshold"].as<double>());
        }
        if (camera["th_far_points"]) {
            out->th_far_points =
                static_cast<float>(camera["th_far_points"].as<double>());
        }
        ReadDistortion(camera, &out->camera_distortion);
        if (!out->camera_distortion.empty() &&
            std::fabs(out->camera_distortion[0]) > 1e-12) {
            out->need_undistort = true;
        }
        if (camera["T_c1_c2"]) {
            out->T_c1_c2 = camera["T_c1_c2"].as<std::vector<double>>();
        }
        if (camera["overlapping_begin"]) {
            out->camera_overlapping_begin =
                camera["overlapping_begin"].as<int>();
        }
        if (camera["overlapping_end"]) {
            out->camera_overlapping_end = camera["overlapping_end"].as<int>();
        }
    }

    if (root["camera2"]) {
        const auto& c2 = root["camera2"];
        ReadOpt(c2, "fx", &out->camera2_fx);
        ReadOpt(c2, "fy", &out->camera2_fy);
        ReadOpt(c2, "cx", &out->camera2_cx);
        ReadOpt(c2, "cy", &out->camera2_cy);
        ReadDistortion(c2, &out->camera2_distortion);
        if (c2["overlapping_begin"]) {
            out->camera2_overlapping_begin =
                c2["overlapping_begin"].as<int>();
        }
        if (c2["overlapping_end"]) {
            out->camera2_overlapping_end = c2["overlapping_end"].as<int>();
        }
    }

    if (root["stereo"] && root["stereo"]["T_c1_c2"]) {
        out->T_c1_c2 =
            root["stereo"]["T_c1_c2"].as<std::vector<double>>();
    }

    if (root["imu"]) {
        const auto& imu = root["imu"];
        ReadOpt(imu, "noise_gyro", &out->imu_gyro_noise);
        ReadOpt(imu, "noise_acc", &out->imu_accel_noise);
        ReadOpt(imu, "walk_gyro", &out->imu_gyro_bias_random_walk);
        ReadOpt(imu, "walk_acc", &out->imu_accel_bias_random_walk);
        if (imu["frequency"]) {
            out->imu_frequency =
                static_cast<float>(imu["frequency"].as<double>());
        }
        if (imu["insert_kfs_when_lost"]) {
            out->insert_kfs_when_lost =
                imu["insert_kfs_when_lost"].as<bool>();
        }
        if (imu["T_b_c"]) {
            out->T_b_c = imu["T_b_c"].as<std::vector<double>>();
        }
    }

    if (root["orb"]) {
        const auto& orb = root["orb"];
        if (orb["n_features"] || orb["nFeatures"]) {
            out->orb.num_features =
                orb["n_features"] ? orb["n_features"].as<int>()
                                  : orb["nFeatures"].as<int>();
        }
        if (orb["scale_factor"] || orb["scaleFactor"]) {
            out->orb.scale_factor =
                orb["scale_factor"]
                    ? orb["scale_factor"].as<float>()
                    : orb["scaleFactor"].as<float>();
        }
        if (orb["n_levels"] || orb["nLevels"]) {
            out->orb.num_levels = orb["n_levels"] ? orb["n_levels"].as<int>()
                                                  : orb["nLevels"].as<int>();
        }
        if (orb["ini_th_fast"] || orb["iniThFAST"]) {
            out->orb.initial_fast_threshold =
                orb["ini_th_fast"] ? orb["ini_th_fast"].as<int>()
                                   : orb["iniThFAST"].as<int>();
        }
        if (orb["min_th_fast"] || orb["minThFAST"]) {
            out->orb.minimum_fast_threshold =
                orb["min_th_fast"] ? orb["min_th_fast"].as<int>()
                                   : orb["minThFAST"].as<int>();
        }
        if (orb["vocabulary"]) {
            out->vocabulary_path = orb["vocabulary"].as<std::string>();
        }
    }

    if (root["system"]) {
        const auto& sys = root["system"];
        if (sys["th_far_points"]) {
            out->th_far_points =
                static_cast<float>(sys["th_far_points"].as<double>());
        }
        ReadOpt(sys, "atlas_load", &out->atlas_load_file);
        ReadOpt(sys, "atlas_save", &out->atlas_save_file);
    }

    if (root["platform_name"]) {
        out->platform_name = root["platform_name"].as<std::string>();
    }
    if (root["vocabulary"]) {
        out->vocabulary_path = root["vocabulary"].as<std::string>();
    }

    // Derived: bf overrides baseline when set.
    if (out->camera_bf > 0.0 && out->camera_fx > 0.0) {
        out->camera_baseline_meters = out->camera_bf / out->camera_fx;
    }
}

void LoadOrbFlat(const YAML::Node& root, AtlasConfig* out) {
    // Detect ORB-SLAM3 flat schema.
    const bool looks_orb = root["Camera.type"] || root["Camera1.fx"] ||
                           root["ORBextractor.nFeatures"] ||
                           root["IMU.NoiseGyro"];
    if (!looks_orb) {
        return;
    }
    out->schema = "orb";

    if (root["Camera.type"]) {
        out->camera_type =
            ParseCameraType(root["Camera.type"].as<std::string>());
    }
    ReadOpt(root, "Camera1.fx", &out->camera_fx);
    ReadOpt(root, "Camera1.fy", &out->camera_fy);
    ReadOpt(root, "Camera1.cx", &out->camera_cx);
    ReadOpt(root, "Camera1.cy", &out->camera_cy);
    // Legacy Camera.fx
    ReadOpt(root, "Camera.fx", &out->camera_fx);
    ReadOpt(root, "Camera.fy", &out->camera_fy);
    ReadOpt(root, "Camera.cx", &out->camera_cx);
    ReadOpt(root, "Camera.cy", &out->camera_cy);

    std::vector<double> d;
    for (const char* k :
         {"Camera1.k1", "Camera1.k2", "Camera1.p1", "Camera1.p2", "Camera1.k3",
          "Camera1.k4"}) {
        if (root[k]) {
            d.push_back(root[k].as<double>());
        }
    }
    if (d.empty()) {
        for (const char* k :
             {"Camera.k1", "Camera.k2", "Camera.p1", "Camera.p2", "Camera.k3"}) {
            if (root[k]) {
                d.push_back(root[k].as<double>());
            }
        }
    }
    if (!d.empty()) {
        out->camera_distortion = std::move(d);
        out->need_undistort = true;
    }

    ReadOpt(root, "Camera.width", &out->image_width);
    ReadOpt(root, "Camera.height", &out->image_height);
    ReadOpt(root, "Camera.newWidth", &out->image_new_width);
    ReadOpt(root, "Camera.newHeight", &out->image_new_height);
    if (root["Camera.fps"]) {
        out->fps = static_cast<float>(root["Camera.fps"].as<double>());
    }
    if (root["Camera.RGB"]) {
        out->rgb = root["Camera.RGB"].as<int>() != 0;
    }
    if (root["Camera.bf"]) {
        out->camera_bf = root["Camera.bf"].as<double>();
        if (out->camera_fx > 0.0) {
            out->camera_baseline_meters = out->camera_bf / out->camera_fx;
        }
    }
    if (root["ThDepth"] || root["Camera.ThDepth"]) {
        const auto& n = root["ThDepth"] ? root["ThDepth"] : root["Camera.ThDepth"];
        out->depth_threshold = static_cast<float>(n.as<double>());
    }
    if (root["DepthMapFactor"]) {
        const double v = root["DepthMapFactor"].as<double>();
        out->depth_map_factor = (v > 1.0) ? (1.0 / v) : v;
    }
    if (root["System.thFarPoints"]) {
        out->th_far_points =
            static_cast<float>(root["System.thFarPoints"].as<double>());
    }

    ReadOpt(root, "Camera2.fx", &out->camera2_fx);
    ReadOpt(root, "Camera2.fy", &out->camera2_fy);
    ReadOpt(root, "Camera2.cx", &out->camera2_cx);
    ReadOpt(root, "Camera2.cy", &out->camera2_cy);
    {
        std::vector<double> d2;
        for (const char* k :
             {"Camera2.k1", "Camera2.k2", "Camera2.k3", "Camera2.k4",
              "Camera2.p1", "Camera2.p2"}) {
            if (root[k]) {
                d2.push_back(root[k].as<double>());
            }
        }
        if (!d2.empty()) {
            out->camera2_distortion = d2;
        }
    }
    if (root["Camera1.overlappingBegin"]) {
        out->camera_overlapping_begin =
            root["Camera1.overlappingBegin"].as<int>();
    }
    if (root["Camera1.overlappingEnd"]) {
        out->camera_overlapping_end =
            root["Camera1.overlappingEnd"].as<int>();
    }
    if (root["Camera2.overlappingBegin"]) {
        out->camera2_overlapping_begin =
            root["Camera2.overlappingBegin"].as<int>();
    }
    if (root["Camera2.overlappingEnd"]) {
        out->camera2_overlapping_end =
            root["Camera2.overlappingEnd"].as<int>();
    }
    // Stereo.T_c1_c2 — OpenCV matrix or flat 16-list
    if (root["Stereo.T_c1_c2"]) {
        const YAML::Node& tlr = root["Stereo.T_c1_c2"];
        if (tlr.IsSequence()) {
            out->T_c1_c2 = tlr.as<std::vector<double>>();
        } else if (tlr["data"]) {
            out->T_c1_c2 = tlr["data"].as<std::vector<double>>();
        }
    } else if (root["Camera1.T_c1_c2"]) {
        const YAML::Node& tlr = root["Camera1.T_c1_c2"];
        if (tlr.IsSequence()) {
            out->T_c1_c2 = tlr.as<std::vector<double>>();
        } else if (tlr["data"]) {
            out->T_c1_c2 = tlr["data"].as<std::vector<double>>();
        }
    }

    ReadOpt(root, "IMU.NoiseGyro", &out->imu_gyro_noise);
    ReadOpt(root, "IMU.NoiseAcc", &out->imu_accel_noise);
    ReadOpt(root, "IMU.GyroWalk", &out->imu_gyro_bias_random_walk);
    ReadOpt(root, "IMU.AccWalk", &out->imu_accel_bias_random_walk);
    if (root["IMU.Frequency"]) {
        out->imu_frequency =
            static_cast<float>(root["IMU.Frequency"].as<double>());
    }
    if (root["IMU.InsertKFsWhenLost"]) {
        out->insert_kfs_when_lost = root["IMU.InsertKFsWhenLost"].as<int>() != 0;
    }
    // OpenCV matrix or flat 16-list under IMU.T_b_c1
    if (root["IMU.T_b_c1"]) {
        const YAML::Node& tbc = root["IMU.T_b_c1"];
        if (tbc.IsSequence()) {
            out->T_b_c = tbc.as<std::vector<double>>();
        } else if (tbc["data"]) {
            out->T_b_c = tbc["data"].as<std::vector<double>>();
        }
    }

    if (root["ORBextractor.nFeatures"]) {
        out->orb.num_features = root["ORBextractor.nFeatures"].as<int>();
    }
    if (root["ORBextractor.scaleFactor"]) {
        out->orb.scale_factor =
            static_cast<float>(root["ORBextractor.scaleFactor"].as<double>());
    }
    if (root["ORBextractor.nLevels"]) {
        out->orb.num_levels = root["ORBextractor.nLevels"].as<int>();
    }
    if (root["ORBextractor.iniThFAST"]) {
        out->orb.initial_fast_threshold =
            root["ORBextractor.iniThFAST"].as<int>();
    }
    if (root["ORBextractor.minThFAST"]) {
        out->orb.minimum_fast_threshold =
            root["ORBextractor.minThFAST"].as<int>();
    }

    if (root["File.loadAtlasFrom"] || root["System.LoadAtlasFromFile"]) {
        const auto& n = root["File.loadAtlasFrom"]
                            ? root["File.loadAtlasFrom"]
                            : root["System.LoadAtlasFromFile"];
        out->atlas_load_file = n.as<std::string>();
    }
    if (root["File.saveAtlasTo"] || root["System.SaveAtlasToFile"]) {
        const auto& n = root["File.saveAtlasTo"] ? root["File.saveAtlasTo"]
                                                 : root["System.SaveAtlasToFile"];
        out->atlas_save_file = n.as<std::string>();
    }
}

void EmitSeq(YAML::Emitter& out, const std::vector<double>& d) {
    out << YAML::Flow << YAML::BeginSeq;
    for (double v : d) {
        out << v;
    }
    out << YAML::EndSeq;
}

}  // namespace

bool LoadConfig(const std::string& path, AtlasConfig* out) {
    if (out == nullptr) {
        return false;
    }
    try {
        const YAML::Node root = YAML::LoadFile(path);
        out->config_path = path;
        out->schema = "atlas";
        LoadAtlasNested(root, out);
        LoadOrbFlat(root, out);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool SaveConfig(const std::string& path, const AtlasConfig& cfg) {
    try {
        YAML::Emitter out;
        out.SetIndent(2);
        out << YAML::BeginMap;
        out << YAML::Key << "platform_name" << YAML::Value << cfg.platform_name;
        out << YAML::Key << "schema" << YAML::Value << cfg.schema;

        out << YAML::Key << "frontend" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "mode" << YAML::Value << ToString(cfg.mode);
        out << YAML::Key << "fusion" << YAML::Value
            << (cfg.fusion == FusionStyle::kTight ? "tight" : "loose");
        out << YAML::Key << "vo" << YAML::Value << YAML::BeginMap
            << YAML::Key << "enabled" << YAML::Value << cfg.vo_enabled
            << YAML::EndMap;
        out << YAML::Key << "vio" << YAML::Value << YAML::BeginMap
            << YAML::Key << "enabled" << YAML::Value << cfg.vio_enabled
            << YAML::EndMap;
        out << YAML::Key << "lio" << YAML::Value << YAML::BeginMap
            << YAML::Key << "enabled" << YAML::Value << cfg.lio_enabled
            << YAML::EndMap;
        out << YAML::EndMap;

        out << YAML::Key << "camera" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "sensor" << YAML::Value
            << (cfg.camera_sensor == CameraSensor::kMonocular
                    ? "mono"
                    : (cfg.camera_sensor == CameraSensor::kStereo ? "stereo"
                                                                  : "rgbd"));
        out << YAML::Key << "type" << YAML::Value
            << CameraTypeName(cfg.camera_type);
        out << YAML::Key << "fx" << YAML::Value << cfg.camera_fx;
        out << YAML::Key << "fy" << YAML::Value << cfg.camera_fy;
        out << YAML::Key << "cx" << YAML::Value << cfg.camera_cx;
        out << YAML::Key << "cy" << YAML::Value << cfg.camera_cy;
        out << YAML::Key << "baseline" << YAML::Value
            << cfg.camera_baseline_meters;
        if (cfg.camera_bf > 0.0) {
            out << YAML::Key << "bf" << YAML::Value << cfg.camera_bf;
        }
        if (cfg.image_width > 0) {
            out << YAML::Key << "width" << YAML::Value << cfg.image_width;
            out << YAML::Key << "height" << YAML::Value << cfg.image_height;
        }
        if (cfg.image_new_width > 0) {
            out << YAML::Key << "new_width" << YAML::Value
                << cfg.image_new_width;
            out << YAML::Key << "new_height" << YAML::Value
                << cfg.image_new_height;
        }
        out << YAML::Key << "fps" << YAML::Value << cfg.fps;
        out << YAML::Key << "rgb" << YAML::Value << cfg.rgb;
        out << YAML::Key << "depth_map_factor" << YAML::Value
            << cfg.depth_map_factor;
        out << YAML::Key << "depth_threshold" << YAML::Value
            << cfg.depth_threshold;
        if (cfg.th_far_points > 0.f) {
            out << YAML::Key << "th_far_points" << YAML::Value
                << cfg.th_far_points;
        }
        if (!cfg.camera_distortion.empty()) {
            out << YAML::Key << "distortion" << YAML::Value;
            EmitSeq(out, cfg.camera_distortion);
        }
        if (cfg.T_c1_c2.size() == 16) {
            out << YAML::Key << "T_c1_c2" << YAML::Value;
            EmitSeq(out, cfg.T_c1_c2);
        }
        if (cfg.camera_overlapping_begin != 0 ||
            cfg.camera_overlapping_end != 1000) {
            out << YAML::Key << "overlapping_begin" << YAML::Value
                << cfg.camera_overlapping_begin;
            out << YAML::Key << "overlapping_end" << YAML::Value
                << cfg.camera_overlapping_end;
        }
        out << YAML::EndMap;

        if (cfg.camera2_fx > 0.0) {
            out << YAML::Key << "camera2" << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "fx" << YAML::Value << cfg.camera2_fx;
            out << YAML::Key << "fy" << YAML::Value << cfg.camera2_fy;
            out << YAML::Key << "cx" << YAML::Value << cfg.camera2_cx;
            out << YAML::Key << "cy" << YAML::Value << cfg.camera2_cy;
            if (!cfg.camera2_distortion.empty()) {
                out << YAML::Key << "distortion" << YAML::Value;
                EmitSeq(out, cfg.camera2_distortion);
            }
            if (cfg.camera2_overlapping_begin != 0 ||
                cfg.camera2_overlapping_end != 1000) {
                out << YAML::Key << "overlapping_begin" << YAML::Value
                    << cfg.camera2_overlapping_begin;
                out << YAML::Key << "overlapping_end" << YAML::Value
                    << cfg.camera2_overlapping_end;
            }
            out << YAML::EndMap;
        }

        out << YAML::Key << "imu" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "noise_gyro" << YAML::Value << cfg.imu_gyro_noise;
        out << YAML::Key << "noise_acc" << YAML::Value << cfg.imu_accel_noise;
        out << YAML::Key << "walk_gyro" << YAML::Value
            << cfg.imu_gyro_bias_random_walk;
        out << YAML::Key << "walk_acc" << YAML::Value
            << cfg.imu_accel_bias_random_walk;
        out << YAML::Key << "frequency" << YAML::Value << cfg.imu_frequency;
        out << YAML::Key << "insert_kfs_when_lost" << YAML::Value
            << cfg.insert_kfs_when_lost;
        if (cfg.T_b_c.size() == 16) {
            out << YAML::Key << "T_b_c" << YAML::Value;
            EmitSeq(out, cfg.T_b_c);
        }
        out << YAML::EndMap;

        out << YAML::Key << "orb" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "n_features" << YAML::Value << cfg.orb.num_features;
        out << YAML::Key << "scale_factor" << YAML::Value
            << cfg.orb.scale_factor;
        out << YAML::Key << "n_levels" << YAML::Value << cfg.orb.num_levels;
        out << YAML::Key << "ini_th_fast" << YAML::Value
            << cfg.orb.initial_fast_threshold;
        out << YAML::Key << "min_th_fast" << YAML::Value
            << cfg.orb.minimum_fast_threshold;
        out << YAML::EndMap;

        out << YAML::Key << "vocabulary" << YAML::Value << cfg.vocabulary_path;

        out << YAML::Key << "backend" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "type" << YAML::Value
            << (cfg.backend == BackendType::kCeres ? "ceres" : "iekf");
        out << YAML::Key << "window_size" << YAML::Value << cfg.window_size;
        out << YAML::Key << "max_iterations" << YAML::Value
            << cfg.ceres_max_iterations;
        out << YAML::EndMap;

        if (!cfg.atlas_load_file.empty() || !cfg.atlas_save_file.empty()) {
            out << YAML::Key << "system" << YAML::Value << YAML::BeginMap;
            if (!cfg.atlas_load_file.empty()) {
                out << YAML::Key << "atlas_load" << YAML::Value
                    << cfg.atlas_load_file;
            }
            if (!cfg.atlas_save_file.empty()) {
                out << YAML::Key << "atlas_save" << YAML::Value
                    << cfg.atlas_save_file;
            }
            if (cfg.th_far_points > 0.f) {
                out << YAML::Key << "th_far_points" << YAML::Value
                    << cfg.th_far_points;
            }
            out << YAML::EndMap;
        }

        out << YAML::EndMap;

        std::ofstream ofs(path);
        if (!ofs) {
            return false;
        }
        ofs << out.c_str() << "\n";
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

void DumpConfig(std::ostream& os, const AtlasConfig& cfg) {
    os << "AtlasConfig [" << cfg.platform_name << "] schema=" << cfg.schema
       << "\n";
    os << "  Camera: type=" << CameraTypeName(cfg.camera_type)
       << " fx=" << cfg.camera_fx << " fy=" << cfg.camera_fy
       << " cx=" << cfg.camera_cx << " cy=" << cfg.camera_cy
       << " baseline=" << cfg.camera_baseline_meters
       << " size=" << cfg.image_width << "x" << cfg.image_height
       << " fps=" << cfg.fps << " rgb=" << cfg.rgb << "\n";
    if (!cfg.camera_distortion.empty()) {
        os << "  Distortion:";
        for (double v : cfg.camera_distortion) {
            os << " " << v;
        }
        os << "\n";
    }
    os << "  ORB: nFeatures=" << cfg.orb.num_features
       << " scale=" << cfg.orb.scale_factor << " levels=" << cfg.orb.num_levels
       << " iniFAST=" << cfg.orb.initial_fast_threshold
       << " minFAST=" << cfg.orb.minimum_fast_threshold << "\n";
    os << "  IMU: ng=" << cfg.imu_gyro_noise << " na=" << cfg.imu_accel_noise
       << " ngw=" << cfg.imu_gyro_bias_random_walk
       << " naw=" << cfg.imu_accel_bias_random_walk
       << " freq=" << cfg.imu_frequency
       << " T_b_c=" << (cfg.T_b_c.size() == 16 ? "yes" : "no") << "\n";
    os << "  RGBD: depth_map_factor=" << cfg.depth_map_factor
       << " ThDepth=" << cfg.depth_threshold
       << " thFar=" << cfg.th_far_points << "\n";
    os << "  Vocab: " << cfg.vocabulary_path << "\n";
    if (!cfg.atlas_load_file.empty() || !cfg.atlas_save_file.empty()) {
        os << "  Atlas IO: load=" << cfg.atlas_load_file
           << " save=" << cfg.atlas_save_file << "\n";
    }
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
