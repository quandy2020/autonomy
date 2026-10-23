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

#include "argus/common/camera/camera_loader.hpp"

#include <filesystem>
#include <iostream>
#include <utility>

#include <opencv2/core.hpp>

#include "argus/common/camera/atan_camera.hpp"
#include "argus/common/camera/equidistant_camera.hpp"
#include "argus/common/camera/omni_camera.hpp"
#include "argus/common/camera/pinhole_camera.hpp"
#include "argus/common/camera/polynomial_camera.hpp"

namespace argus {
namespace camera {
namespace {

namespace fs = std::filesystem;

template <typename T>
T YamlGet(const cv::FileNode& node, const std::string& key, const T& def) {
    const cv::FileNode child = node[key];
    if (child.empty()) {
        return def;
    }
    T value = def;
    child >> value;
    return value;
}

std::string NormalizeModel(std::string m) {
    if (m == "Equidistant") {
        return "EquidistantCamera";
    }
    if (m == "ATAN" || m == "FOV" || m == "ATANCamera") {
        return "ATANCamera";
    }
    if (m == "Polynomial") {
        return "PolynomialCamera";
    }
    if (m == "Omni") {
        return "OmniCamera";
    }
    return m;
}

std::string JoinDir(const std::string& yaml_path, const std::string& rel) {
    if (rel.empty()) {
        return {};
    }
    const fs::path p(rel);
    if (p.is_absolute()) {
        return p.string();
    }
    return (fs::path(yaml_path).parent_path() / p).string();
}

}  // namespace

std::unique_ptr<AbstractCamera> Create(const CameraIntrinsics& i) {
    const std::string model = NormalizeModel(i.model);
    if (model == "Pinhole") {
        return std::make_unique<PinholeCamera>(i.width, i.height, i.scale, i.fx,
                                               i.fy, i.cx, i.cy, i.d0, i.d1,
                                               i.d2, i.d3, i.d4);
    }
    if (model == "EquidistantCamera") {
        return std::make_unique<EquidistantCamera>(i.width, i.height, i.scale,
                                                   i.fx, i.fy, i.cx, i.cy, i.k1,
                                                   i.k2, i.k3, i.k4);
    }
    if (model == "ATANCamera") {
        const double s = (i.s != 0.0) ? i.s : i.k1;
        if (i.atan_normalized) {
            return std::make_unique<ATANCamera>(ATANCamera::FromNormalized(
                static_cast<double>(i.width), static_cast<double>(i.height),
                i.fx, i.fy, i.cx, i.cy, s));
        }
        return std::make_unique<ATANCamera>(i.width, i.height, i.scale, i.fx,
                                            i.fy, i.cx, i.cy, s);
    }
    if (model == "PolynomialCamera") {
        return std::make_unique<PolynomialCamera>(
            i.width, i.height, i.scale, i.fx, i.fy, i.cx, i.cy, i.skew, i.k2,
            i.k3, i.k4, i.k5, i.k6, i.k7);
    }
    if (model == "OmniCamera") {
        if (i.ocam_calib_file.empty()) {
            std::cerr
                << "[argus::camera] OmniCamera requires ocam_calib_file\n";
            return nullptr;
        }
        auto cam = std::make_unique<OmniCamera>(i.ocam_calib_file);
        if (!cam->valid()) {
            return nullptr;
        }
        return cam;
    }
    std::cerr << "[argus::camera] Unsupported cam_model: " << i.model
              << std::endl;
    return nullptr;
}

std::unique_ptr<AbstractCamera> LoadFromYaml(const std::string& yaml_path) {
    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[argus::camera] Cannot open YAML: " << yaml_path
                  << std::endl;
        return nullptr;
    }
    const cv::FileNode root = fs.root();
    CameraIntrinsics i;
    i.model = YamlGet<std::string>(root, "cam_model", "");
    i.width = YamlGet<int>(root, "cam_width", 0);
    i.height = YamlGet<int>(root, "cam_height", 0);
    i.scale = YamlGet<double>(root, "scale", 1.0);
    i.fx = YamlGet<double>(root, "cam_fx", 0.0);
    i.fy = YamlGet<double>(root, "cam_fy", 0.0);
    i.cx = YamlGet<double>(root, "cam_cx", 0.0);
    i.cy = YamlGet<double>(root, "cam_cy", 0.0);
    i.d0 = YamlGet<double>(root, "cam_d0", 0.0);
    i.d1 = YamlGet<double>(root, "cam_d1", 0.0);
    i.d2 = YamlGet<double>(root, "cam_d2", 0.0);
    i.d3 = YamlGet<double>(root, "cam_d3", 0.0);
    i.d4 = YamlGet<double>(root, "cam_d4", 0.0);
    i.k1 = YamlGet<double>(root, "k1", 0.0);
    i.k2 = YamlGet<double>(root, "k2", 0.0);
    i.k3 = YamlGet<double>(root, "k3", 0.0);
    i.k4 = YamlGet<double>(root, "k4", 0.0);
    i.k5 = YamlGet<double>(root, "k5", 0.0);
    i.k6 = YamlGet<double>(root, "k6", 0.0);
    i.k7 = YamlGet<double>(root, "k7", 0.0);
    i.s = YamlGet<double>(root, "cam_s", YamlGet<double>(root, "s", 0.0));
    i.skew =
        YamlGet<double>(root, "cam_skew", YamlGet<double>(root, "skew", 0.0));
    i.atan_normalized = YamlGet<int>(root, "atan_normalized", 0) != 0;
    const std::string ocam_rel =
        YamlGet<std::string>(root, "ocam_calib_file", "");
    i.ocam_calib_file = JoinDir(yaml_path, ocam_rel);
    return Create(i);
}

bool LoadFromYaml(const std::string& yaml_path, AbstractCamera*& cam) {
    auto owned = LoadFromYaml(yaml_path);
    if (!owned) {
        cam = nullptr;
        return false;
    }
    cam = owned.release();
    return true;
}

bool Create(const CameraIntrinsics& intrinsics, AbstractCamera*& cam) {
    auto owned = Create(intrinsics);
    if (!owned) {
        cam = nullptr;
        return false;
    }
    cam = owned.release();
    return true;
}

}  // namespace camera
}  // namespace argus
