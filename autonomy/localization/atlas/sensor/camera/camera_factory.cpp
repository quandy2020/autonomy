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
 * @file
 * @brief Implementations of CameraFactory::Create / CreateFromYaml / ParseModelType.
 */

#include "autonomy/localization/atlas/sensor/camera/camera_factory.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "autonomy/localization/atlas/sensor/camera/double_sphere.hpp"
#include "autonomy/localization/atlas/sensor/camera/equirectangular.hpp"
#include "autonomy/localization/atlas/sensor/camera/eucm.hpp"
#include "autonomy/localization/atlas/sensor/camera/fov.hpp"
#include "autonomy/localization/atlas/sensor/camera/kannala_brandt.hpp"
#include "autonomy/localization/atlas/sensor/camera/pinhole.hpp"
#include "autonomy/localization/atlas/sensor/camera/radial_division.hpp"
#include "autonomy/localization/atlas/sensor/camera/radtan.hpp"
#include "autonomy/localization/atlas/sensor/camera/ucm.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {
namespace {

std::string ToLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

double YamlOr(const YAML::Node& node, const char* key, double fallback) {
    if (node[key]) {
        return node[key].as<double>();
    }
    return fallback;
}

int YamlOrInt(const YAML::Node& node, const char* key, int fallback) {
    if (node[key]) {
        return node[key].as<int>();
    }
    return fallback;
}

}  // namespace

GeometricCamera::Type CameraFactory::ParseModelType(const std::string& name) {
    const std::string m = ToLower(name);
    if (m == "pinhole" || m == "ideal" || m == "none") {
        return GeometricCamera::Type::kPinhole;
    }
    if (m == "radtan" || m == "opencv" || m == "brown" || m == "perspective" ||
        m == "pinhole-radtan" || m == "pinhole_radtan") {
        return GeometricCamera::Type::kRadTan;
    }
    if (m == "kannala_brandt" || m == "kannala" || m == "fisheye" || m == "equi" ||
        m == "kb8" || m == "kannalabrandt8" || m == "pinhole-equi" ||
        m == "pinhole_equi") {
        return GeometricCamera::Type::kKannalaBrandt;
    }
    if (m == "fov" || m == "pinhole-fov" || m == "pinhole_fov") {
        return GeometricCamera::Type::kFov;
    }
    if (m == "ucm" || m == "mei" || m == "omni" || m == "omnidirectional") {
        return GeometricCamera::Type::kUcm;
    }
    if (m == "eucm" || m == "extended_unified") {
        return GeometricCamera::Type::kEucm;
    }
    if (m == "double_sphere" || m == "ds" || m == "doublesphere") {
        return GeometricCamera::Type::kDoubleSphere;
    }
    if (m == "equirectangular" || m == "equirect" || m == "panorama" ||
        m == "spherical") {
        return GeometricCamera::Type::kEquirectangular;
    }
    if (m == "radial_division" || m == "division" || m == "radialdivision") {
        return GeometricCamera::Type::kRadialDivision;
    }
    throw std::invalid_argument("Unknown camera model: " + name);
}

std::unique_ptr<GeometricCamera> CameraFactory::Create(
    const Intrinsics& intrinsics) {
    std::unique_ptr<GeometricCamera> cam;
    switch (ParseModelType(intrinsics.model)) {
        case GeometricCamera::Type::kPinhole:
            cam = std::make_unique<Pinhole>(intrinsics.fx, intrinsics.fy,
                                            intrinsics.cx, intrinsics.cy);
            break;
        case GeometricCamera::Type::kRadTan:
            cam = std::make_unique<RadTan>(
                intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy,
                intrinsics.k1, intrinsics.k2, intrinsics.p1, intrinsics.p2,
                intrinsics.k3);
            break;
        case GeometricCamera::Type::kKannalaBrandt:
            cam = std::make_unique<KannalaBrandt>(
                intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy,
                intrinsics.k1, intrinsics.k2, intrinsics.k3, intrinsics.k4);
            break;
        case GeometricCamera::Type::kFov:
            cam = std::make_unique<Fov>(intrinsics.fx, intrinsics.fy,
                                        intrinsics.cx, intrinsics.cy,
                                        intrinsics.w);
            break;
        case GeometricCamera::Type::kUcm:
            cam = std::make_unique<Ucm>(intrinsics.fx, intrinsics.fy,
                                        intrinsics.cx, intrinsics.cy,
                                        intrinsics.xi);
            break;
        case GeometricCamera::Type::kEucm:
            cam = std::make_unique<Eucm>(intrinsics.fx, intrinsics.fy,
                                         intrinsics.cx, intrinsics.cy,
                                         intrinsics.alpha, intrinsics.beta);
            break;
        case GeometricCamera::Type::kDoubleSphere:
            cam = std::make_unique<DoubleSphere>(
                intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy,
                intrinsics.xi, intrinsics.alpha);
            break;
        case GeometricCamera::Type::kEquirectangular:
            cam = std::make_unique<Equirectangular>(
                intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy,
                intrinsics.width, intrinsics.height);
            break;
        case GeometricCamera::Type::kRadialDivision:
            cam = std::make_unique<RadialDivision>(
                intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy,
                intrinsics.k);
            break;
    }
    if (cam && (intrinsics.width > 0 || intrinsics.height > 0)) {
        cam->set_image_size(intrinsics.width, intrinsics.height);
    }
    return cam;
}

std::unique_ptr<GeometricCamera> CameraFactory::CreateFromYaml(
    const YAML::Node& node) {
    // Accept either flat node or nested Camera: / camera: block.
    const YAML::Node cam =
        node["Camera"] ? node["Camera"]
                       : (node["camera"] ? node["camera"] : node);

    Intrinsics intrinsics;
    if (cam["model"]) {
        intrinsics.model = cam["model"].as<std::string>();
    } else if (cam["Camera.model"]) {
        intrinsics.model = cam["Camera.model"].as<std::string>();
    } else if (cam["Camera.type"]) {
        intrinsics.model = cam["Camera.type"].as<std::string>();
    }

    intrinsics.fx = YamlOr(cam, "fx", YamlOr(cam, "Camera.fx", 1.0));
    intrinsics.fy = YamlOr(cam, "fy", YamlOr(cam, "Camera.fy", intrinsics.fx));
    intrinsics.cx = YamlOr(cam, "cx", YamlOr(cam, "Camera.cx", 0.0));
    intrinsics.cy = YamlOr(cam, "cy", YamlOr(cam, "Camera.cy", 0.0));

    intrinsics.k1 = YamlOr(cam, "k1", YamlOr(cam, "Camera.k1", 0.0));
    intrinsics.k2 = YamlOr(cam, "k2", YamlOr(cam, "Camera.k2", 0.0));
    intrinsics.k3 = YamlOr(cam, "k3", YamlOr(cam, "Camera.k3", 0.0));
    intrinsics.k4 = YamlOr(cam, "k4", YamlOr(cam, "Camera.k4", 0.0));
    intrinsics.p1 = YamlOr(cam, "p1", YamlOr(cam, "Camera.p1", 0.0));
    intrinsics.p2 = YamlOr(cam, "p2", YamlOr(cam, "Camera.p2", 0.0));
    intrinsics.w = YamlOr(cam, "w", YamlOr(cam, "omega", 0.0));
    intrinsics.xi = YamlOr(cam, "xi", 0.0);
    intrinsics.alpha = YamlOr(cam, "alpha", 0.5);
    intrinsics.beta = YamlOr(cam, "beta", 1.0);
    intrinsics.k = YamlOr(cam, "k", YamlOr(cam, "distortion", 0.0));
    intrinsics.width = YamlOrInt(cam, "width", YamlOrInt(cam, "cols", 0));
    intrinsics.height = YamlOrInt(cam, "height", YamlOrInt(cam, "rows", 0));

    // OpenCV-style distortion vector d: [k1,k2,p1,p2,k3] or fisheye [k1..k4].
    if (cam["distortion"] && cam["distortion"].IsSequence()) {
        const auto& d = cam["distortion"];
        if (d.size() >= 1) {
            intrinsics.k1 = d[0].as<double>();
        }
        if (d.size() >= 2) {
            intrinsics.k2 = d[1].as<double>();
        }
        if (d.size() >= 3) {
            intrinsics.p1 = d[2].as<double>();
        }
        if (d.size() >= 4) {
            intrinsics.p2 = d[3].as<double>();
        }
        if (d.size() >= 5) {
            intrinsics.k3 = d[4].as<double>();
        }
        // Fisheye-style 4 coeffs → also fill k3,k4 for KB.
        if (d.size() == 4 && ToLower(intrinsics.model).find("fish") !=
                                 std::string::npos) {
            intrinsics.k3 = d[2].as<double>();
            intrinsics.k4 = d[3].as<double>();
            intrinsics.p1 = 0.0;
            intrinsics.p2 = 0.0;
        }
        if (d.size() >= 4 && (ToLower(intrinsics.model) == "kannala_brandt" ||
                              ToLower(intrinsics.model) == "kannala" ||
                              ToLower(intrinsics.model) == "equi" ||
                              ToLower(intrinsics.model) == "kb8")) {
            intrinsics.k3 = d[2].as<double>();
            intrinsics.k4 = d[3].as<double>();
            intrinsics.p1 = 0.0;
            intrinsics.p2 = 0.0;
        }
    }
    return Create(intrinsics);
}

}  // namespace camera
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
