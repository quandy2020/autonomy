/*
 * Copyright 2026 Autodriver contributors
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

#include "autodriver/common/calibration.hpp"

#include <yaml-cpp/yaml.h>

namespace autodriver {
namespace common {

bool LoadExtrinsicYaml(const std::string& path, Extrinsic* out,
                       std::string* error) {
    if (out == nullptr) {
        if (error) {
            *error = "null Extrinsic output";
        }
        return false;
    }
    try {
        const YAML::Node root = YAML::LoadFile(path);
        Extrinsic extrinsic;
        if (root["header"] && root["header"]["frame_id"]) {
            extrinsic.parent_frame = root["header"]["frame_id"].as<std::string>();
        } else if (root["frame_id"]) {
            extrinsic.parent_frame = root["frame_id"].as<std::string>();
        }
        if (root["child_frame_id"]) {
            extrinsic.child_frame = root["child_frame_id"].as<std::string>();
        }
        const YAML::Node tf = root["transform"] ? root["transform"] : root;
        double tx = 0;
        double ty = 0;
        double tz = 0;
        double qx = 0;
        double qy = 0;
        double qz = 0;
        double qw = 1;
        if (tf["translation"]) {
            tx = tf["translation"]["x"].as<double>(0.0);
            ty = tf["translation"]["y"].as<double>(0.0);
            tz = tf["translation"]["z"].as<double>(0.0);
        }
        if (tf["rotation"]) {
            qx = tf["rotation"]["x"].as<double>(0.0);
            qy = tf["rotation"]["y"].as<double>(0.0);
            qz = tf["rotation"]["z"].as<double>(0.0);
            qw = tf["rotation"]["w"].as<double>(1.0);
        }
        Eigen::Quaterniond q(qw, qx, qy, qz);
        q.normalize();
        extrinsic.transform = Eigen::Translation3d(tx, ty, tz) * q;
        *out = std::move(extrinsic);
        return true;
    } catch (const YAML::Exception& ex) {
        if (error) {
            *error = ex.what();
        }
        return false;
    }
}

}  // namespace common
}  // namespace autodriver
