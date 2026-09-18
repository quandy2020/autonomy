/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/localization/atlas/imu/config.hpp"

#include <stdexcept>
#include <vector>

namespace autonomy::localization::atlas {
namespace imu {
namespace {

Mat33_t read_mat33(const YAML::Node& node, const Mat33_t& fallback) {
    if (!node || !node.IsSequence() || node.size() != 9) {
        return fallback;
    }
    Mat33_t m;
    for (int i = 0; i < 9; ++i) {
        m(i / 3, i % 3) = node[i].as<double>();
    }
    return m;
}

Vec3_t read_vec3(const YAML::Node& node, const Vec3_t& fallback) {
    if (!node || !node.IsSequence() || node.size() != 3) {
        return fallback;
    }
    return Vec3_t(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

}  // namespace

config config::from_yaml(const YAML::Node& node) {
    config cfg;
    if (!node || !node.IsMap()) {
        return cfg;
    }

    cfg.enabled = node["enabled"].as<bool>(false);
    cfg.frequency = node["frequency"].as<double>(cfg.frequency);
    cfg.noise_acc = node["noise_acc"].as<double>(cfg.noise_acc);
    cfg.noise_gyro = node["noise_gyro"].as<double>(cfg.noise_gyro);
    cfg.random_walk_acc = node["random_walk_acc"].as<double>(cfg.random_walk_acc);
    cfg.random_walk_gyro = node["random_walk_gyro"].as<double>(cfg.random_walk_gyro);
    cfg.gravity_magnitude = node["gravity_magnitude"].as<double>(cfg.gravity_magnitude);
    cfg.buffer_capacity = node["buffer_capacity"].as<std::size_t>(cfg.buffer_capacity);

    cfg.R_c_b = read_mat33(node["R_c_b"], cfg.R_c_b);
    cfg.t_c_b = read_vec3(node["t_c_b"], cfg.t_c_b);
    cfg.time_offset = node["time_offset"].as<double>(cfg.time_offset);
    cfg.time_sync_slop = node["time_sync_slop"].as<double>(cfg.time_sync_slop);
    cfg.buffer_capacity = node["buffer_capacity"].as<std::size_t>(cfg.buffer_capacity);
    cfg.init_min_keyframes = node["init_min_keyframes"].as<unsigned int>(cfg.init_min_keyframes);

    // Optional 4x4 T_c_b (row-major 16 values) overrides R/t.
    if (node["T_c_b"] && node["T_c_b"].IsSequence() && node["T_c_b"].size() == 16) {
        Mat44_t T = Mat44_t::Identity();
        for (int i = 0; i < 16; ++i) {
            T(i / 4, i % 4) = node["T_c_b"][i].as<double>();
        }
        cfg.R_c_b = T.block<3, 3>(0, 0);
        cfg.t_c_b = T.block<3, 1>(0, 3);
    }

    if (cfg.frequency <= 0.0) {
        throw std::runtime_error("IMU.frequency must be > 0");
    }
    return cfg;
}

Mat44_t config::T_c_b() const {
    Mat44_t T = Mat44_t::Identity();
    T.block<3, 3>(0, 0) = R_c_b;
    T.block<3, 1>(0, 3) = t_c_b;
    return T;
}

}  // namespace imu
}  // namespace autonomy::localization::atlas
