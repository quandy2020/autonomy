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

//! sensor/lidar/preprocess — canonical lidar cloud filter (range / blind / voxel).

#include "autonomy/localization/atlas/type.hpp"

#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace autonomy::localization::atlas {
namespace sensor {

enum class LidarModel {
    kGeneric = 0,
    kVelodyne,
    kOuster,
    kLivox,
};

inline LidarModel LidarModelFromString(const std::string& s) {
    if (s == "velodyne" || s == "Velodyne" || s == "VELODYNE") {
        return LidarModel::kVelodyne;
    }
    if (s == "ouster" || s == "Ouster" || s == "OUSTER") {
        return LidarModel::kOuster;
    }
    if (s == "livox" || s == "Livox" || s == "LIVOX") {
        return LidarModel::kLivox;
    }
    return LidarModel::kGeneric;
}

struct TimedPoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec3_t p = Vec3_t::Zero();
    //! Relative time in scan [0,1]; 0 if unknown.
    double t_rel = 0.0;
};

//! Lidar preprocess (range / blind / voxel) — measurement only, not a SLAM.
class Preprocess {
public:
    struct Options {
        LidarModel model = LidarModel::kGeneric;
        double min_range = 0.5;
        double max_range = 80.0;
        double blind = 0.1;
        double voxel_leaf = 0.2;
        int max_points = 20000;
        //! When true, prefer RunTimed path (needs per-point times from driver).
        bool use_point_time = false;
    };

    Preprocess() = default;
    explicit Preprocess(Options options) : options_(std::move(options)) {}

    static Options FromYaml(const YAML::Node& node);

    [[nodiscard]] std::vector<Vec3_t> Run(
        const std::vector<Vec3_t>& points_body) const;

    //! Same filters as Run, preserving optional relative times.
    //! If point_time_rel empty or size mismatch, t_rel is left 0.
    [[nodiscard]] std::vector<TimedPoint> RunTimed(
        const std::vector<Vec3_t>& points_body,
        const std::vector<double>& point_time_rel) const;

    const Options& options() const { return options_; }

private:
    Options options_;
};

}  // namespace sensor
}  // namespace autonomy::localization::atlas
