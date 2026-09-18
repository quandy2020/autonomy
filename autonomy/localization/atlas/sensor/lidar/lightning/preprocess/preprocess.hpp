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

#include "autonomy/localization/atlas/type.hpp"

#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace autonomy::localization::atlas {
namespace sensor {
namespace lightning {

//! Lidar preprocess (range / blind / voxel) — measurement only, not a SLAM.
class Preprocess {
public:
    struct Options {
        double min_range = 0.5;
        double max_range = 80.0;
        double blind = 0.1;
        double voxel_leaf = 0.2;
        int max_points = 20000;
    };

    Preprocess() = default;
    explicit Preprocess(Options options) : options_(std::move(options)) {}

    static Options FromYaml(const YAML::Node& node);

    [[nodiscard]] std::vector<Vec3_t> Run(
        const std::vector<Vec3_t>& points_body) const;

    const Options& options() const { return options_; }

private:
    Options options_;
};

}  // namespace lightning
}  // namespace sensor
}  // namespace autonomy::localization::atlas
