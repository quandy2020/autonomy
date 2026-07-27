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

#include "autonomy/map/strata/robot/visual/robot3d_presets.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace visual {

namespace {

Robot3DModelConfig WithModelUrl(const Robot3DModelConfig& base, const std::string& filename,
                                const ResourcePaths& paths) {
    Robot3DModelConfig config = base;
    if (filename.empty()) {
        return config;
    }
    if (filename.front() == '/') {
        config.modelUrl = filename;
    } else {
        config.modelUrl = paths.assetsUrl + "/models/" + filename;
    }
    return config;
}

}  // namespace

Robot3DModelConfig RobotExpressivePreset() {
    Robot3DModelConfig config;
    config.modelUrl = "/bicMap/assets/models/RobotExpressive.glb";
    config.scale = 3.0;
    config.metersScale = 1.0;
    config.rotateX = M_PI / 2.0;
    config.rotateY = 0.;
    config.rotateZ = 0.;
    config.defaultAnimation = "walk";
    config.animations = {
        {"idle", "Idle"},
        {"walk", "Walking"},
        {"dwell", "Standing"},
        {"run", "Running"},
        {"wave", "Wave"},
    };
    return config;
}

Robot3DModelConfig ModelConfigFromProfile(const core::RobotProfile& profile,
                                          const ResourcePaths& paths) {
    Robot3DModelConfig config = RobotExpressivePreset();
    if (!profile.display.model3D.empty()) {
        config = WithModelUrl(config, profile.display.model3D, paths);
    }
    return config;
}

}  // namespace visual
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
