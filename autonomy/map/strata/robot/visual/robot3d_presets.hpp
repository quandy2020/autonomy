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

#pragma once

#include <cmath>
#include <string>
#include <unordered_map>

#include "autonomy/map/strata/robot/core/robot_profile.hpp"
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace visual {

/** Three.js RobotExpressive 预设，对齐 BICMap robot3DPresets.js。 */
Robot3DModelConfig RobotExpressivePreset();

/** 将 profile.display.model3D 映射为完整 3D 配置。 */
Robot3DModelConfig ModelConfigFromProfile(const core::RobotProfile& profile,
                                          const ResourcePaths& paths = {});

}  // namespace visual
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
