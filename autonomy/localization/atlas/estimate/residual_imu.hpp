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

//! estimate/residual_imu — canonical IMU residual edges for Local/Global.
//! Re-exports real optimize/imu_g2o/preintegration_edge (not a stub).

#include "autonomy/localization/atlas/optimize/imu_g2o/preintegration_edge.hpp"

namespace autonomy::localization::atlas {
namespace estimate {

namespace residual_imu {
// Types exported from optimize/imu_g2o for joint BA consumers.
using PreintegrationEdge = optimize::imu_g2o::preintegration_edge;
}  // namespace residual_imu

}  // namespace estimate
}  // namespace autonomy::localization::atlas
