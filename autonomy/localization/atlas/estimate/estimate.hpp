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

//! Shared residual builders for Local / Global Joint BA (single AtlasSystem).
//! §2b: residual_vision / residual_imu / residual_lidar / residual_odom.

#include "autonomy/localization/atlas/estimate/buffered_lidar_residual_source.hpp"
#include "autonomy/localization/atlas/estimate/lidar_residual_source.hpp"
#include "autonomy/localization/atlas/estimate/residual_imu.hpp"
#include "autonomy/localization/atlas/estimate/residual_lidar.hpp"
#include "autonomy/localization/atlas/estimate/residual_mask.hpp"
#include "autonomy/localization/atlas/estimate/residual_odom.hpp"
#include "autonomy/localization/atlas/estimate/residual_vision.hpp"

namespace autonomy::localization::atlas {
namespace estimate {

struct ResidualSetTag {};

}  // namespace estimate
}  // namespace autonomy::localization::atlas
