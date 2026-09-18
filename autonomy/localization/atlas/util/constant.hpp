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

//! Atlas-only gravity magnitude for ESKF / IMUInit.
//! Deg/rad → `autonomy::common::DegToRad` / `RadToDeg` (`common/math/math.hpp`).
//! SO(3) → `autonomy::common::math::{SkewSymmetricMatrix,RotationVectorToRotationMatrix,…}`.

namespace autonomy::localization::atlas {
namespace util {
namespace constant {

constexpr double kGRAVITY = 9.80665;

}  // namespace constant
}  // namespace util
}  // namespace autonomy::localization::atlas
