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

//! estimate/residual_vision — canonical vision residual edges for Local/Global.
//! Re-exports real optimize/internal/se3/* edge types (not stubs).

#include "autonomy/localization/atlas/optimize/internal/se3/perspective_reproj_edge.hpp"
#include "autonomy/localization/atlas/optimize/internal/se3/shot_vertex.hpp"

namespace autonomy::localization::atlas {
namespace estimate {

//! Canonical names for joint BA residual builders (vision family).
namespace residual_vision {
using ShotVertex = optimize::internal::se3::shot_vertex;
using MonoReprojEdge = optimize::internal::se3::mono_perspective_reproj_edge;
}  // namespace residual_vision

}  // namespace estimate
}  // namespace autonomy::localization::atlas
